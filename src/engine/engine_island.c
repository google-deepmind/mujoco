// Copyright 2023 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "engine/engine_island.h"

#include <stdio.h>
#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjxmacro.h>
#include "engine/engine_core_util.h"
#include "engine/engine_memory.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif


//-------------------------- local utilities -------------------------------------------------------

// allocate island arrays on arena, return 1 on success, 0 on failure
static int arenaAllocIsland(const mjModel* m, mjData* d) {
#undef MJ_M
#define MJ_M(n) m->n
#undef MJ_D
#define MJ_D(n) d->n

  // check for single-island aliasing opportunities
  int single_efc = (d->nisland == 1 && d->nefc > 0);
  int all_dofs = (d->nisland == 1 && d->nidof == m->nv);

  // efc fields aliased directly to global buffers when nisland == 1
#define IS_SINGLE_EFC_ALIAS(field)                                   \
  (offsetof(mjData, field) == offsetof(mjData, map_iefc2efc)      || \
   offsetof(mjData, field) == offsetof(mjData, iefc_type)         || \
   offsetof(mjData, field) == offsetof(mjData, iefc_id)           || \
   offsetof(mjData, field) == offsetof(mjData, iefc_frictionloss) || \
   offsetof(mjData, field) == offsetof(mjData, iefc_D)            || \
   offsetof(mjData, field) == offsetof(mjData, iefc_R)            || \
   offsetof(mjData, field) == offsetof(mjData, iefc_aref)         || \
   offsetof(mjData, field) == offsetof(mjData, iefc_force))

#define X(type, name, nr, nc)                                                      \
  if (!((single_efc && IS_SINGLE_EFC_ALIAS(name)) ||                               \
        (all_dofs && offsetof(mjData, name) == offsetof(mjData, map_idof2dof)))) { \
    d->name = mj_arenaAllocByte(d, sizeof(type) * (nr) * (nc), _Alignof(type));    \
    if (!d->name) {                                                                \
      mj_warning(d, mjWARN_CNSTRFULL, d->narena);                                  \
      mj_clearEfc(d);                                                              \
      return 0;                                                                    \
    }                                                                              \
  }

  MJDATA_ARENA_POINTERS_ISLAND

#undef X
#undef IS_SINGLE_EFC_ALIAS

  // single island: alias identity maps and efc arrays directly into global arena buffers
  if (single_efc) {
    d->map_iefc2efc      = d->map_efc2iefc;
    d->iefc_type         = d->efc_type;
    d->iefc_id           = d->efc_id;
    d->iefc_frictionloss = d->efc_frictionloss;
    d->iefc_D            = d->efc_D;
    d->iefc_R            = d->efc_R;
    d->iefc_aref         = d->efc_aref;
    d->iefc_force        = d->efc_force;
  }

  // single island with all dofs: alias idof map directly to dof map
  if (all_dofs) {
    d->map_idof2dof      = d->map_dof2idof;
  }

#undef MJ_M
#define MJ_M(n) n
#undef MJ_D
#define MJ_D(n) n
  return 1;
}


//-------------------------- union-find and flood-fill  --------------------------------------------

// inline path-compressing root lookup
static inline int dsuRoot(int* parent, int tree) {
  int root = tree;
  while (parent[root] != root) {
    root = parent[root];
  }

  while (parent[tree] != tree) {
    int next = parent[tree];
    parent[tree] = root;
    tree = next;
  }

  return root;
}


// find the canonical root of an active tree and compress its path
int mj_dsuRoot(int* parent, int tree) {
  return dsuRoot(parent, tree);
}


// inline activation and union of two incident trees; -1 denotes a static endpoint
static inline void dsuMerge(int* parent, int tree1, int tree2) {
  if (tree1 == -1 && tree2 == -1) {
    mjERROR("self-incidence of the static tree");  // SHOULD NOT OCCUR
    return;
  }

  if (tree1 == -1) tree1 = tree2;
  if (tree2 == -1) tree2 = tree1;

  if (parent[tree1] == -1) parent[tree1] = tree1;
  if (parent[tree2] == -1) parent[tree2] = tree2;

  if (parent[tree1] == parent[tree2]) return;

  int root1 = dsuRoot(parent, tree1);
  int root2 = dsuRoot(parent, tree2);
  if (root1 < root2) {
    parent[root2] = root1;
  } else if (root2 < root1) {
    parent[root1] = root2;
  }
}


// activate and union two incident trees; -1 denotes a static endpoint
void mj_dsuMerge(int* parent, int tree1, int tree2) {
  dsuMerge(parent, tree1, tree2);
}


// assign deterministic island ids in ascending canonical-root order
int mj_dsuAssign(int* island, int* parent, const int* tree_dofnum, int ntree, int* nidof) {
  int nisland = 0;
  *nidof = 0;
  for (int tree=0; tree < ntree; tree++) {
    if (parent[tree] == -1) {
      island[tree] = -1;
      continue;
    }

    if (parent[tree] == tree) {
      island[tree] = nisland++;
    } else {
      // union always links the larger root to the smaller root. Since trees are visited in
      // ascending order, this predecessor has already been compressed and assigned an island.
      parent[tree] = parent[parent[tree]];
      island[tree] = island[parent[tree]];
    }
    *nidof += tree_dofnum[tree];
  }

  return nisland;
}


// find disjoint subgraphs ("islands") given sparse symmetric adjacency matrix
//   arguments:
//     island  (nr)   - island index assigned to vertex, -1 if vertex has no edges
//     nr             - number of rows/columns of adjacency matrix
//     rownnz  (nr)   - matrix row nonzeros
//     rowadr  (nr)   - matrix row addresses
//     colind  (nnz)  - matrix column indices
//     stack   (nnz)  - stack space
//   returns number of islands
//   note: column indices are not required to be unique or sorted
int mj_floodFill(int* island, int nr, const int* rownnz, const int* rowadr, const int* colind,
                 int* stack) {
  // initialize island count, set ids to -1
  int nisland = 0;
  mju_fillInt(island, -1, nr);

  // iterate over vertices, discover islands
  for (int i=0; i < nr; i++) {
    // vertex already in island or singleton with no edges: skip
    if (island[i] != -1 || !rownnz[i]) {
      continue;
    }

    // push i onto stack
    int nstack = 0;
    stack[nstack++] = i;

    // DFS traversal of island
    while (nstack) {
      // pop v from stack
      int v = stack[--nstack];

      // if v is already assigned, continue
      if (island[v] != -1) {
        continue;
      }

      // assign v to current island
      island[v] = nisland;

      // push adjacent vertices onto stack
      mju_copyInt(stack + nstack, colind + rowadr[v], rownnz[v]);
      nstack += rownnz[v];
    }

    // island is filled: increment nisland
    nisland++;
  }

  return nisland;
}


// state of iterator for finding trees involved in a constraint
typedef struct {
  int trees[2];   // pre-calculated trees (special-cased constraints); -2: empty/sentinel
  int jac_idx;    // generic scan: current lookup index in Jacobian row; -1: scan disabled
  int tree_prev;  // generic scan: previous tree in ongoing scan
} mjTreeIter;


// return next tree of constraint i from iterator; -2: no more trees
static int treeNext(const mjModel* m, const mjData* d, int i, mjTreeIter* iter) {
  // handle special cases
  if (iter->trees[0] != -2) {
    // get first tree, queue up second tree, return first tree
    int tree = iter->trees[0];
    iter->trees[0] = iter->trees[1];
    iter->trees[1] = -2;
    return tree;
  }

  // special case mode complete
  if (iter->jac_idx == -1) {
    return -2;
  }

  // generic scan mode
  int j;
  int tree_next = -2;

  // sparse
  if (mj_isSparse(m)) {
    int rownnz = d->efc_J_rownnz[i];
    const int* colind = d->efc_J_colind + d->efc_J_rowadr[i];
    for (j = iter->jac_idx; j < rownnz; j++) {
      int tree_j = m->dof_treeid[colind[j]];
      if (tree_j != iter->tree_prev) {
        // found new tree
        tree_next = tree_j;
        break;
      }
    }
  }

  // dense
  else {
    int nv = m->nv;
    const mjtNum* J = d->efc_J + nv * i;
    for (j = iter->jac_idx; j < nv; j++) {
      if (J[j]) {
        int tree_j = m->dof_treeid[j];
        if (tree_j != iter->tree_prev) {
          // found new tree
          tree_next = tree_j;
          break;
        }

        // skip to end of tree's dof block
        j = m->tree_dofadr[tree_j] + m->tree_dofnum[tree_j] - 1;
      }
    }
  }

  // update iterator state
  iter->jac_idx = j;
  if (tree_next != -2) {
    iter->tree_prev = tree_next;
  }

  return tree_next;
}


// initialize tree iterator, handle special cases
static void treeIterInit(const mjModel* m, const mjData* d, int i, mjTreeIter* iter) {
  iter->trees[0] = -2;
  iter->trees[1] = -2;
  iter->jac_idx = -1;
  iter->tree_prev = -1;

  int efc_type = d->efc_type[i];
  int efc_id = d->efc_id[i];

  // ==== special cases: fill iter->trees where possible

  // joint friction
  if (efc_type == mjCNSTR_FRICTION_DOF) {
    iter->trees[0] = m->dof_treeid[efc_id];
  }

  // joint limit
  else if (efc_type == mjCNSTR_LIMIT_JOINT) {
    iter->trees[0] = m->dof_treeid[m->jnt_dofadr[efc_id]];
  }

  // connect or weld constraints
  else if (efc_type == mjCNSTR_EQUALITY &&
           (m->eq_type[efc_id] == mjEQ_CONNECT ||
            m->eq_type[efc_id] == mjEQ_WELD)) {
    int b1 = m->eq_obj1id[efc_id];
    int b2 = m->eq_obj2id[efc_id];

    // get body ids if using site semantics
    if (m->eq_objtype[efc_id] == mjOBJ_SITE) {
      b1 = m->site_bodyid[b1];
      b2 = m->site_bodyid[b2];
    }

    // get trees
    iter->trees[0] = m->body_treeid[b1];
    iter->trees[1] = m->body_treeid[b2];
    if (iter->trees[0] < 0 && iter->trees[1] < 0) {
      mjERROR("equality %d is between two static bodies", efc_id);  // SHOULD NOT OCCUR
    }
  }

  // otherwise enable generic scan
  else {
    iter->jac_idx = 0;
  }
}


// return whether repeated scalar rows of this constraint require separate tree scans
static int isFlexEquality(const mjModel* m, int efc_type, int efc_id) {
  return efc_type == mjCNSTR_EQUALITY &&
         (m->eq_type[efc_id] == mjEQ_FLEX ||
          m->eq_type[efc_id] == mjEQ_FLEXSTRAIN);
}


// activate and union the trees of the bodies of a contact, return the first tree, -1 if none:
// with flexes a contact can span many trees, and the values of its Jacobian rows can vanish
// (e.g. contact point on a hinge axis), so the trees cannot be found by scanning the rows
static int unionContactTrees(const mjModel* m, const mjData* d, int* parent,
                             const mjContact* con) {
  // geom-geom: trees of the two geom bodies
  if (con->geom[0] >= 0 && con->geom[1] >= 0) {
    int tree1 = m->body_treeid[m->geom_bodyid[con->geom[0]]];
    int tree2 = m->body_treeid[m->geom_bodyid[con->geom[1]]];
    if (tree1 < 0) {
      tree1 = tree2;
      tree2 = -1;
    }
    if (tree1 >= 0) {
      if (tree2 < 0 || tree1 == tree2) {
        // activate single dynamic tree
        if (parent[tree1] == -1) parent[tree1] = tree1;
      } else {
        dsuMerge(parent, tree1, tree2);
      }
    }
    return tree1;
  }

  // flex: get bodies of both sides
  int body[2*mjMAXFLEXINTERPW];
  int nb = mj_contactBodyWeight(m, d, con, 0, body, NULL);
  nb += mj_contactBodyWeight(m, d, con, 1, body+nb, NULL);

  // activate the first tree, union the others with it, skip static bodies
  int tree1 = -1;
  for (int j=0; j < nb; j++) {
    int tree2 = m->body_treeid[body[j]];
    if (tree2 < 0 || tree2 == tree1) {
      continue;
    }
    if (tree1 < 0) {
      tree1 = tree2;
      if (parent[tree1] == -1) parent[tree1] = tree1;
    } else {
      dsuMerge(parent, tree1, tree2);
    }
  }

  return tree1;
}


// union the awake trees of all dofs in tendon i's Jacobian: the static ten_J sparsity is
// the coupling footprint of the tendon's metric term, whatever the number of path trees
static void unionTendonTrees(const mjModel* m, const mjData* d, int* parent, int i) {
  int start = m->ten_J_rowadr[i], end = start + m->ten_J_rownnz[i];
  int tree1 = -1;
  for (int j=start; j < end; j++) {
    int tree2 = m->dof_treeid[m->ten_J_colind[j]];
    if (!d->tree_awake[tree2]) {
      continue;
    }
    if (tree1 < 0) {
      tree1 = tree2;
    } else if (tree1 != tree2) {
      dsuMerge(parent, tree1, tree2);
    }
  }
}


// activate and union all trees with direct incidence in a constraint
static const char* unionConstraintTrees(const mjModel* m, const mjData* d, int* restrict parent,
                                        int* restrict efc_tree, int* err_i) {
  int nefc = d->nefc;
  const int* restrict efc_type_arr = d->efc_type;
  const int* restrict efc_id_arr = d->efc_id;
  const mjContact* restrict contact = d->contact;
  int efc_type = -1;
  int efc_id = -1;

  // iterate over constraints and union incident trees
  for (int i=0; i < nefc; i++) {
    // row i is still in the same constraint: skip it
    if (i > 0 && efc_type == efc_type_arr[i] && efc_id == efc_id_arr[i]) {
      // unless it is a flex equality, where the tree pattern changes per dof
      if (!isFlexEquality(m, efc_type, efc_id)) {
        efc_tree[i] = efc_tree[i-1];
        continue;
      }
    }
    efc_type = efc_type_arr[i];
    efc_id = efc_id_arr[i];

    // contact: union the trees of its bodies and fill all contiguous rows of this contact
    if (efc_type == mjCNSTR_CONTACT_FRICTIONLESS ||
        efc_type == mjCNSTR_CONTACT_PYRAMIDAL ||
        efc_type == mjCNSTR_CONTACT_ELLIPTIC) {
      int tree = unionContactTrees(m, d, parent, contact + efc_id);
      if (tree < 0) {
        *err_i = efc_id;
        return "contact %d is between two static bodies";
      }
      efc_tree[i] = tree;

      // fill remaining contiguous rows of the same contact
      while (i + 1 < nefc && efc_id_arr[i+1] == efc_id && efc_type_arr[i+1] == efc_type) {
        efc_tree[++i] = tree;
      }
      continue;
    }

    // initialize tree iterator
    mjTreeIter iter;
    treeIterInit(m, d, i, &iter);

    // iterate over trees involved in constraint i
    int tree1 = treeNext(m, d, i, &iter);
    if (tree1 != -2) {
      int tree2 = treeNext(m, d, i, &iter);
      // assign tree to constraint, one of (tree1, tree2) must be non-negative
      efc_tree[i] = tree1 >= 0 ? tree1 : tree2;
      if (efc_tree[i] < 0) {
        *err_i = i;
        return "constraint %d is between two static bodies";
      }

      // activate a singleton or union all trees in a multi-tree constraint
      if (tree2 == -2) {
        if (parent[tree1] == -1) parent[tree1] = tree1;
      } else {
        while (tree2 != -2) {
          dsuMerge(parent, tree1, tree2);
          tree1 = tree2;
          tree2 = treeNext(m, d, i, &iter);
        }
      }
    } else {
      *err_i = i;
      return "no tree found for constraint %d";
    }
  }

  // multi-tree metric couplings only merge distinct dynamic trees; nothing to union for <= 1 tree
  if (m->ntree < 2) {
    return NULL;
  }

  // flexes with metric terms couple all their vertices (nodes for interpolated flexes) without
  // any constraint row representing the coupling: union the trees of every such flex
  // (star around the first dynamic tree). This keeps the partition valid when the implicit
  // effective metric (integrator=discrete) carries the coupling inside the constraint solve. Awake
  // trees only: sleeping trees must stay out of islands (mj_sleep invariant, matching the
  // constraint filter); waking a flex as a unit remains the wake machinery's job.
  for (int f=0; f < m->nflex; f++) {
    if (!mj_effFlexPossible(m, f)) {
      continue;
    }

    int num, adr;
    const int* bodyid;
    if (m->flex_interp[f]) {
      num = m->flex_nodenum[f];
      adr = m->flex_nodeadr[f];
      bodyid = m->flex_nodebodyid;
    } else {
      num = m->flex_vertnum[f];
      adr = m->flex_vertadr[f];
      bodyid = m->flex_vertbodyid;
    }

    int tree1 = -1;
    for (int j=0; j < num; j++) {
      int tree2 = m->body_treeid[bodyid[adr+j]];
      if (tree2 < 0 || tree2 == tree1 || !d->tree_awake[tree2]) {
        continue;
      }
      if (tree1 < 0) {
        tree1 = tree2;
      } else {
        dsuMerge(parent, tree1, tree2);
      }
    }
    if (mj_rodHasTwist(m, f)) {
      int nedge = (int)m->flex_bending[m->flex_bendingadr[f]] + 1;
      for (int e=0; e < nedge; e++) {
        int jnt = mj_rodChainTwistJnt(m, f, e);
        if (jnt < 0) {
          continue;
        }
        int tree2 = m->body_treeid[m->jnt_bodyid[jnt]];
        if (tree2 < 0 || tree2 == tree1 || !d->tree_awake[tree2]) {
          continue;
        }
        if (tree1 < 0) {
          tree1 = tree2;
        } else {
          dsuMerge(parent, tree1, tree2);
        }
      }
    }
  }

  // passive flex contacts (exclude == 4) also carry metric terms without efc rows: union the
  // awake trees of their bodies so two flexes (or a flex and a rigid body) coupled by passive
  // contact share one island
  if (m->nflex && mj_isMetric(m)) {
    for (int i=0; i < d->ncon; i++) {
      const mjContact* con = d->contact + i;
      if (con->exclude != 4) {
        continue;
      }
      int body[2*mjMAXFLEXINTERPW];
      int nb = mj_contactBodyWeight(m, d, con, 0, body, NULL);
      nb += mj_contactBodyWeight(m, d, con, 1, body+nb, NULL);

      int tree1 = -1;
      for (int j=0; j < nb; j++) {
        int tree2 = m->body_treeid[body[j]];
        if (tree2 < 0 || tree2 == tree1 || !d->tree_awake[tree2]) {
          continue;
        }
        if (tree1 < 0) {
          tree1 = tree2;
        } else {
          dsuMerge(parent, tree1, tree2);
        }
      }
    }
  }

  // discrete metric: tendon stiffness/damping couples every tree in the tendon's path
  // with no constraint row (mj_effTendonPossible); the coupling footprint is the static
  // ten_J sparsity, covering tendons that span any number of trees. Awake trees only: the
  // pre-existing sleep policy (mj_setConst) keeps metric-coupled trees awake. Under the
  // dual solver these classes are excluded from the metric (mj_effCouplings): no
  // couplings, no unions
  if (mj_effCouplings(m)) {
    for (int i=0; i < m->ntendon; i++) {
      if (m->tendon_treenum[i] < 2 || !mj_effTendonPossible(m, i)) {
        continue;
      }
      unionTendonTrees(m, d, parent, i);
    }

    // actuator gains couple every tree their transmission touches (mj_effActuatorPossible);
    // static transmission targets, mirroring the sleep-policy sweep in mj_setConst
    for (int i=0; i < m->nactuator; i++) {
      if (!mj_effActuatorPossible(m, i)) {
        continue;
      }
      int tree1 = -1, tree2 = -1;
      int tid = m->actuator_trnid[2*i];
      int tid2 = m->actuator_trnid[2*i + 1];
      switch ((mjtTrn) m->actuator_trntype[i]) {
      case mjTRN_TENDON:
        unionTendonTrees(m, d, parent, tid);
        break;
      case mjTRN_SITE:
      case mjTRN_SLIDERCRANK:
        tree1 = m->body_treeid[m->site_bodyid[tid]];
        if (tid2 >= 0) {
          tree2 = m->body_treeid[m->site_bodyid[tid2]];
        }
        break;
      case mjTRN_SO3:
        if (tid2 >= 0) {
          tree1 = m->body_treeid[m->site_bodyid[tid]];
          tree2 = m->body_treeid[m->site_bodyid[tid2]];
        }
        break;
      default:
        break;  // joint, jointinparent, body: single tree, no edge
      }
      if (tree1 >= 0 && tree2 >= 0 && tree1 != tree2 &&
          d->tree_awake[tree1] && d->tree_awake[tree2]) {
        dsuMerge(parent, tree1, tree2);
      }
    }
  }

  return NULL;
}


//-------------------------- main entry-point  -----------------------------------------------------

// discover islands:
//   nisland, island_idofadr, dof_island, dof_islandnext, island_efcadr, efc_island, efc_islandnext
void mj_island(const mjModel* m, mjData* d) {
  int nv = m->nv, nefc = d->nefc, ntree = m->ntree;

  // no constraints (and no sleeping metric-coupled flexes) or islands disabled: quick return
  if (mjDISABLED(mjDSBL_ISLAND) ||
      (!nefc && !(mjENABLED(mjENBL_SLEEP) && mj_isMetric(m) && mj_effFlexAny(m)))) {
    d->nisland = d->nidof = 0;
    return;
  }

  mj_markStack(d);

  // union direct tree incidence and assign deterministic components
  int* efc_tree = mjSTACKALLOC(d, nefc, int);
  int* parent = mjSTACKALLOC(d, ntree, int);
  mju_fillInt(parent, -1, ntree);
  int err_i = -1;
  const char* err_msg = unionConstraintTrees(m, d, parent, efc_tree, &err_i);
  if (err_msg) {
    mj_freeStack(d);
    mjERROR(err_msg, err_i);
  }
  int* tree_island = mjSTACKALLOC(d, ntree, int);
  int nidof;
  d->nisland = mj_dsuAssign(tree_island, parent, m->tree_dofnum, ntree, &nidof);

  // no islands found: quick return
  if (!d->nisland) {
    d->nidof = 0;
    mj_freeStack(d);
    return;
  }

  d->nidof = nidof;

  // allocate island arrays on arena
  if (!arenaAllocIsland(m, d)) {
    mj_freeStack(d);
    return;
  }

  // local copy
  int nisland = d->nisland;


  // ------------------------------------- trees & degrees of freedom ------------------------------

  // single island containing all dofs: all maps are identity
  if (nisland == 1 && nidof == nv) {
    // trees
    mju_zeroInt(d->tree_island, ntree);
    d->island_ntree[0] = ntree;
    d->island_itreeadr[0] = 0;
    int* restrict map_itree2tree = d->map_itree2tree;
    for (int i=0; i < ntree; i++) {
      map_itree2tree[i] = i;
    }

    // degrees of freedom
    mju_zeroInt(d->dof_island, nv);
    d->island_nv[0] = nv;
    d->island_idofadr[0] = 0;
    d->island_dofadr[0] = 0;
    int* restrict map_dof2idof = d->map_dof2idof;
    for (int i=0; i < nv; i++) {
      map_dof2idof[i] = i;
    }
  }

  // multiple islands or unconstrained dofs
  else {
    // copy tree_island from stack to arena
    mju_copyInt(d->tree_island, tree_island, ntree);

    // compute island_ntree and island_nv (number of trees and DOFs per island)
    int* restrict island_ntree = d->island_ntree;
    int* restrict island_nv = d->island_nv;
    const int* restrict tree_dofadr = m->tree_dofadr;
    const int* restrict tree_dofnum = m->tree_dofnum;
    mju_zeroInt(island_ntree, nisland);
    mju_zeroInt(island_nv, nisland);
    for (int i=0; i < ntree; i++) {
      int island = tree_island[i];
      if (island >= 0) {
        island_ntree[island]++;
        island_nv[island] += tree_dofnum[i];
      }
    }

    // compute island_itreeadr and island_idofadr (cumsum of island_ntree and island_nv)
    int* restrict island_itreeadr = d->island_itreeadr;
    int* restrict island_idofadr = d->island_idofadr;
    island_itreeadr[0] = 0;
    island_idofadr[0] = 0;
    for (int i=1; i < nisland; i++) {
      island_itreeadr[i] = island_itreeadr[i-1] + island_ntree[i-1];
      island_idofadr[i] = island_idofadr[i-1] + island_nv[i-1];
    }
    int last_tree = island_itreeadr[nisland-1] + island_ntree[nisland-1];

    // compute map_itree2tree, dof_island, and dof <-> idof maps in tree blocks
    int* restrict tree_cursor = mjSTACKALLOC(d, nisland+1, int);  // last elem: unconstrained trees
    int* restrict dof_cursor = mjSTACKALLOC(d, nisland+1, int);   // last elem: unconstrained DOFs
    mju_copyInt(tree_cursor, island_itreeadr, nisland);
    tree_cursor[nisland] = last_tree;
    mju_copyInt(dof_cursor, island_idofadr, nisland);
    dof_cursor[nisland] = nidof;

    int* restrict map_itree2tree = d->map_itree2tree;
    int* restrict dof_island = d->dof_island;
    int* restrict map_dof2idof = d->map_dof2idof;
    int* restrict map_idof2dof = d->map_idof2dof;
    for (int t=0; t < ntree; t++) {
      // append tree to its island (or unconstrained tail)
      int island = tree_island[t];
      int slot = island >= 0 ? island : nisland;
      map_itree2tree[tree_cursor[slot]++] = t;

      // assign tree dofs to island and populate dof <-> idof maps
      int adr = tree_dofadr[t];
      int num = tree_dofnum[t];
      int idof0 = dof_cursor[slot];
      dof_cursor[slot] = idof0 + num;
      for (int k=0; k < num; k++) {
        dof_island[adr+k] = island;
        map_dof2idof[adr+k] = idof0 + k;
        map_idof2dof[idof0+k] = adr + k;  // only the first nidof elements are in some island
      }
    }

    // verify unconstrained counts
    if (tree_cursor[nisland] != ntree) {
      mjERROR("miscount of unconstrained trees");  // SHOULD NOT OCCUR
    }
    if (dof_cursor[nisland] != nv) {
      mjERROR("miscount of unconstrained dofs");  // SHOULD NOT OCCUR
    }

    // compute island_dofadr (used for visualization)
    for (int i=0; i < nisland; i++) {
      d->island_dofadr[i] = map_idof2dof[island_idofadr[i]];
    }
  }


  // ------------------------------------- constraints ---------------------------------------------

  // single island: all constraints are in island 0, maps are identity, iefc_* aliased to efc_*
  if (nisland == 1) {
    mju_zeroInt(d->efc_island, nefc);
    d->island_ne[0] = d->ne;
    d->island_nf[0] = d->nf;
    d->island_nefc[0] = nefc;
    d->island_iefcadr[0] = 0;

    // identity constraint map
    int* restrict map_efc2iefc = d->map_efc2iefc;
    for (int c=0; c < nefc; c++) {
      map_efc2iefc[c] = c;
    }
  }

  // multiple islands
  else {
    // compute efc_island, island_{ne,nf,nefc}
    int ne = d->ne, nf = d->nf;
    int* restrict efc_island = d->efc_island;
    int* restrict island_ne = d->island_ne;
    int* restrict island_nf = d->island_nf;
    int* restrict island_nefc = d->island_nefc;
    mju_zeroInt(island_ne, nisland);
    mju_zeroInt(island_nf, nisland);
    mju_zeroInt(island_nefc, nisland);

    // equality constraints: island_ne and island_nefc
    for (int i=0; i < ne; i++) {
      int island = tree_island[efc_tree[i]];
      efc_island[i] = island;
      island_ne[island]++;
      island_nefc[island]++;
    }

    // friction constraints: island_nf and island_nefc
    for (int i=ne; i < ne+nf; i++) {
      int island = tree_island[efc_tree[i]];
      efc_island[i] = island;
      island_nf[island]++;
      island_nefc[island]++;
    }

    // remaining constraints (limits and contacts): island_nefc
    for (int i=ne+nf; i < nefc; i++) {
      int island = tree_island[efc_tree[i]];
      efc_island[i] = island;
      island_nefc[island]++;
    }

    // compute island_iefcadr (cumsum of island_nefc)
    int* restrict island_iefcadr = d->island_iefcadr;
    island_iefcadr[0] = 0;
    for (int i=1; i < nisland; i++) {
      island_iefcadr[i] = island_iefcadr[i-1] + island_nefc[i-1];
    }

    // compute efc <-> iefc maps
    int* restrict efc_cursor = mjSTACKALLOC(d, nisland, int);
    mju_copyInt(efc_cursor, island_iefcadr, nisland);
    int* restrict map_efc2iefc = d->map_efc2iefc;
    int* restrict map_iefc2efc = d->map_iefc2efc;
    for (int c=0; c < nefc; c++) {
      int ic = efc_cursor[efc_island[c]]++;
      map_efc2iefc[c] = ic;
      map_iefc2efc[ic] = c;
    }

    // gather efc arrays into island order (sequential writes)
    int* restrict iefc_type = d->iefc_type;
    int* restrict iefc_id = d->iefc_id;
    const int* restrict efc_type = d->efc_type;
    const int* restrict efc_id = d->efc_id;
    for (int ic=0; ic < nefc; ic++) {
      int c = map_iefc2efc[ic];
      iefc_type[ic] = efc_type[c];
      iefc_id[ic] = efc_id[c];
    }

    // gather frictionloss (or zero if no friction constraints)
    if (nf) {
      mju_gather(d->iefc_frictionloss, d->efc_frictionloss, map_iefc2efc, nefc);
    } else {
      mju_zero(d->iefc_frictionloss, nefc);
    }

    // gather D and R (metric integrators compute and refresh them in mj_regularizeConstraint)
    if (!mj_isMetric(m)) {
      mju_gather(d->iefc_D, d->efc_D, map_iefc2efc, nefc);
      mju_gather(d->iefc_R, d->efc_R, map_iefc2efc, nefc);
    }
  }

  mj_freeStack(d);
}
