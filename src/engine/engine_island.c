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
#include "engine/engine_util_sparse.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif


//-------------------------- local utilities -------------------------------------------------------

// clear island-related arena pointers in mjData
static void clearIsland(mjData* d, size_t parena) {
#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS_ISLAND
#undef X
  d->nefc = 0;
  d->nisland = 0;
  d->nidof = 0;
  d->parena = parena;

  // poison remaining memory
#ifdef mjUSEASAN
  ASAN_POISON_MEMORY_REGION(
    (char*)d->arena + d->parena, d->narena - d->pstack - d->parena);
#endif
}


// allocate island arrays on arena, return 1 on success, 0 on failure
static int arenaAllocIsland(const mjModel* m, mjData* d) {
#undef MJ_M
#define MJ_M(n) m->n
#undef MJ_D
#define MJ_D(n) d->n

  size_t parena_old = d->parena;

#define X(type, name, nr, nc)                                                 \
  d->name = mj_arenaAllocByte(d, sizeof(type) * (nr) * (nc), _Alignof(type)); \
  if (!d->name) {                                                             \
    mj_warning(d, mjWARN_CNSTRFULL, d->narena);                               \
    clearIsland(d, parena_old);                                               \
    return 0;                                                                 \
  }

  MJDATA_ARENA_POINTERS_ISLAND

#undef X

#undef MJ_M
#define MJ_M(n) n
#undef MJ_D
#define MJ_D(n) n
  return 1;
}


//-------------------------- union-find and flood-fill  --------------------------------------------

// find the canonical root of an active tree and compress its path
int mj_dsuRoot(int* parent, int tree) {
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


// activate and union two incident trees; -1 denotes a static endpoint
void mj_dsuMerge(int* parent, int tree1, int tree2) {
  if (tree1 == -1 && tree2 == -1) {
    mjERROR("self-incidence of the static tree");  // SHOULD NOT OCCUR
    return;
  }

  if (tree1 == -1) tree1 = tree2;
  if (tree2 == -1) tree2 = tree1;

  if (parent[tree1] == -1) parent[tree1] = tree1;
  if (parent[tree2] == -1) parent[tree2] = tree2;

  if (parent[tree1] == parent[tree2]) return;

  int root1 = mj_dsuRoot(parent, tree1);
  int root2 = mj_dsuRoot(parent, tree2);
  if (root1 < root2) {
    parent[root2] = root1;
  } else if (root2 < root1) {
    parent[root1] = root2;
  }
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

  // contact
  else if (efc_type == mjCNSTR_CONTACT_FRICTIONLESS ||
           efc_type == mjCNSTR_CONTACT_PYRAMIDAL ||
           efc_type == mjCNSTR_CONTACT_ELLIPTIC) {
    int g1 = d->contact[efc_id].geom[0];
    int g2 = d->contact[efc_id].geom[1];

    // geom-geom contact
    if (g1 >= 0 && g2 >= 0) {
      iter->trees[0] = m->body_treeid[m->geom_bodyid[g1]];
      iter->trees[1] = m->body_treeid[m->geom_bodyid[g2]];
      if (iter->trees[0] < 0 && iter->trees[1] < 0) {
        mjERROR("contact %d is between two static bodies", efc_id);  // SHOULD NOT OCCUR
      }
    }

    // no shortcut for flex contacts: enable generic scan
    else {
      iter->jac_idx = 0;
    }
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
          m->eq_type[efc_id] == mjEQ_FLEXVERT ||
          m->eq_type[efc_id] == mjEQ_FLEXSTRAIN);
}


// activate and union all trees with direct incidence in a constraint
static const char* unionConstraintTrees(const mjModel* m, const mjData* d, int* parent,
                                        int* efc_tree, int* err_i) {
  int nefc = d->nefc;
  int efc_type = -1;
  int efc_id = -1;

  // iterate over constraints and union incident trees
  for (int i=0; i < nefc; i++) {
    // row i is still in the same constraint: skip it
    if (i > 0 && efc_type == d->efc_type[i] && efc_id == d->efc_id[i]) {
      // unless it is a flex equality, where the tree pattern changes per dof
      if (!isFlexEquality(m, efc_type, efc_id)) {
        efc_tree[i] = efc_tree[i-1];
        continue;
      }
    }
    efc_type = d->efc_type[i];
    efc_id = d->efc_id[i];

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
        mj_dsuMerge(parent, tree1, -1);
      } else {
        while (tree2 != -2) {
          mj_dsuMerge(parent, tree1, tree2);
          tree1 = tree2;
          tree2 = treeNext(m, d, i, &iter);
        }
      }
    } else {
      *err_i = i;
      return "no tree found for constraint %d";
    }
  }

  // flex stiffness couples all vertices (nodes for interpolated flexes) of a flex without any
  // constraint row representing the coupling: union the trees of every stiffness-active flex
  // (star around the first dynamic tree). This keeps the partition valid when the implicit
  // effective metric (mj_flexCG) carries the stiffness inside the constraint solve. Awake
  // trees only: sleeping trees must stay out of islands (mj_sleep invariant, matching the
  // constraint filter); waking a flex as a unit remains the wake machinery's job.
  for (int f=0; f < m->nflex; f++) {
    // mirror the stiffness-activity conditions of engine_derivative's flexStiff_active /
    // flexInterp_processed: deformable dim>=2 flex with bending or nonzero stiffness
    if (m->flex_rigid[f] || m->flex_dim[f] < 2) {
      continue;
    }
    int sadr = m->flex_stiffnessadr[f];
    if (m->flex_bendingadr[f] < 0 && (sadr < 0 || m->flex_stiffness[sadr] == 0)) {
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
        mj_dsuMerge(parent, tree1, tree2);
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

  // no constraints or islands disabled: quick return
  if (mjDISABLED(mjDSBL_ISLAND) || !nefc) {
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


  // ------------------------------------- trees ---------------------------------------------------

  // copy tree_island from stack to arena
  mju_copyInt(d->tree_island, tree_island, ntree);

  // compute island_ntree, number of trees per island
  mju_zeroInt(d->island_ntree, nisland);
  for (int i=0; i < ntree; i++) {
    int island = tree_island[i];
    if (island >= 0) {
      d->island_ntree[island]++;
    }
  }

  // compute island_itreeadr (cumsum of island_ntree)
  d->island_itreeadr[0] = 0;
  for (int i=1; i < nisland; i++) {
    d->island_itreeadr[i] = d->island_itreeadr[i-1] + d->island_ntree[i-1];
  }
  int last_tree = d->island_itreeadr[nisland-1] + d->island_ntree[nisland-1];

  // compute map_itree2tree
  int* island_ntree2 = mjSTACKALLOC(d, nisland + 1, int);  // last elem counts unconstrained trees
  mju_zeroInt(island_ntree2, nisland + 1);
  for (int i=0; i < ntree; i++) {
    int island = tree_island[i];
    if (island >= 0) {
      d->map_itree2tree[d->island_itreeadr[island] + island_ntree2[island]++] = i;
    } else {
      d->map_itree2tree[last_tree + island_ntree2[nisland]++] = i;
    }
  }

  // SHOULD NOT OCCUR
  if (!mju_compare(island_ntree2, d->island_ntree, nisland)) mjERROR("island_ntree miscount");
  if (last_tree + island_ntree2[nisland] != ntree) mjERROR("miscount of unconstrained trees");


  // ------------------------------------- degrees of freedom --------------------------------------

  // compute dof_island, island_nv
  mju_zeroInt(d->island_nv, nisland);
  for (int i=0; i < nv; i++) {
    // assign DOFs to islands
    int island = tree_island[m->dof_treeid[i]];  // -1 if unconstrained
    d->dof_island[i] = island;

    // increment island_nv
    if (island >= 0) {
      d->island_nv[island]++;
    }
  }

  // compute island_idofadr (cumsum of island_nv)
  d->island_idofadr[0] = 0;
  for (int i=1; i < nisland; i++) {
    d->island_idofadr[i] = d->island_idofadr[i-1] + d->island_nv[i-1];
  }

  // compute dof <-> idof maps
  int* island_nv2 = mjSTACKALLOC(d, nisland + 1, int);  // last element counts unconstrained DOFs
  mju_zeroInt(island_nv2, nisland + 1);
  for (int dof=0; dof < nv; dof++) {
    int island = d->dof_island[dof];
    int idof;
    if (island >= 0) {
      // constrained dof
      idof = d->island_idofadr[island] + island_nv2[island]++;
    } else {
      // unconstrained dof
      idof = nidof + island_nv2[nisland]++;
    }

    d->map_dof2idof[dof] = idof;
    d->map_idof2dof[idof] = dof;  // only the first nidof elements of map_idof2dof are in some island
  }

  // SHOULD NOT OCCUR
  if (!mju_compare(island_nv2, d->island_nv, nisland)) mjERROR("island_nv miscount");
  if (nidof + island_nv2[nisland] != nv) mjERROR("miscount of unconstrained dofs");

  // compute island_dofadr (used for visualization)
  for (int i=0; i < nisland; i++) {
    d->island_dofadr[i] = d->map_idof2dof[d->island_idofadr[i]];
  }


  // ------------------------------------- constraints ---------------------------------------------

  // compute efc_island, island_{ne,nf,nefc}
  mju_zeroInt(d->island_ne, nisland);
  mju_zeroInt(d->island_nf, nisland);
  mju_zeroInt(d->island_nefc, nisland);
  for (int i=0; i < nefc; i++) {
    d->efc_island[i] = tree_island[efc_tree[i]];
    int island = d->efc_island[i];
    d->island_nefc[island]++;
    switch (d->efc_type[i]) {
      case mjCNSTR_EQUALITY:
        d->island_ne[island]++;
        break;
      case mjCNSTR_FRICTION_DOF:
      case mjCNSTR_FRICTION_TENDON:
        d->island_nf[island]++;
        break;
      default:
        break;
    }
  }

  // compute island_iefcadr (cumsum of island_nefc)
  d->island_iefcadr[0] = 0;
  for (int i=1; i < nisland; i++) {
    d->island_iefcadr[i] = d->island_iefcadr[i-1] + d->island_nefc[i-1];
  }

  // compute efc <-> iefc maps
  int* island_nefc2 = island_nv2;  // reuse island_nv2
  mju_zeroInt(island_nefc2, nisland);
  for (int c=0; c < nefc; c++) {
    int island = d->efc_island[c];
    int ic = d->island_iefcadr[island] + island_nefc2[island]++;
    d->map_efc2iefc[c] = ic;
    d->map_iefc2efc[ic] = c;
    d->iefc_type[ic] = d->efc_type[c];
    d->iefc_id[ic] = d->efc_id[c];
    d->iefc_frictionloss[ic] = d->efc_frictionloss[c];
    d->iefc_D[ic] = d->efc_D[c];
    d->iefc_R[ic] = d->efc_R[c];
  }

  // SHOULD NOT OCCUR
  if (!mju_compare(island_nefc2, d->island_nefc, nisland)) mjERROR("island_nefc miscount");

  mj_freeStack(d);
}
