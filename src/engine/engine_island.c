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
#include <string.h>

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
#ifdef ADDRESS_SANITIZER
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


//-------------------------- flood-fill and graph construction  ------------------------------------

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


// add 0, 1 or 2 edges to uncompressed CSR adjacency matrix
//   increment rownnz using tree_tree to de-dupe; return number of edges added
static int addEdge(int* rownnz, int* colind, mjtByte* tree_tree, int ntree, int tree1, int tree2) {
  if (tree1 == -1 && tree2 == -1) {
    mjERROR("self-edge of the static tree");  // SHOULD NOT OCCUR
    return 0;
  }

  // handle static trees (treat as self-edge)
  if (tree1 == -1) tree1 = tree2;
  if (tree2 == -1) tree2 = tree1;

  // skip if edge already present
  if (tree_tree[tree1*ntree + tree2]) {
    return 0;
  }

  // add edge
  tree_tree[tree1*ntree + tree2] = 1;
  colind[tree1*ntree + rownnz[tree1]++] = tree2;  // uncompressed format, rowadr is known

  // add flipped edge (off-diagonal)
  if (tree1 != tree2) {
    tree_tree[tree2*ntree + tree1] = 1;
    colind[tree2*ntree + rownnz[tree2]++] = tree1;  // uncompressed format, rowadr is known
    return 2;
  }

  return 1;
}


// find tree-tree edges (column indices), return total number of edges
//   efc_tree: first nonegative tree index of each constraint
static int findEdges(const mjModel* m, const mjData* d,
                     int* rownnz, int* colind, mjtByte* tree_tree, int* efc_tree, int ntree) {
  int nefc = d->nefc;
  int nnz = 0;
  int efc_type = -1;
  int efc_id = -1;

  // clear row nonzeros
  mju_zeroInt(rownnz, ntree);

  // iterate over constraints, compute tree-tree edges, assign efc_tree
  for (int i=0; i < nefc; i++) {
    // row i is still in the same constraint: skip it,
    if (efc_type == d->efc_type[i] && efc_id == d->efc_id[i]) {
      // unless it is a flex equality, where the tree pattern changes per dof
      if (!(efc_type == mjCNSTR_EQUALITY &&
            (m->eq_type[efc_id] == mjEQ_FLEX ||
             m->eq_type[efc_id] == mjEQ_FLEXVERT))) {
        // copy tree assignment from previous constraint and continue
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
        mjERROR("constraint %d is between two static bodies", i);  // SHOULD NOT OCCUR
      }

      // add one edge or continue to search for more edges
      if (tree2 == -2) {
        nnz += addEdge(rownnz, colind, tree_tree, ntree, tree1, -1);
      } else {
        while (tree2 != -2) {
          nnz += addEdge(rownnz, colind, tree_tree, ntree, tree1, tree2);
          tree1 = tree2;
          tree2 = treeNext(m, d, i, &iter);
        }
      }
    } else {
      mjERROR("no tree found for constraint %d", i);  // SHOULD NOT OCCUR
    }
  }

  return nnz;
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

  // dense tree-tree adjacency matrix
  int ntree2 = ntree * ntree;
  mjtByte* tree_tree = mjSTACKALLOC(d, ntree2, mjtByte);
  memset(tree_tree, 0, ntree2);

  // CSR representation of tree-tree adjacency matrix (uncompressed)
  int* colind = mjSTACKALLOC(d, ntree2, int);
  int* rownnz = mjSTACKALLOC(d, ntree, int);
  int* rowadr = mjSTACKALLOC(d, ntree, int);
  for (int r=0; r < ntree; r++) {
    rowadr[r] = r * ntree;
  }

  // first non-negative tree index of each constraint, used later for computing efc_island
  int* efc_tree = mjSTACKALLOC(d, nefc, int);

  // compute tree-tree adjacency matrix: fill rownnz and colind
  int nnz = findEdges(m, d, rownnz, colind, tree_tree, efc_tree, ntree);

  // discover islands
  int* tree_island = mjSTACKALLOC(d, ntree, int);
  int* stack = mjSTACKALLOC(d, nnz, int);
  d->nisland = mj_floodFill(tree_island, ntree, rownnz, rowadr, colind, stack);

  // no islands found: quick return
  if (!d->nisland) {
    d->nidof = 0;
    mj_freeStack(d);
    return;
  }

  // count nidof: total number of dofs in islands
  int nidof = 0;
  for (int i=0; i < ntree; i++) {
    if (tree_island[i] >= 0) {
      nidof += m->tree_dofnum[i];
    }
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
    // assign dofs to islands
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
  int* island_nv2 = mjSTACKALLOC(d, nisland + 1, int);  // last element counts unconstrained dofs
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

  // inertia: block-diagonalize both iLD <- qLD and iM <- qM
  mju_blockDiagSparse(d->iLD, d->iM_rownnz, d->iM_rowadr, d->iM_colind,
                      d->qLD,  m->M_rownnz, m->M_rowadr, m->M_colind,
                      nidof, nisland,
                      d->map_idof2dof, d->map_dof2idof,
                      d->island_idofadr, d->island_idofadr,
                      d->iM, d->M);
  mju_gather(d->iLDiagInv, d->qLDiagInv, d->map_idof2dof, nidof);


  // ------------------------------------- constraints ---------------------------------------------

  // compute efc_island, island_{ne,nf,nefc}
  mju_zeroInt(d->island_ne, nisland);
  mju_zeroInt(d->island_nf, nisland);
  mju_zeroInt(d->island_nefc, nisland);
  for (int i=0; i < nefc; i++) {
    int island = tree_island[efc_tree[i]];
    d->efc_island[i] = island;
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
  }

  // SHOULD NOT OCCUR
  if (!mju_compare(island_nefc2, d->island_nefc, nisland)) mjERROR("island_nefc miscount");

  // dense: block-diagonalize Jacobian
  if (!mj_isSparse(m)) {
    mju_blockDiag(d->iefc_J, d->efc_J,
                  nv, nidof, nisland,
                  d->map_iefc2efc, d->map_idof2dof,
                  d->island_nefc, d->island_nv,
                  d->island_iefcadr, d->island_idofadr);
  }

  // sparse
  else {
    // block-diagonalize Jacobian
    mju_blockDiagSparse(d->iefc_J, d->iefc_J_rownnz, d->iefc_J_rowadr, d->iefc_J_colind,
                        d->efc_J, d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind,
                        nefc, nisland,
                        d->map_iefc2efc, d->map_dof2idof,
                        d->island_iefcadr, d->island_idofadr, NULL, NULL);

    // recompute rowsuper per island
    for (int island=0; island < nisland; island++) {
      int adr = d->island_iefcadr[island];
      mju_superSparse(d->island_nefc[island], d->iefc_J_rowsuper + adr,
                      d->iefc_J_rownnz + adr, d->iefc_J_rowadr + adr, d->iefc_J_colind);
    }
  }

  // copy position-dependent efc vectors required by solver
  mju_gatherInt(d->iefc_type, d->efc_type, d->map_iefc2efc, nefc);
  mju_gatherInt(d->iefc_id, d->efc_id, d->map_iefc2efc, nefc);
  mju_gather(d->iefc_frictionloss, d->efc_frictionloss, d->map_iefc2efc, nefc);
  mju_gather(d->iefc_D, d->efc_D, d->map_iefc2efc, nefc);
  mju_gather(d->iefc_R, d->efc_R, d->map_iefc2efc, nefc);

  mj_freeStack(d);
}
