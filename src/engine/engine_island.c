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
#include "engine/engine_core_constraint.h"
#include "engine/engine_io.h"
#include "engine/engine_support.h"
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
  for (int i=0; i < nr; i++) island[i] = -1;

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



// return upper bound on number of tree-tree edges
static int countMaxEdge(const mjModel* m, const mjData* d) {
  int nedge_max = 0;
  nedge_max += 2*d->ncon;  // contact: 2 edges
  nedge_max += 2*d->ne;    // equality: 2 edges
  nedge_max += d->nl;      // limit: 1 edges (always within same tree)
  nedge_max += d->nf;      // joint friction: 1 edge (always within same tree)

  // tendon limits and friction add up to tendon_num edges
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_frictionloss[i]) {
      nedge_max += m->tendon_num[i];
    }
    if (m->tendon_limited[i]) {
      nedge_max += m->tendon_num[i];
    }
  }

  return nedge_max;
}



// return id of next tree in Jacobian row i that is different from tree, -1 if not found
//   start search from *index
//   write the index of the found tree to *index
//   if J is (dense/sparse) *index is the (column/nonzero) index, respectively
static int treeNext(const mjModel* m, const mjData* d, int tree, int i, int *index) {
  int tree_next = -1;
  int j;  // local loop variable, saved to *index

  // sparse
  if (mj_isSparse(m)) {
    int rownnz = d->efc_J_rownnz[i];
    int* colind = d->efc_J_colind + d->efc_J_rowadr[i];

    // loop over remaining nonzeros, look for different tree
    for (j=(*index); j < rownnz; j++) {
      int tree_j = m->dof_treeid[colind[j]];
      if (tree_j != tree) {
        // found different tree
        tree_next = tree_j;
        break;
      }
    }
  }

  // dense
  else {
    int nv = m->nv;

    // scan row, look for different tree
    for (j=(*index); j < nv; j++) {
      if (d->efc_J[nv*i + j]) {
        int tree_j = m->dof_treeid[j];
        if (tree_j != tree) {
          // found different tree
          tree_next = tree_j;
          break;
        }
      }
    }
  }

  // save last index
  *index = j;

  return tree_next;
}



// find first and possibly second nonegative tree ids in Jacobian row i
//   if row i is special-cased (no more trees), return -1
//   otherwise call treeNext, starting scan at index 0, return index
static int treeFirst(const mjModel* m, const mjData* d, int tree[2], int i) {
  int efc_type = d->efc_type[i];
  int efc_id = d->efc_id[i];

  // clear outputs
  tree[0] = -1;
  tree[1] = -1;

  // ==== fast handling of special cases

  // joint friction
  if (efc_type == mjCNSTR_FRICTION_DOF) {
    tree[0] = m->dof_treeid[efc_id];
    return -1;
  }

  // joint limit
  if (efc_type == mjCNSTR_LIMIT_JOINT) {
    tree[0] = m->dof_treeid[m->jnt_dofadr[efc_id]];
    return -1;
  }

  // contact
  if (efc_type == mjCNSTR_CONTACT_FRICTIONLESS ||
      efc_type == mjCNSTR_CONTACT_PYRAMIDAL ||
      efc_type == mjCNSTR_CONTACT_ELLIPTIC) {
    tree[0] = m->body_treeid[m->geom_bodyid[d->contact[efc_id].geom[0]]];
    tree[1] = m->body_treeid[m->geom_bodyid[d->contact[efc_id].geom[1]]];

    // handle static bodies
    if (tree[0] < 0) {
      if (tree[1] < 0) {
        mjERROR("contact %d is between two static bodies", efc_id);  // SHOULD NOT OCCUR
      } else {
        int tmp = tree[0];
        tree[0] = tree[1];
        tree[1] = tmp;
      }
    }

    return -1;
  }

  // connect or weld constraints
  if (efc_type == mjCNSTR_EQUALITY) {
    mjtEq eq_type = m->eq_type[efc_id];
    if (eq_type == mjEQ_CONNECT || eq_type == mjEQ_WELD) {
      int b1 = m->eq_obj1id[efc_id];
      int b2 = m->eq_obj2id[efc_id];

      // get body ids if using site semantics
      if (m->eq_objtype[efc_id] == mjOBJ_SITE) {
        b1 = m->site_bodyid[b1];
        b2 = m->site_bodyid[b2];
      }

      tree[0] = m->body_treeid[b1];
      tree[1] = m->body_treeid[b2];

      // handle static bodies
      if (tree[0] < 0) {
        if (tree[1] < 0) {
          mjERROR("equality %d is between two static bodies", efc_id);  // SHOULD NOT OCCUR
        } else {
          int tmp = tree[0];
          tree[0] = tree[1];
          tree[1] = tmp;
        }
      }

      return -1;
    }
  }

  // ==== generic case: scan Jacobian
  int index = 0;
  tree[0] = treeNext(m, d, -1, i, &index);

  if (tree[0] < 0) {
    mjERROR("no tree found for constraint %d", i);  // SHOULD NOT OCCUR
  }

  return index;
}



// add 0 edges, 1 self-edge or 2 flipped edges to array, increment treenedge
//   return current number of edges
static int addEdge(int* treenedge, int* edge, int nedge, int tree1, int tree2, int nedge_max) {
  // handle the static tree
  if (tree1 == -1 && tree2 == -1) {
    mjERROR("self-edge of the static tree");  // SHOULD NOT OCCUR
    return 0;
  }
  if (tree1 == -1) tree1 = tree2;
  if (tree2 == -1) tree2 = tree1;

  // previous edge
  int p1 = nedge ? edge[2*nedge - 2] : -1;
  int p2 = nedge ? edge[2*nedge - 1] : -1;

  // === self edge
  if (tree1 == tree2) {
    // same as previous edge, return
    if (nedge && tree1 == p1 && tree1 == p2) {
      return nedge;
    }

    // check size
    if (nedge >= nedge_max) {
      mjERROR("edge array too small");
      return 0;
    }

    // add tree1-tree1 self-edge
    edge[2*nedge + 0] = tree1;
    edge[2*nedge + 1] = tree1;
    treenedge[tree1]++;
    return nedge + 1;
  }

  // === non-self edge
  if (nedge && ((tree1 == p1 && tree2 == p2) || (tree1 == p2 && tree2 == p1))) {
    // same as previous edge, return
    return nedge;
  }

  // check size
  if (nedge + 2 > nedge_max) {
    mjERROR("edge array too small");
    return 0;
  }

  // add tree1-tree2 and tree2-tree1
  edge[2*nedge + 0] = tree1;
  edge[2*nedge + 1] = tree2;
  edge[2*nedge + 2] = tree2;
  edge[2*nedge + 3] = tree1;
  treenedge[tree1]++;
  treenedge[tree2]++;
  return nedge + 2;
}



// find tree-tree edges, increment treenedge counters, return total number of edges
static int findEdges(const mjModel* m, const mjData* d, int* treenedge, int* edge, int nedge_max) {
  int nefc = d->nefc;
  int efc_type = -1;
  int efc_id = -1;

  // clear treenedge
  mju_zeroInt(treenedge, m->ntree);

  int nedge = 0;
  for (int i=0; i < nefc; i++) {
    // row i is still in the same constraint: skip
    if (efc_type == d->efc_type[i] && efc_id == d->efc_id[i]) {
      continue;
    }
    efc_type = d->efc_type[i];
    efc_id = d->efc_id[i];

    int tree[2];
    int index = treeFirst(m, d, tree, i);
    int tree1 = tree[0];
    int tree2 = tree[1];

    // no more edges to find, add and continue
    if (index == -1) {
      nedge = addEdge(treenedge, edge, nedge, tree1, tree2 == -1 ? tree1 : tree2, nedge_max);
      continue;
    }

    // possibly more edges, scan Jacobian row
    else {
      tree2 = treeNext(m, d, tree1, i, &index);

      if (tree2 == -1) {
        // 1 tree found: add self-edge
        nedge = addEdge(treenedge, edge, nedge, tree1, tree1, nedge_max);
      } else {
        // 2 trees found: add edge, keep scanning and adding until no more trees
        nedge = addEdge(treenedge, edge, nedge, tree1, tree2, nedge_max);
        int tree3 = treeNext(m, d, tree2, i, &index);
        while (tree3 > -1 && tree3 != tree2) {
          tree1 = tree2;
          tree2 = tree3;
          nedge = addEdge(treenedge, edge, nedge, tree1, tree2, nedge_max);
          tree3 = treeNext(m, d, tree2, i, &index);
        }
      }
    }
  }

  return nedge;
}



//-------------------------- main entry-point  -----------------------------------------------------

// discover islands:
//   nisland, island_idofadr, dof_island, dof_islandnext, island_efcadr, efc_island, efc_islandnext
void mj_island(const mjModel* m, mjData* d) {
  int nv = m->nv, nefc = d->nefc, ntree=m->ntree;

  // no constraints: quick return
  if (!nefc || m->nflex) {  // TODO: add flex support to island discovery
    d->nisland = 0;
    d->nidof = 0;
    return;
  }

  mj_markStack(d);

  // allocate edge array
  int nedge_max = countMaxEdge(m, d);
  int* edge = mjSTACKALLOC(d, 2*nedge_max, int);

  // get tree-tree edges and rownnz counts from efc arrays
  int* rownnz = mjSTACKALLOC(d, ntree, int);  // number of edges per tree
  int nedge = findEdges(m, d, rownnz, edge, nedge_max);

  // compute starting address of tree's column indices while resetting rownnz
  int* rowadr = mjSTACKALLOC(d, ntree, int);
  rowadr[0] = 0;
  for (int r=1; r < ntree; r++) {
    rowadr[r] = rowadr[r-1] + rownnz[r-1];
    rownnz[r-1] = 0;
  }
  rownnz[ntree-1] = 0;

  // copy column indices: list each tree's neighbors
  int* colind = mjSTACKALLOC(d, nedge, int);
  for (int e=0; e < nedge; e++) {
    int row = edge[2*e];
    int col = edge[2*e + 1];
    colind[rowadr[row] + rownnz[row]++] = col;
  }

  // discover islands
  int* tree_island = mjSTACKALLOC(d, ntree, int);  // id of island assigned to tree
  int* stack = mjSTACKALLOC(d, nedge, int);
  d->nisland = mj_floodFill(tree_island, ntree, rownnz, rowadr, colind, stack);

  // no islands found: quick return
  if (!d->nisland) {
    d->nidof = 0;
    mj_freeStack(d);
    return;
  }

  // count ni: total number of dofs in islands
  int nidof = 0;
  for (int i=0; i < nv; i++) {
    nidof += (tree_island[m->dof_treeid[i]] >= 0);
  }
  d->nidof = nidof;

  // allocate island arrays on arena
  if (!arenaAllocIsland(m, d)) {
    mj_freeStack(d);
    return;
  }

  // local copy
  int nisland = d->nisland;


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
    d->map_idof2dof[idof] = dof;  // only the first ni elements of map_idof2dof are in some island
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
                      d->qLD,  d->M_rownnz, d->M_rowadr, d->M_colind,
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
    int tree[2];
    treeFirst(m, d, tree, i);
    int island = tree_island[tree[0]];
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

    // block-diagonalize Jacobian-transpose
    mju_blockDiagSparse(d->iefc_JT, d->iefc_JT_rownnz, d->iefc_JT_rowadr, d->iefc_JT_colind,
                        d->efc_JT, d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind,
                        nidof, nisland,
                        d->map_idof2dof, d->map_efc2iefc,
                        d->island_idofadr, d->island_iefcadr, NULL, NULL);

    // recompute rowsuper per island
    for (int island=0; island < nisland; island++) {
      int adr = d->island_idofadr[island];
      mju_superSparse(d->island_nv[island], d->iefc_JT_rowsuper + adr,
                      d->iefc_JT_rownnz + adr, d->iefc_JT_rowadr + adr, d->iefc_JT_colind);
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
