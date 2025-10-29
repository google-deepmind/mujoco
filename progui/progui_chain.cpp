#include "progui_globals.h"
#include <cmath>
#include <iostream>

static int g_nextCubeId =0;

static void UpdateProbeForFace()
{
 if (!spec || !m || !d || !g_lastBody || !g_probeRect) return;
 int axisIdx=0, sign=+1; FaceToAxisSign(g_spawnFace, axisIdx, sign);
 const double boxEdge =2.0 * kBoxHalf; const double gap =0.05 * boxEdge;
 g_probeRect->type = mjGEOM_BOX;
 g_probeRect->size[0] = kBoxHalf; g_probeRect->size[1] = kBoxHalf; g_probeRect->size[2] = kBoxHalf;
 double ppos[3] = {0,0,0}; ppos[axisIdx] = sign * (boxEdge + gap);
 g_probeRect->pos[0] = ppos[0]; g_probeRect->pos[1] = ppos[1]; g_probeRect->pos[2] = ppos[2];
 g_probeRect->rgba[0]=0.1f; g_probeRect->rgba[1]=1.0f; g_probeRect->rgba[2]=0.1f; g_probeRect->rgba[3]=0.9f;
 g_probeRect->contype =4; g_probeRect->conaffinity =1; g_probeRect->density =0.0; g_probeRect->margin = kGeomMargin;
 int old_nv=m->nv, old_na=m->na; if (mj_recompile(spec,nullptr,m,d)!=0) return; ApplyBuildModeOptions();
 for (int i=old_nv;i<m->nv;++i) d->qvel[i]=0; for (int i=old_na;i<m->na;++i) d->act[i]=0; mj_forward(m,d); RebuildRenderContext();
}

void UpdateMarkerOnLastBody()
{
 if (!spec || !m || !d || !g_lastBody || !g_lastMarker) return;
 const double boxWidth =2.0 * kBoxHalf; const double sphereR =0.1 * boxWidth; double pos[3]={0,0,0};
 int axisIdx=0, sign=+1; FaceToAxisSign(g_spawnFace, axisIdx, sign);
 pos[axisIdx] = sign * (kBoxHalf -0.5 * sphereR);
 g_lastMarker->pos[0]=pos[0]; g_lastMarker->pos[1]=pos[1]; g_lastMarker->pos[2]=pos[2];
 UpdateProbeForFace();
}

static void DeleteEqualitiesReferencing(const std::string& bodyName)
{
 if (!spec) return;
 std::vector<mjsElement*> toDelete;
 for (mjsElement* el = mjs_firstElement(spec, mjOBJ_EQUALITY); el; el = mjs_nextElement(spec, el)) {
 mjsEquality* eq = mjs_asEquality(el);
 if (!eq) continue; const char* n1 = mjs_getString(eq->name1); const char* n2 = mjs_getString(eq->name2);
 if ((n1 && bodyName == n1) || (n2 && bodyName == n2)) toDelete.push_back(el);
 }
 for (auto* el : toDelete) mjs_delete(spec, el);
}

void spawnCube()
{
 if (!spec || !m || !d) { std::cerr << "spawnCube: model/spec not ready\n"; return; }
 const double halfSize[3] = {kBoxHalf,kBoxHalf,kBoxHalf}; const double boxEdge=2.0*kBoxHalf; const double gap=0.05*boxEdge;
 mjsBody* world = mjs_findBody(spec, "world"); if (!world) { std::cerr << "no world body\n"; return; }
 mjsBody* parent = g_lastBody ? g_lastBody : world;
 bool obstructed=false; std::string targetBodyName; int targetBodyId=-1;
 mjsBody* body = mjs_addBody(parent,nullptr); if (!body) { std::cerr<<"add body failed\n"; return; }
 std::string bodyName = std::string("cube_") + std::to_string(++g_nextCubeId);
 mjs_setName(body->element, bodyName.c_str());
 int spawnAxis=0, spawnSign=+1; FaceToAxisSign(g_spawnFace, spawnAxis, spawnSign);
 double delta=(boxEdge+gap);
 if (g_lastBody && g_probeRect) {
 int hitBodyId = ProbeGetHitBodyId();
 if (hitBodyId>=0) { const char* hitName = mj_id2name(m, mjOBJ_BODY, hitBodyId); delta=(2.0*boxEdge+gap); obstructed=true; targetBodyId=hitBodyId; if (hitName) targetBodyName=hitName; }
 }
 if (!g_lastBody) { body->pos[0]=0; body->pos[1]=0; body->pos[2]=1.0; }
 else { double off = spawnSign*delta; body->pos[0]=(spawnAxis==0)?off:0.0; body->pos[1]=(spawnAxis==1)?off:0.0; body->pos[2]=(spawnAxis==2)?off:0.0; }
 body->quat[0]=1; body->quat[1]=0; body->quat[2]=0; body->quat[3]=0;
 if (!g_lastBody) {
 if (mjsJoint* fj=mjs_addJoint(body,nullptr)) { fj->type=mjJNT_FREE; fj->damping=kHingeDamping; std::string jname=bodyName+"_free"; mjs_setName(fj->element,jname.c_str()); }
 }
 else {
 if (!obstructed) {
 int faceAxis=0, faceSign=+1; FaceToAxisSign(g_spawnFace, faceAxis, faceSign);
 const double boxEdge2=2.0*kBoxHalf; const double gap2=0.05*boxEdge2; double anchor[3]={0,0,0};
 anchor[faceAxis] = -faceSign*(kBoxHalf +0.5*gap2);
 if (g_jointMode==1) {
 if (mjsJoint* bj=mjs_addJoint(body,nullptr)) { bj->type=mjJNT_BALL; bj->pos[0]=anchor[0]; bj->pos[1]=anchor[1]; bj->pos[2]=anchor[2]; bj->damping=kHingeDamping; std::string jn=bodyName+"_ball"; mjs_setName(bj->element, jn.c_str()); } else { std::cerr<<"add ball failed\n"; return; }
 } else {
 if (mjsJoint* hjY=mjs_addJoint(body,nullptr)) {
 hjY->type=mjJNT_HINGE; hjY->axis[0]=0; hjY->axis[1]=1; hjY->axis[2]=0; hjY->pos[0]=anchor[0]; hjY->pos[1]=anchor[1]; hjY->pos[2]=anchor[2]; hjY->damping=kHingeDamping;
 const double r_perp_y = std::sqrt(kBoxHalf*kBoxHalf + kBoxHalf*kBoxHalf);
 double theta_max_y = std::atan2(std::max(1e-6,0.5*gap2), std::max(1e-6,r_perp_y));
 const double th_min = mjPI*5.0/180.0; const double th_max = mjPI*80.0/180.0; theta_max_y = std::min(std::max(theta_max_y, th_min), th_max);
 hjY->limited=mjLIMITED_TRUE; hjY->range[0]=-theta_max_y; hjY->range[1]=theta_max_y; std::string jn=bodyName+"_hinge_y"; mjs_setName(hjY->element, jn.c_str());
 } else { std::cerr<<"add hinge y failed\n"; return; }
 if (mjsJoint* hjX=mjs_addJoint(body,nullptr)) {
 hjX->type=mjJNT_HINGE; hjX->axis[0]=1; hjX->axis[1]=0; hjX->axis[2]=0; hjX->pos[0]=anchor[0]; hjX->pos[1]=anchor[1]; hjX->pos[2]=anchor[2]; hjX->damping=kHingeDamping;
 const double r_perp_x = std::sqrt(kBoxHalf*kBoxHalf + kBoxHalf*kBoxHalf);
 double theta_max_x = std::atan2(std::max(1e-6,0.5*gap2), std::max(1e-6,r_perp_x));
 const double th_min = mjPI*5.0/180.0; const double th_max = mjPI*80.0/180.0; theta_max_x = std::min(std::max(theta_max_x, th_min), th_max);
 hjX->limited=mjLIMITED_TRUE; hjX->range[0]=-theta_max_x; hjX->range[1]=theta_max_x; std::string jn=bodyName+"_hinge_x"; mjs_setName(hjX->element, jn.c_str());
 } else { std::cerr<<"add hinge x failed\n"; return; }
 }
 } else {
 // obstructed: connect via equalities (added after geom creation)
 }
 }
 mjsGeom* geom = mjs_addGeom(body,nullptr); if (!geom) { std::cerr<<"add geom failed\n"; return; }
 geom->type=mjGEOM_BOX; geom->size[0]=halfSize[0]; geom->size[1]=halfSize[1]; geom->size[2]=halfSize[2]; geom->density=1000.0;
 geom->rgba[0]=1; geom->rgba[1]=1; geom->rgba[2]=1; geom->rgba[3]=1;
 if (g_boxesCollide) { geom->contype=1; geom->conaffinity=1|2|4; geom->friction[0]=1.0; geom->friction[1]=0.005; geom->friction[2]=0.0001; }
 else { geom->contype=1; geom->conaffinity=2|4; }
 geom->margin=kGeomMargin; geom->solref[0]=kSolref[0]; geom->solref[1]=kSolref[1]; for (int i=0;i<5;++i) geom->solimp[i]=kSolimp[i];
 if (mjsGeom* marker=mjs_addGeom(body,nullptr)) {
 marker->type=mjGEOM_SPHERE; const double boxWidth=2.0*kBoxHalf; const double sphereRadius=0.1*boxWidth; marker->size[0]=sphereRadius;
 marker->pos[0]=0; marker->pos[1]=0; marker->pos[2]=0; int axisIdx=0, sign=+1; FaceToAxisSign(g_spawnFace, axisIdx, sign);
 marker->pos[axisIdx]= sign*(kBoxHalf-0.5*sphereRadius);
 marker->rgba[0]=1; marker->rgba[1]=0.1f; marker->rgba[2]=0.1f; marker->rgba[3]=1;
 marker->contype=0; marker->conaffinity=0; marker->density=0.0; std::string sname=bodyName+"_spawn_marker"; mjs_setName(marker->element, sname.c_str()); g_lastMarker=marker;
 }
 if (g_probeRect) { g_probeRect->contype=0; g_probeRect->conaffinity=0; g_probeRect->rgba[3]=0.0f; }
 if (mjsGeom* probe=mjs_addGeom(body,nullptr)) {
 int axisIdx=0, sign=+1; FaceToAxisSign(g_spawnFace, axisIdx, sign); const double boxEdge=2.0*kBoxHalf; const double gap=0.05*boxEdge;
 probe->type=mjGEOM_BOX; probe->size[0]=kBoxHalf; probe->size[1]=kBoxHalf; probe->size[2]=kBoxHalf;
 probe->pos[0]=0; probe->pos[1]=0; probe->pos[2]=0; probe->pos[axisIdx]= sign*(boxEdge+gap);
 probe->rgba[0]=0.1f; probe->rgba[1]=1.0f; probe->rgba[2]=0.1f; probe->rgba[3]=0.9f; probe->contype=4; probe->conaffinity=1; probe->density=0.0; probe->margin=kGeomMargin;
 g_probeRect=probe; g_probeName=bodyName+"_spawn_probe"; mjs_setName(probe->element, g_probeName.c_str());
 }
 if (obstructed && g_lastBody && !targetBodyName.empty()) {
 int axisIdx=spawnAxis; int sign=spawnSign; double prev_anchor[3]={0,0,0}; double new_anchor[3]={0,0,0}; double tgt_anchor_to_prev[3]={0,0,0}; double tgt_anchor_to_new[3]={0,0,0};
 prev_anchor[axisIdx]= sign*kBoxHalf; new_anchor[axisIdx] = -sign*kBoxHalf; tgt_anchor_to_prev[axisIdx]= -sign*kBoxHalf; tgt_anchor_to_new[axisIdx]= sign*kBoxHalf;
 if (!g_chain.empty()) {
 const std::string& prevName = g_chain.back().name; if (mjsEquality* eq1=mjs_addEquality(spec,nullptr)) {
 eq1->type=mjEQ_CONNECT; eq1->objtype=mjOBJ_BODY; eq1->active=1; eq1->solref[0]=kSolref[0]; eq1->solref[1]=kSolref[1]; for (int i=0;i<5;++i) eq1->solimp[i]=kSolimp[i];
 mjs_setString(eq1->name1, prevName.c_str()); mjs_setString(eq1->name2, targetBodyName.c_str());
 eq1->data[0]=prev_anchor[0]; eq1->data[1]=prev_anchor[1]; eq1->data[2]=prev_anchor[2]; eq1->data[3]=tgt_anchor_to_prev[0]; eq1->data[4]=tgt_anchor_to_prev[1]; eq1->data[5]=tgt_anchor_to_prev[2];
 std::string ename1 = prevName + "__to__" + targetBodyName + "_connect"; mjs_setName(eq1->element, ename1.c_str()); }
 }
 if (mjsEquality* eq2=mjs_addEquality(spec,nullptr)) { eq2->type=mjEQ_CONNECT; eq2->objtype=mjOBJ_BODY; eq2->active=1; eq2->solref[0]=kSolref[0]; eq2->solref[1]=kSolref[1]; for (int i=0;i<5;++i) eq2->solimp[i]=kSolimp[i];
 mjs_setString(eq2->name1, bodyName.c_str()); mjs_setString(eq2->name2, targetBodyName.c_str());
 eq2->data[0]=new_anchor[0]; eq2->data[1]=new_anchor[1]; eq2->data[2]=new_anchor[2]; eq2->data[3]=tgt_anchor_to_new[0]; eq2->data[4]=tgt_anchor_to_new[1]; eq2->data[5]=tgt_anchor_to_new[2];
 std::string ename2 = bodyName + "__to__" + targetBodyName + "_connect"; mjs_setName(eq2->element, ename2.c_str()); }
 }
 g_lastBody=body; g_chain.push_back(ChainEntry{body, bodyName, -1});
 int old_nv=m->nv, old_na=m->na; if (mj_recompile(spec,nullptr,m,d)!=0) { std::cerr<<"recompile failed: "<< mjs_getError(spec) <<"\n"; return; }
 ApplyBuildModeOptions(); for (int i=old_nv;i<m->nv;++i) d->qvel[i]=0; for (int i=old_na;i<m->na;++i) d->act[i]=0; mj_forward(m,d); UpdateBodyIdIndexMap(); RebuildRenderContext();
 std::cout << "Cube spawned: " << bodyName << "\n";
}

void deleteLastCube()
{
 if (!spec || !m || !d) return; if (g_chain.size()<=1) { std::cerr<<"deleteLastCube: root cannot be deleted\n"; return; }
 ChainEntry last = g_chain.back(); ChainEntry prev = g_chain[g_chain.size()-2];
 DeleteEqualitiesReferencing(last.name);
 if (last.specBody) mjs_delete(spec, last.specBody->element);
 g_chain.pop_back(); g_lastBody = prev.specBody; g_lastMarker=nullptr; g_probeRect=nullptr;
 std::string prevMarkerName=prev.name+"_spawn_marker"; std::string prevProbeName=prev.name+"_spawn_probe";
 if (mjsElement* mel=mjs_findElement(spec, mjOBJ_GEOM, prevMarkerName.c_str())) g_lastMarker=mjs_asGeom(mel);
 if (mjsElement* pel=mjs_findElement(spec, mjOBJ_GEOM, prevProbeName.c_str())) { g_probeRect=mjs_asGeom(pel); g_probeName=prevProbeName; if (g_probeRect) { g_probeRect->contype=4; g_probeRect->conaffinity=1; g_probeRect->rgba[0]=0.1f; g_probeRect->rgba[1]=1.0f; g_probeRect->rgba[2]=0.1f; g_probeRect->rgba[3]=0.9f; g_probeRect->density=0.0; g_probeRect->margin=kGeomMargin; } }
 int old_nv=m->nv, old_na=m->na; if (mj_recompile(spec,nullptr,m,d)!=0) { std::cerr<<"deleteLastCube: recompile failed: "<<mjs_getError(spec)<<"\n"; return; }
 ApplyBuildModeOptions(); for (int i=old_nv;i<m->nv;++i) d->qvel[i]=0; for (int i=old_na;i<m->na;++i) d->act[i]=0; mj_forward(m,d); UpdateBodyIdIndexMap(); RebuildRenderContext(); UpdateMarkerOnLastBody();
}

void moveLastCubeY(int dir)
{
 if (!spec || !m || !d) { std::cerr<<"moveLastCubeY: not ready\n"; return; } if (g_chain.size()<2) { std::cerr<<"need >=2 cubes\n"; return; }
 const double step=2.0*kBoxHalf; const double dy=(dir>0?step:-step); const size_t N=g_chain.size(); const double invN=1.0/static_cast<double>(N);
 for (size_t i=0;i<N;++i) { mjsBody* b=g_chain[i].specBody; if (!b) continue; double f=static_cast<double>(i+1)*invN; b->pos[1]+=dy*f; }
 int old_nv=m->nv, old_na=m->na; if (mj_recompile(spec,nullptr,m,d)!=0) { std::cerr<<"moveLastCubeY: recompile failed: "<<mjs_getError(spec)<<"\n"; return; }
 for (int i=old_nv;i<m->nv;++i) d->qvel[i]=0; for (int i=old_na;i<m->na;++i) d->act[i]=0; ApplyBuildModeOptions(); mj_forward(m,d); UpdateBodyIdIndexMap(); RebuildRenderContext();
}

void moveLastCubeX(int dir)
{
 if (!spec || !m || !d) { std::cerr<<"moveLastCubeX: not ready\n"; return; } if (g_chain.size()<2) { std::cerr<<"need >=2 cubes\n"; return; }
 const double step=2.0*kBoxHalf; const double dx=(dir>0?step:-step); const size_t N=g_chain.size(); const double invN=1.0/static_cast<double>(N);
 for (size_t i=0;i<N;++i) { mjsBody* b=g_chain[i].specBody; if (!b) continue; double f=static_cast<double>(i+1)*invN; b->pos[0]+=dx*f; }
 int old_nv=m->nv, old_na=m->na; if (mj_recompile(spec,nullptr,m,d)!=0) { std::cerr<<"moveLastCubeX: recompile failed: "<<mjs_getError(spec)<<"\n"; return; }
 ApplyBuildModeOptions(); for (int i=old_nv;i<m->nv;++i) d->qvel[i]=0; for (int i=old_na;i<m->na;++i) d->act[i]=0; mj_forward(m,d); UpdateBodyIdIndexMap(); RebuildRenderContext();
}

void moveLastCubeZ(int dir)
{
 if (!spec || !m || !d) { std::cerr<<"moveLastCubeZ: not ready\n"; return; } if (g_chain.size()<2) { std::cerr<<"need >=2 cubes\n"; return; }
 const double step=2.0*kBoxHalf; const double dz=(dir>0?step:-step); const size_t N=g_chain.size(); const double invN=1.0/static_cast<double>(N);
 for (size_t i=0;i<N;++i) { mjsBody* b=g_chain[i].specBody; if (!b) continue; double f=static_cast<double>(i+1)*invN; b->pos[2]+=dz*f; }
 int old_nv=m->nv, old_na=m->na; if (mj_recompile(spec,nullptr,m,d)!=0) { std::cerr<<"moveLastCubeZ: recompile failed: "<<mjs_getError(spec)<<"\n"; return; }
 ApplyBuildModeOptions(); for (int i=old_nv;i<m->nv;++i) d->qvel[i]=0; for (int i=old_na;i<m->na;++i) d->act[i]=0; mj_forward(m,d); UpdateBodyIdIndexMap(); RebuildRenderContext();
}

void EnablePhysicsForAll()
{
 if (!spec || !m || !d) { std::cerr<<"EnablePhysicsForAll: not ready\n"; return; }
 if (g_chain.empty()) { std::cerr<<"EnablePhysicsForAll: no cubes\n"; return; }
 if (g_physicsEnabled) return;
 m->opt.gravity[0]=g_savedGravity[0]; m->opt.gravity[1]=g_savedGravity[1]; m->opt.gravity[2]=g_savedGravity[2];
 m->opt.disableflags &= ~mjDSBL_GRAVITY; mj_forward(m,d); RebuildRenderContext(); g_physicsEnabled=true;
}
