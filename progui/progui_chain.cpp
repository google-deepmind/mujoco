#include "progui_globals.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// Globals / state
// -----------------------------------------------------------------------------

static int g_nextCubeId =0;

// key name used to persist the gap value in the spec
static constexpr const char* kPersistGapKey = "__progui_gap__";

// -----------------------------------------------------------------------------
// Utilities
// -----------------------------------------------------------------------------

// parse cube name of form "cube_<number>", return id or -1 if not a cube name
static int ParseCubeId(const char* name) {
 if (!name) return -1;
 const char prefix[] = "cube_";
 constexpr size_t kPrefixLen = sizeof(prefix) -1;
 if (std::strncmp(name, prefix, kPrefixLen) !=0) return -1;

 const char* num = name + kPrefixLen;
 if (!*num) return -1;

 char* endp = nullptr;
 long val = std::strtol(num, &endp,10);
 if (endp == num || val <0) return -1;
 return static_cast<int>(val);
}

// scan current spec and set g_nextCubeId to the maximum cube id present
static void RefreshNextCubeIdFromSpec() {
 if (!spec) return;

 int maxId =0;
 for (mjsElement* el = mjs_firstElement(spec, mjOBJ_BODY);
 el; el = mjs_nextElement(spec, el)) {
 const char* nm = mjs_getString(mjs_getName(el));
 int id = ParseCubeId(nm);
 if (id > maxId) maxId = id;
 }
 g_nextCubeId = maxId;
}

// persist current gap to spec (creates or updates a numeric named kPersistGapKey)
static void SaveGapToSpec() {
 if (!spec) return;

 mjsElement* el = mjs_findElement(spec, mjOBJ_NUMERIC, kPersistGapKey);
 mjsNumeric* num = el ? mjs_asNumeric(el) : mjs_addNumeric(spec);
 if (!num) return;

 mjs_setName(num->element, kPersistGapKey);
 num->size =1;

 double val = g_gapRatio;
 mjs_setDouble(num->data, &val,1);
}

// load gap from spec if present
static void LoadGapFromSpec(mjSpec* s) {
 if (!s) return;

 mjsElement* el = mjs_findElement(s, mjOBJ_NUMERIC, kPersistGapKey);
 if (!el) return;

 mjsNumeric* num = mjs_asNumeric(el);
 if (!num) return;

 int sz =0;
 const double* data = mjs_getDouble(num->data, &sz);
 if (data && sz >0) {
 g_gapRatio = data[0];
 }
}

// helper to get current gap value from ratio
static inline double CurrentGap() {
 const double boxEdge =2.0 * kBoxHalf;
 return g_gapRatio * boxEdge;
}

// -----------------------------------------------------------------------------
// Loop utilities
// -----------------------------------------------------------------------------

// Detect if the ghost probe hits an existing body. On hit, fill outTargetBodyName
// and return true.
static bool LoopContactCheck(std::string& outTargetBodyName) {
 outTargetBodyName.clear();
 if (!g_lastBody || !g_probeRect) return false;

 // ensure probe is active and collisions are up-to-date
 if (g_probeRect->contype ==0 && g_probeRect->conaffinity ==0) {
 g_probeRect->contype =4;
 g_probeRect->conaffinity =1;
 }

 mj_kinematics(m, d);
 mj_collision(m, d);

 int hitBodyId = ProbeGetHitBodyId();
 if (hitBodyId <0) return false;

 if (const char* hitName = mj_id2name(m, mjOBJ_BODY, hitBodyId)) {
 outTargetBodyName = hitName;
 std::cout << "LoopContactCheck: hit body " << hitBodyId
 << " name=" << outTargetBodyName << "\n";
 }

 return true;
}

// Create the equality constraints to close a loop between the previous body/new
// body and the target body.
static void LoopCreate(int spawnAxis, int spawnSign,
 const std::string& newBodyName,
 const std::string& targetBodyName) {
 if (!spec || !g_lastBody || targetBodyName.empty()) return;

 auto equalityExists = [&](const std::string& name) -> bool {
 return mjs_findElement(spec, mjOBJ_EQUALITY, name.c_str()) != nullptr;
 };

 int axisIdx = spawnAxis;
 int sign = spawnSign;

 double prev_anchor[3] = {0,0,0};
 double new_anchor[3] = {0,0,0};
 double tgt_anchor_to_prev[3] = {0,0,0};
 double tgt_anchor_to_new[3] = {0,0,0};

 prev_anchor[axisIdx] = sign * kBoxHalf;
 new_anchor[axisIdx] = -sign * kBoxHalf;
 tgt_anchor_to_prev[axisIdx] = -sign * kBoxHalf;
 tgt_anchor_to_new[axisIdx] = sign * kBoxHalf;

 // Construct names up-front to avoid duplicates across iterations
 std::string prevName = g_chain.empty() ? std::string() : g_chain.back().name;
 std::string ename1 = prevName + "__to__" + targetBodyName + "_connect";
 std::string ename2 = newBodyName + "__to__" + targetBodyName + "_connect";

 // Create prev-to-target equality if not already present
 if (!prevName.empty() && !equalityExists(ename1)) {
 if (mjsEquality* eq1 = mjs_addEquality(spec, nullptr)) {
 eq1->type = mjEQ_CONNECT;
 eq1->objtype = mjOBJ_BODY;
 eq1->active =1;
 eq1->solref[0] = kSolref[0];
 eq1->solref[1] = kSolref[1];
 for (int i =0; i <5; ++i) eq1->solimp[i] = kSolimp[i];

 mjs_setString(eq1->name1, prevName.c_str());
 mjs_setString(eq1->name2, targetBodyName.c_str());

 eq1->data[0] = prev_anchor[0];
 eq1->data[1] = prev_anchor[1];
 eq1->data[2] = prev_anchor[2];
 eq1->data[3] = tgt_anchor_to_prev[0];
 eq1->data[4] = tgt_anchor_to_prev[1];
 eq1->data[5] = tgt_anchor_to_prev[2];

 mjs_setName(eq1->element, ename1.c_str());
 }
 }

 // Create new-to-target equality if not already present
 if (!equalityExists(ename2)) {
 if (mjsEquality* eq2 = mjs_addEquality(spec, nullptr)) {
 eq2->type = mjEQ_CONNECT;
 eq2->objtype = mjOBJ_BODY;
 eq2->active =1;
 eq2->solref[0] = kSolref[0];
 eq2->solref[1] = kSolref[1];
 for (int i =0; i <5; ++i) eq2->solimp[i] = kSolimp[i];

 mjs_setString(eq2->name1, newBodyName.c_str());
 mjs_setString(eq2->name2, targetBodyName.c_str());

 eq2->data[0] = new_anchor[0];
 eq2->data[1] = new_anchor[1];
 eq2->data[2] = new_anchor[2];
 eq2->data[3] = tgt_anchor_to_new[0];
 eq2->data[4] = tgt_anchor_to_new[1];
 eq2->data[5] = tgt_anchor_to_new[2];

 mjs_setName(eq2->element, ename2.c_str());
 }
 }
}

// -----------------------------------------------------------------------------
// Save/Load helpers
// -----------------------------------------------------------------------------

// Save the current chain bodies' local positions (relative to their parents)
void SaveChainPrePhysicsState() {
 g_savedPrePhysicsChain.clear();

 for (const auto& e : g_chain) {
 if (!e.specBody) continue;

 SavedBodyPos s{};
 s.name = e.name;
 s.pos[0] = e.specBody->pos[0];
 s.pos[1] = e.specBody->pos[1];
 s.pos[2] = e.specBody->pos[2];
 s.quat[0] = e.specBody->quat[0];
 s.quat[1] = e.specBody->quat[1];
 s.quat[2] = e.specBody->quat[2];
 s.quat[3] = e.specBody->quat[3];
 g_savedPrePhysicsChain.push_back(s);
 }

 g_hasSavedPrePhysicsState = !g_savedPrePhysicsChain.empty();
 std::cout << "Saved pre-physics state for "
 << g_savedPrePhysicsChain.size() << " bodies\n";
}

static void UpdateProbeForFace() {
 if (!spec || !m || !d || !g_lastBody || !g_probeRect) return;

 int axisIdx =0, sign = +1;
 FaceToAxisSign(g_spawnFace, axisIdx, sign);

 const double boxEdge =2.0 * kBoxHalf;
 const double gap = CurrentGap();

 g_probeRect->type = mjGEOM_BOX;
 g_probeRect->size[0] = kBoxHalf;
 g_probeRect->size[1] = kBoxHalf;
 g_probeRect->size[2] = kBoxHalf;

 double ppos[3] = {0,0,0};
 ppos[axisIdx] = sign * (boxEdge + gap);

 g_probeRect->pos[0] = ppos[0];
 g_probeRect->pos[1] = ppos[1];
 g_probeRect->pos[2] = ppos[2];

 g_probeRect->rgba[0] =0.1f;
 g_probeRect->rgba[1] =1.0f;
 g_probeRect->rgba[2] =0.1f;
 g_probeRect->rgba[3] =0.9f;

 g_probeRect->contype =4;
 g_probeRect->conaffinity =1;
 g_probeRect->density =0.0;
 g_probeRect->margin = kGeomMargin;

 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) return;

 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;
 mj_forward(m, d);
 RebuildRenderContext();
}

void UpdateMarkerOnLastBody() {
 if (!spec || !m || !d || !g_lastBody || !g_lastMarker) return;

 const double boxWidth =2.0 * kBoxHalf;
 const double sphereR =0.1 * boxWidth;

 double pos[3] = {0,0,0};
 int axisIdx =0, sign = +1;
 FaceToAxisSign(g_spawnFace, axisIdx, sign);
 pos[axisIdx] = sign * (kBoxHalf -0.5 * sphereR);

 g_lastMarker->pos[0] = pos[0];
 g_lastMarker->pos[1] = pos[1];
 g_lastMarker->pos[2] = pos[2];

 UpdateProbeForFace();
}

static void DeleteEqualitiesReferencing(const std::string& bodyName) {
 if (!spec) return;

 std::vector<mjsElement*> toDelete;
 for (mjsElement* el = mjs_firstElement(spec, mjOBJ_EQUALITY);
 el; el = mjs_nextElement(spec, el)) {
 mjsEquality* eq = mjs_asEquality(el);
 if (!eq) continue;

 const char* n1 = mjs_getString(eq->name1);
 const char* n2 = mjs_getString(eq->name2);
 if ((n1 && bodyName == n1) || (n2 && bodyName == n2)) {
 toDelete.push_back(el);
 }
 }

 for (auto* el : toDelete) {
 mjs_delete(spec, el);
 }
}

// -----------------------------------------------------------------------------
// Chain reset / spawn / delete
// -----------------------------------------------------------------------------

// Reset the chain back to the saved positions (and disable physics)
void ResetChainToSavedState() {
 if (!spec || !m || !d) return;

 if (!g_hasSavedPrePhysicsState || g_savedPrePhysicsChain.empty()) {
 std::cerr << "No saved pre-physics state to reset to\n";
 return;
 }

 g_physicsEnabled = false;
 m->opt.gravity[0] =0.0;
 m->opt.gravity[1] =0.0;
 m->opt.gravity[2] =0.0;
 m->opt.disableflags |= mjDSBL_GRAVITY;

 const size_t targetN = g_savedPrePhysicsChain.size();
 while (g_chain.size() > targetN) {
 deleteLastCube();
 }
 while (g_chain.size() < targetN) {
 spawnCube();
 }

 const size_t N = std::min(g_chain.size(), targetN);
 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;

 b->pos[0] = g_savedPrePhysicsChain[i].pos[0];
 b->pos[1] = g_savedPrePhysicsChain[i].pos[1];
 b->pos[2] = g_savedPrePhysicsChain[i].pos[2];
 b->quat[0] = g_savedPrePhysicsChain[i].quat[0];
 b->quat[1] = g_savedPrePhysicsChain[i].quat[1];
 b->quat[2] = g_savedPrePhysicsChain[i].quat[2];
 b->quat[3] = g_savedPrePhysicsChain[i].quat[3];
 }

 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "ResetChainToSavedState: recompile failed: "
 << mjs_getError(spec) << "\n";
 return;
 }

 mj_resetData(m, d);
 ApplyBuildModeOptions();
 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
 UpdateMarkerOnLastBody();

 std::cout << "Chain reset to saved pre-physics state\n";
}

void spawnCube() {
 if (!spec || !m || !d) {
 std::cerr << "spawnCube: model/spec not ready\n";
 return;
 }

 const double halfSize[3] = {kBoxHalf, kBoxHalf, kBoxHalf};
 const double boxEdge =2.0 * kBoxHalf;
 const double gap = CurrentGap();

 mjsBody* world = mjs_findBody(spec, "world");
 if (!world) {
 std::cerr << "no world body\n";
 return;
 }

 mjsBody* parent = g_lastBody ? g_lastBody : world;

 bool obstructed = false;
 std::string targetBodyName;
 int targetBodyId = -1;

 mjsBody* body = mjs_addBody(parent, nullptr);
 if (!body) {
 std::cerr << "add body failed\n";
 return;
 }

 std::string bodyName = std::string("cube_") + std::to_string(++g_nextCubeId);
 mjs_setName(body->element, bodyName.c_str());

 int spawnAxis =0, spawnSign = +1;
 FaceToAxisSign(g_spawnFace, spawnAxis, spawnSign);

 // compute placement delta and detect if we are creating a loop
 double delta = (boxEdge + gap);
 std::string outTarget;
 obstructed = LoopContactCheck(outTarget);
 if (obstructed) {
 // set target and place new body farther so there is gap clearance when forming the loop
 targetBodyName = outTarget;
 delta = (2.0 * delta);
 }

 if (!g_lastBody) {
 body->pos[0] =0;
 body->pos[1] =0;
 body->pos[2] =1.0;
 } else {
 double off = spawnSign * delta;
 body->pos[0] = (spawnAxis ==0) ? off :0.0;
 body->pos[1] = (spawnAxis ==1) ? off :0.0;
 body->pos[2] = (spawnAxis ==2) ? off :0.0;
 }

 body->quat[0] =1;
 body->quat[1] =0;
 body->quat[2] =0;
 body->quat[3] =0;

 // Note: mjsBody does not expose per-body damping; damping is set per-joint
 // (see kHingeDamping)
 if (!g_lastBody) {
 if (mjsJoint* fj = mjs_addJoint(body, nullptr)) {
 fj->type = mjJNT_FREE;
 fj->damping = kHingeDamping;
 std::string jname = bodyName + "_free";
 mjs_setName(fj->element, jname.c_str());
 }
 } else {
 if (!obstructed) {
 int faceAxis =0, faceSign = +1;
 FaceToAxisSign(g_spawnFace, faceAxis, faceSign);

 const double boxEdge2 =2.0 * kBoxHalf;
 const double gap2 = CurrentGap();

 double anchor[3] = {0,0,0};
 anchor[faceAxis] = -faceSign * (kBoxHalf +0.5 * gap2);

 if (g_jointMode ==1) {
 if (mjsJoint* bj = mjs_addJoint(body, nullptr)) {
 bj->type = mjJNT_BALL;
 bj->pos[0] = anchor[0];
 bj->pos[1] = anchor[1];
 bj->pos[2] = anchor[2];
 bj->damping = kHingeDamping; // softer
 std::string jn = bodyName + "_ball";
 mjs_setName(bj->element, jn.c_str());

 // Compute ball joint cone from current gap
 const double a = kBoxHalf;
 const double g = gap2;
 const double c =1.0 + (a >0.0 ? (g / a) :0.0);
 const double root2 = std::sqrt(2.0);

 double phi = (c >= root2)
 ? (mjPI /2.0)
 : (std::asin(std::max(0.0, std::min(1.0, c / root2))) -
 (mjPI /4.0));
 const double th_min = mjPI *5.0 /180.0;
 const double th_max = mjPI *80.0 /180.0;
 phi = std::max(th_min, std::min(th_max, phi));

 bj->limited = mjLIMITED_TRUE;
 bj->range[0] =0.0;
 bj->range[1] = phi;
 } else {
 std::cerr << "add ball failed\n";
 return;
 }
 } else {
 if (mjsJoint* hjY = mjs_addJoint(body, nullptr)) {
 hjY->type = mjJNT_HINGE;
 hjY->axis[0] =0;
 hjY->axis[1] =1;
 hjY->axis[2] =0;
 hjY->pos[0] = anchor[0];
 hjY->pos[1] = anchor[1];
 hjY->pos[2] = anchor[2];
 hjY->damping = kHingeDamping; // softer

 const double r_perp =
 std::sqrt(kBoxHalf * kBoxHalf + kBoxHalf * kBoxHalf);
 double theta = std::atan2(std::max(1e-6,0.5 * gap2),
 std::max(1e-6, r_perp));
 const double th_min = mjPI *5.0 /180.0;
 const double th_max = mjPI *80.0 /180.0;
 theta = std::max(th_min, std::min(th_max, theta));

 hjY->limited = mjLIMITED_TRUE;
 hjY->range[0] = -theta;
 hjY->range[1] = +theta;

 std::string jn = bodyName + "_hinge_y";
 mjs_setName(hjY->element, jn.c_str());
 } else {
 std::cerr << "add hinge y failed\n";
 return;
 }

 if (mjsJoint* hjX = mjs_addJoint(body, nullptr)) {
 hjX->type = mjJNT_HINGE;
 hjX->axis[0] =1;
 hjX->axis[1] =0;
 hjX->axis[2] =0;
 hjX->pos[0] = anchor[0];
 hjX->pos[1] = anchor[1];
 hjX->pos[2] = anchor[2];
 hjX->damping = kHingeDamping; // softer

 const double r_perp =
 std::sqrt(kBoxHalf * kBoxHalf + kBoxHalf * kBoxHalf);
 double theta = std::atan2(std::max(1e-6,0.5 * gap2),
 std::max(1e-6, r_perp));
 const double th_min = mjPI *5.0 /180.0;
 const double th_max = mjPI *80.0 /180.0;
 theta = std::max(th_min, std::min(th_max, theta));

 hjX->limited = mjLIMITED_TRUE;
 hjX->range[0] = -theta;
 hjX->range[1] = +theta;

 std::string jn = bodyName + "_hinge_x";
 mjs_setName(hjX->element, jn.c_str());
 } else {
 std::cerr << "add hinge x failed\n";
 return;
 }
 }
 } else {
 // obstructed path: equality constraints will be created below
 }
 }

 mjsGeom* geom = mjs_addGeom(body, nullptr);
 if (!geom) {
 std::cerr << "add geom failed\n";
 return;
 }

 geom->type = mjGEOM_BOX;
 geom->size[0] = halfSize[0];
 geom->size[1] = halfSize[1];
 geom->size[2] = halfSize[2];
 geom->density =1000.0;
 geom->rgba[0] =1;
 geom->rgba[1] =1;
 geom->rgba[2] =1;
 geom->rgba[3] =1;

 if (g_boxesCollide) {
 geom->contype =1;
 geom->conaffinity =1 |2 |4;
 geom->friction[0] =1.0;
 geom->friction[1] =0.005;
 geom->friction[2] =0.0001;
 } else {
 geom->contype =1;
 geom->conaffinity =2 |4;
 }

 geom->margin = kGeomMargin;
 geom->solref[0] = kSolref[0];
 geom->solref[1] = kSolref[1];
 for (int i =0; i <5; ++i) geom->solimp[i] = kSolimp[i];

 if (mjsGeom* marker = mjs_addGeom(body, nullptr)) {
 marker->type = mjGEOM_SPHERE;

 const double boxWidth =2.0 * kBoxHalf;
 const double sphereRadius =0.1 * boxWidth;

 marker->size[0] = sphereRadius;
 marker->pos[0] =0;
 marker->pos[1] =0;
 marker->pos[2] =0;

 int axisIdx =0, sign = +1;
 FaceToAxisSign(g_spawnFace, axisIdx, sign);
 marker->pos[axisIdx] = sign * (kBoxHalf -0.5 * sphereRadius);

 marker->rgba[0] =1;
 marker->rgba[1] =0.1f;
 marker->rgba[2] =0.1f;
 marker->rgba[3] =1;

 marker->contype =0;
 marker->conaffinity =0;
 marker->density =0.0;

 std::string sname = bodyName + "_spawn_marker";
 mjs_setName(marker->element, sname.c_str());
 g_lastMarker = marker;
 }

 if (g_probeRect) {
 g_probeRect->contype =0;
 g_probeRect->conaffinity =0;
 g_probeRect->rgba[3] =0.0f;
 }

 if (mjsGeom* probe = mjs_addGeom(body, nullptr)) {
 int axisIdx =0, sign = +1;
 FaceToAxisSign(g_spawnFace, axisIdx, sign);

 const double boxEdgeL =2.0 * kBoxHalf;
 const double gapL = CurrentGap();

 probe->type = mjGEOM_BOX;
 probe->size[0] = kBoxHalf;
 probe->size[1] = kBoxHalf;
 probe->size[2] = kBoxHalf;
 probe->pos[0] =0;
 probe->pos[1] =0;
 probe->pos[2] =0;
 probe->pos[axisIdx] = sign * (boxEdgeL + gapL);

 probe->rgba[0] =0.1f;
 probe->rgba[1] =1.0f;
 probe->rgba[2] =0.1f;
 probe->rgba[3] =0.9f;

 probe->contype =4;
 probe->conaffinity =1;
 probe->density =0.0;
 probe->margin = kGeomMargin;

 g_probeRect = probe;
 g_probeName = bodyName + "_spawn_probe";
 mjs_setName(probe->element, g_probeName.c_str());
 }

 if (obstructed && g_lastBody && !targetBodyName.empty()) {
 // create equality constraints to close the loop between previous and target,
 // and new and target
 LoopCreate(spawnAxis, spawnSign, bodyName, targetBodyName);
 }

 g_lastBody = body;
 g_chain.push_back(ChainEntry{body, bodyName, -1, spawnAxis, spawnSign});

 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }

 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;

 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
 std::cout << "Cube spawned: " << bodyName << "\n";
}

void deleteLastCube() {
 if (!spec || !m || !d) return;

 if (g_chain.size() <=1) {
 std::cerr << "deleteLastCube: root cannot be deleted\n";
 return;
 }

 ChainEntry last = g_chain.back();
 ChainEntry prev = g_chain[g_chain.size() -2];

 DeleteEqualitiesReferencing(last.name);

 if (last.specBody) mjs_delete(spec, last.specBody->element);

 g_chain.pop_back();
 g_lastBody = prev.specBody;
 g_lastMarker = nullptr;
 g_probeRect = nullptr;

 std::string prevMarkerName = prev.name + "_spawn_marker";
 std::string prevProbeName = prev.name + "_spawn_probe";

 if (mjsElement* mel = mjs_findElement(spec, mjOBJ_GEOM, prevMarkerName.c_str())) {
 g_lastMarker = mjs_asGeom(mel);
 }

 if (mjsElement* pel = mjs_findElement(spec, mjOBJ_GEOM, prevProbeName.c_str())) {
 g_probeRect = mjs_asGeom(pel);
 g_probeName = prevProbeName;

 if (g_probeRect) {
 g_probeRect->contype =4;
 g_probeRect->conaffinity =1;
 g_probeRect->rgba[0] =0.1f;
 g_probeRect->rgba[1] =1.0f;
 g_probeRect->rgba[2] =0.1f;
 g_probeRect->rgba[3] =0.9f;
 g_probeRect->density =0.0;
 g_probeRect->margin = kGeomMargin;
 }
 }

 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "deleteLastCube: recompile failed: "
 << mjs_getError(spec) << "\n";
 return;
 }

 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;

 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
 UpdateMarkerOnLastBody();
}

void moveLastCubeY(int dir) {
 if (!spec || !m || !d) {
 std::cerr << "moveLastCubeY: not ready\n";
 return;
 }
 if (g_chain.size() <2) {
 std::cerr << "need >=2 cubes\n";
 return;
 }

 const double step =2.0 * kBoxHalf;
 const double dy = (dir >0 ? step : -step);

 const size_t N = g_chain.size();
 const double invN =1.0 / static_cast<double>(N);

 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;

 double f = static_cast<double>(i +1) * invN;
 b->pos[1] += dy * f;
 }

 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "moveLastCubeY: recompile failed: "
 << mjs_getError(spec) << "\n";
 return;
 }

 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;

 ApplyBuildModeOptions();
 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
}

void moveLastCubeX(int dir) {
 if (!spec || !m || !d) {
 std::cerr << "moveLastCubeX: not ready\n";
 return;
 }
 if (g_chain.size() <2) {
 std::cerr << "need >=2 cubes\n";
 return;
 }

 const double step =2.0 * kBoxHalf;
 const double dx = (dir >0 ? step : -step);

 const size_t N = g_chain.size();
 const double invN =1.0 / static_cast<double>(N);

 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;

 double f = static_cast<double>(i +1) * invN;
 b->pos[0] += dx * f;
 }

 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "moveLastCubeX: recompile failed: "
 << mjs_getError(spec) << "\n";
 return;
 }

 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;

 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
}

void moveLastCubeZ(int dir) {
 if (!spec || !m || !d) {
 std::cerr << "moveLastCubeZ: not ready\n";
 return;
 }
 if (g_chain.size() <2) {
 std::cerr << "need >=2 cubes\n";
 return;
 }

 const double step =2.0 * kBoxHalf;
 const double dz = (dir >0 ? step : -step);

 const size_t N = g_chain.size();
 const double invN =1.0 / static_cast<double>(N);

 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;

 double f = static_cast<double>(i +1) * invN;
 b->pos[2] += dz * f;
 }

 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "moveLastCubeZ: recompile failed: "
 << mjs_getError(spec) << "\n";
 return;
 }

 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;

 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
}

void EnablePhysicsForAll() {
 if (!spec || !m || !d) {
 std::cerr << "EnablePhysicsForAll: not ready\n";
 return;
 }
 if (g_chain.empty()) {
 std::cerr << "EnablePhysicsForAll: no cubes\n";
 return;
 }
 if (g_physicsEnabled) return;

 m->opt.gravity[0] = g_savedGravity[0];
 m->opt.gravity[1] = g_savedGravity[1];
 m->opt.gravity[2] = g_savedGravity[2];

 m->opt.disableflags &= ~mjDSBL_GRAVITY;
 mj_forward(m, d);
 RebuildRenderContext();
 g_physicsEnabled = true;
}

// -----------------------------------------------------------------------------
// Save / Load
// -----------------------------------------------------------------------------

bool SaveChainToFile(const char* filename) {
 const char* out = (filename && std::strlen(filename) >0)
 ? filename
 : "chain_save.xml";
 if (!spec && !m) return false;

 // persist current gap into spec before saving
 SaveGapToSpec();

 char error[1024] = {0};
 int ok = -1;

 if (spec) {
	 ok = mj_saveXML(spec, out, error, sizeof(error)); //0 on success
	 if (ok ==0) {
	 std::cout << "Saved chain to XML: " << out << "\n";
	 return true;
 }

 std::cerr << "SaveChainToFile: mj_saveXML failed: " << error << "\n";
 return false;
 } else {
	 // fallback: save from model using global spec inside xml_api
	 int s = mj_saveLastXML(out, m, error, sizeof(error));
	 if (s) {
	 std::cout << "Saved chain to XML: " << out << "\n";
	 return true;
 }

 std::cerr << "SaveChainToFile: mj_saveLastXML failed: " << error << "\n";
 return false;
 }
}

static void ReconstructChainFromSpec() {
 g_chain.clear();
 g_lastBody = nullptr;
 g_lastMarker = nullptr;
 g_probeRect = nullptr;
 g_probeName.clear();

 if (!spec) return;

 // world root
 mjsBody* world = mjs_findBody(spec, "world");
 if (!world) return;

 // find first cube under world
 mjsBody* first = nullptr;
 for (mjsElement* el = mjs_firstChild(world, mjOBJ_BODY, /*recurse=*/0);
 el; el = mjs_nextChild(world, el, /*recurse=*/0)) {
 // el should be a body element; keep a defensive check
 if (el->elemtype == mjOBJ_BODY) {
	 mjsBody* b = mjs_asBody(el);
	 (void)b; // not used now but kept for clarity

	 const char* nm = mjs_getString(mjs_getName(el));
		 if (nm && std::strncmp(nm, "cube_",5) ==0) {
			 first = mjs_asBody(el);
			 break;
		 }
	 }
 }

 if (!first) {
 // even if chain is empty, still refresh id counter from spec to avoid
 // duplicates later
 RefreshNextCubeIdFromSpec();
 return;
 }

 // traverse linear chain: assume each cube has at most one cube child
 mjsBody* cur = first;
 mjsBody* parent = world;

 while (cur) {
 const char* nm = mjs_getString(mjs_getName(cur->element));
 std::string name = nm ? nm : std::string();

 // update counter with any existing cube id encountered
 int id = ParseCubeId(nm);
 if (id >=0 && id > g_nextCubeId) g_nextCubeId = id;

 ChainEntry entry{};
 entry.specBody = cur;
 entry.name = name;
 entry.bodyId = -1;

 // compute axis/sign relative to parent (skip for first/root)
 if (parent && parent != cur) {
 int axis =0;
 int sign = +1;

 double axv[3] = {std::fabs(cur->pos[0]), std::fabs(cur->pos[1]),
 std::fabs(cur->pos[2])};
 if (axv[1] >= axv[0] && axv[1] >= axv[2]) {
 axis =1;
 } else if (axv[2] >= axv[0] && axv[2] >= axv[1]) {
 axis =2;
 } else {
 axis =0;
 }

 sign = ((axis ==0 ? cur->pos[0]
 : (axis ==1 ? cur->pos[1] : cur->pos[2])) >=0.0)
 ? +1
 : -1;

 entry.axis = axis;
 entry.sign = sign;
 } else {
 entry.axis = -1;
 entry.sign = +1;
 }

 g_chain.push_back(entry);

 // last marker and probe on last body if present
 std::string markerName = name + std::string("_spawn_marker");
 if (mjsElement* mel = mjs_findElement(spec, mjOBJ_GEOM, markerName.c_str())) {
 g_lastMarker = mjs_asGeom(mel);
 }

 std::string probeName = name + std::string("_spawn_probe");
 if (mjsElement* pel = mjs_findElement(spec, mjOBJ_GEOM, probeName.c_str())) {
 g_probeRect = mjs_asGeom(pel);
 g_probeName = probeName;
 }

 // find next cube child
 mjsBody* next = nullptr;
 for (mjsElement* el = mjs_firstChild(cur, mjOBJ_BODY, /*recurse=*/0);
 el; el = mjs_nextChild(cur, el, /*recurse=*/0)) {
 if (el->elemtype == mjOBJ_BODY) {
 const char* cnm = mjs_getString(mjs_getName(el));
 if (cnm && std::strncmp(cnm, "cube_",5) ==0) {
 next = mjs_asBody(el);
 break;
 }
 }
 }

 if (!next) {
 g_lastBody = cur;
 break;
 }

 parent = cur;
 cur = next;
 }
}

bool LoadChainFromFile(const char* filename) {
if (!filename || std::strlen(filename) ==0) return false;

 char error[1024] = {0};
	 mjSpec* newspec = mj_parseXML(filename, nullptr, error, sizeof(error));
	 if (!newspec) {
	 std::cerr << "LoadChainFromFile: parse error: " << error << "\n";
	 return false;
 }

 // load persisted gap from parsed spec (before compile)
 LoadGapFromSpec(newspec);

 mjModel* newm = mj_compile(newspec, nullptr);
 if (!newm) {
	 std::cerr << "LoadChainFromFile: compile error: "
	 << mjs_getError(newspec) << "\n";
	 mj_deleteSpec(newspec);
	 return false;
 }

 mjData* newd = mj_makeData(newm);
 if (!newd) {
	 std::cerr << "LoadChainFromFile: mj_makeData failed\n";
	 mj_deleteModel(newm);
	 mj_deleteSpec(newspec);
	 return false;
 }

 // swap in new globals
 if (d) mj_deleteData(d);
 if (m) mj_deleteModel(m);
 if (spec) mj_deleteSpec(spec);

 spec = newspec;
 m = newm;
 d = newd;

 // ensure future cube names are unique by syncing id counter with loaded spec
 RefreshNextCubeIdFromSpec();

 // gravity bookkeeping
 g_savedGravity[0] = m->opt.gravity[0];
 g_savedGravity[1] = m->opt.gravity[1];
 g_savedGravity[2] = m->opt.gravity[2];

 g_physicsEnabled = false;
 ApplyBuildModeOptions();

 // reconstruct the chain vector by traversing spec
 ReconstructChainFromSpec();

 // refresh runtime structures
 mj_resetData(m, d);
 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
 UpdateMarkerOnLastBody();

 // Refresh g_nextCubeId based on loaded spec content
 RefreshNextCubeIdFromSpec();

 std::cout << "Loaded chain from XML: " << filename << "\n";
 return true;
}
