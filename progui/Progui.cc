#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector> // added
#include <cmath> // NEW: for std::sqrt, std::atan2
#include <unordered_map> // NEW: for bodyId -> chain index map

#include <mujoco/mjdata.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>

#include <GLFW/glfw3.h>


// MuJoCo data structures
mjModel* m = NULL; // MuJoCo model
mjData* d = NULL; // MuJoCo data
mjSpec* spec = NULL; // Editable model specification (mjSpec) for model editing
mjvCamera cam; // abstract camera
mjvOption opt; // visualization options
mjvScene scn; // abstract scene
mjrContext con; // custom GPU context

// UI structures
mjuiState uistate; // UI state
mjUI ui; // UI object

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx =0;
double lasty =0;

// global toggle
static bool g_boxesCollide = false;

// joint mode toggle: choose between two hinges or a single ball joint
enum JointMode { JOINT_HINGE_2 =0, JOINT_BALL =1 };
static int g_jointMode = JOINT_BALL; // default:2 hinges

// helper: rebuild rendering scene/context after model changes
static void RebuildRenderContext()
{
 // free old
 mjv_freeScene(&scn);
 mjr_freeContext(&con);

 // recreate scene and context for the (possibly) changed model
 mjv_defaultScene(&scn);
 mjr_defaultContext(&con);
 mjv_makeScene(m, &scn,2000);
 mjr_makeContext(m, &con, mjFONTSCALE_150);

 // recompute UI sizes as font atlas changed
 mjui_resize(&ui, &con);

 // IMPORTANT: re-create the UI aux buffer after context recreation
 mjr_addAux(ui.auxid, ui.width, ui.maxheight, ui.spacing.samples, &con);
}

// unique id for spawned cubes
static int g_nextCubeId =0;

// fixed UI width used for layout and hit-testing
static int g_uiWidth =220;

// chain entry holds spec body pointer, its name, and the compiled body id for fast lookup
struct ChainEntry {
 mjsBody* specBody;
 std::string name;
 int bodyId; // compiled model body id (updated after recompile)
};

// track last spawned body in the editable spec
static mjsBody* g_lastBody = nullptr;

// track all spawned bodies (in order) so we can translate the whole chain
static std::vector<ChainEntry> g_chain;

// map compiled body id -> index in g_chain for O(1) lookup
static std::unordered_map<int, size_t> g_bodyIdToIndex;

// marker for newest box and selected face for next spawn
static mjsGeom* g_lastMarker = nullptr;

// green probe rectangle on newest box used to pre-check collisions toward next spawn direction
static mjsGeom* g_probeRect = nullptr;
static std::string g_probeName;

// damping and contact tuning (tweak as needed)
static const double kHingeDamping =1.0; // N*m*s/rad, viscous damping on each hinge
static const double kGeomMargin =0.001; // small contact margin (meters)
static const double kSolref[2] = {0.03,1.0 }; // [timeconst, dampratio]
static const double kSolimp[5] = {0.9,0.95,0.001,0.5,2.0 }; // contact impedance

// box size used for movement step (matches spawnCube half-size)
static const double kBoxHalf =0.05; // meters


// spawn/dynamics toggle: when false, all spawned boxes are welded; when enabled, we add joints
static bool g_physicsEnabled = false;
static mjtNum g_savedGravity[3] = {0,0,0};

// ensure build-mode options (gravity off) are applied after any recompile
static inline void ApplyBuildModeOptions()
{
 if (!m) return;
 if (!g_physicsEnabled) {
 // zero gravity and set disable flag so nothing moves during build
 m->opt.gravity[0] =0.0;
 m->opt.gravity[1] =0.0;
 m->opt.gravity[2] =0.0;
 m->opt.disableflags |= mjDSBL_GRAVITY;
 }
}

enum SpawnFace {
 FACE_POSX =0, FACE_NEGX =1,
 FACE_POSY =2, FACE_NEGY =3,
 FACE_POSZ =4, FACE_NEGZ =5
};
static int g_spawnFace = FACE_POSX;

// helpers for face/axis/sign and marker placement
static inline void FaceToAxisSign(int face, int& axisIdx, int& sign)
{
 switch (face) {
 case FACE_POSX: axisIdx =0; sign = +1; break;
 case FACE_NEGX: axisIdx =0; sign = -1; break;
 case FACE_POSY: axisIdx =1; sign = +1; break;
 case FACE_NEGY: axisIdx =1; sign = -1; break;
 case FACE_POSZ: axisIdx =2; sign = +1; break;
 case FACE_NEGZ: axisIdx =2; sign = -1; break;
 default: axisIdx =0; sign = +1; break;
 }
}
static inline void AxisUnit(int axisIdx, double out[3])
{
 out[0] = (axisIdx ==0) ?1.0 :0.0;
 out[1] = (axisIdx ==1) ?1.0 :0.0;
 out[2] = (axisIdx ==2) ?1.0 :0.0;
}

// rebuild the compiled body id mapping for fast lookup from contacts
static void UpdateBodyIdIndexMap()
{
 g_bodyIdToIndex.clear();
 for (size_t i =0; i < g_chain.size(); ++i) {
 int id = mj_name2id(m, mjOBJ_BODY, g_chain[i].name.c_str());
 g_chain[i].bodyId = id;
 if (id >=0) g_bodyIdToIndex[id] = i;
 }
}

// reposition the red marker on the newest box to the selected face
static void UpdateMarkerOnLastBody()
{
 if (!spec || !m || !d || !g_lastBody || !g_lastMarker) return;

 const double boxWidth =2.0 * kBoxHalf;
 const double sphereR =0.1 * boxWidth;
 double pos[3] = {0.0, 0.0, 0.0};

 int axisIdx = 0, sign = +1;
 FaceToAxisSign(g_spawnFace, axisIdx, sign);

 pos[axisIdx] = sign * (kBoxHalf - 0.5 * sphereR);
 g_lastMarker->pos[0] = pos[0];
 g_lastMarker->pos[1] = pos[1];
 g_lastMarker->pos[2] = pos[2];

 // Reconfigure probe as a ghost cube at the next spawn location (one width + gap)
 if (g_probeRect) {
 const double boxEdge = 2.0 * kBoxHalf;
 const double gap     = 0.05 * boxEdge;

 g_probeRect->type    = mjGEOM_BOX;
 g_probeRect->size[0] = kBoxHalf;
 g_probeRect->size[1] = kBoxHalf;
 g_probeRect->size[2] = kBoxHalf;

 double ppos[3] = {0.0, 0.0, 0.0};
 ppos[axisIdx]  = sign * (boxEdge + gap);  // center where next cube would spawn
 g_probeRect->pos[0] = ppos[0];
 g_probeRect->pos[1] = ppos[1];
 g_probeRect->pos[2] = ppos[2];

 g_probeRect->rgba[0] = 0.1f; g_probeRect->rgba[1] = 1.0f; g_probeRect->rgba[2] = 0.1f; g_probeRect->rgba[3] = 0.9f;
 g_probeRect->contype = 4;    // probe category
 g_probeRect->conaffinity = 1;// collide with boxes
 g_probeRect->density = 0.0;
 g_probeRect->margin  = kGeomMargin;
 }



 int old_nv = m->nv, old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) != 0) {
 std::cerr << "UpdateMarkerOnLastBody: mj_recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }
 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] = 0;
 for (int i = old_na; i < m->na; ++i) d->act[i]  = 0;
 mj_forward(m, d);
 RebuildRenderContext();
}

// Prints contacts that involve the probe and maps to chain index/name.
static void DebugPrintProbeContacts()
{
 if (!m || !d || g_probeName.empty()) return;

 const int probeGeomId = mj_name2id(m, mjOBJ_GEOM, g_probeName.c_str());
 if (probeGeomId <0) { std::cout << "Probe geom not found\n"; return; }

 // Pure geometry pass: compute kinematics and collisions only
 mj_kinematics(m, d);
 mj_collision(m, d);

 std::cout << "contacts total: " << d->ncon << "\n";
 for (int i =0; i < d->ncon; ++i) {
 const mjContact& c = d->contact[i];
 const bool involvesProbe = (c.geom1 == probeGeomId || c.geom2 == probeGeomId);
 if (!involvesProbe) continue;

 const int otherGeom = (c.geom1 == probeGeomId ? c.geom2 : c.geom1);
 if (otherGeom <0 || otherGeom >= m->ngeom) continue;

 const int otherBody = m->geom_bodyid[otherGeom];
 const char* gname1 = mj_id2name(m, mjOBJ_GEOM, c.geom1);
 const char* gname2 = mj_id2name(m, mjOBJ_GEOM, c.geom2);
 const char* bname = mj_id2name(m, mjOBJ_BODY, otherBody);

 auto it = g_bodyIdToIndex.find(otherBody);
 if (it != g_bodyIdToIndex.end()) {
 std::cout << "Probe contact with chainIndex=" << it->second
 << " body=" << (bname ? bname : "<noname>")
 << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", "
 << (gname2 ? gname2 : "<g2>") << ")\n";
 }
 else {
 std::cout << "Probe contact with bodyId=" << otherBody
 << " body=" << (bname ? bname : "<noname>")
 << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", "
 << (gname2 ? gname2 : "<g2>") << ")\n";
 }
 }
}

// detect which compiled body id (if any) the probe is touching; returns -1 if none
static int ProbeGetHitBodyId()
{
 if (!m || !d) return -1;
 if (g_probeName.empty()) return -1;
 int probeId = mj_name2id(m, mjOBJ_GEOM, g_probeName.c_str());
 if (probeId <0) return -1;

 // latest (newest) body id to ignore self-contact (should not occur anyway)
 int newestBodyId = -1;
 if (!g_chain.empty()) newestBodyId = mj_name2id(m, mjOBJ_BODY, g_chain.back().name.c_str());

 // Pure geometry pass: compute kinematics and collisions only
 mj_kinematics(m, d);
 mj_collision(m, d);

 for (int i =0; i < d->ncon; ++i) {
	 const mjContact& c = d->contact[i];
	 int g1 = c.geom1;
	 int g2 = c.geom2;
		 if (g1 == probeId || g2 == probeId) {
		 int otherGeom = (g1 == probeId ? g2 : g1);
			 if (otherGeom >=0 && otherGeom < m->ngeom) {
			 // check if other is a box (contype has bit1) and not same body as probe
			 int otherBody = m->geom_bodyid[otherGeom];
				 if ((m->geom_contype[otherGeom] &1) && otherBody != newestBodyId) {
				 return otherBody;
				 }
			 }
		 }
	 }
 return -1;
}

// Function to spawn a box by editing mjSpec and recompiling in-place
void spawnCube()
{
 if (!spec) {
 std::cerr << "spawnCube: editable spec is null (likely loaded .mjb). Load an XML to enable runtime editing.\n";
 return;
 }
 if (!m || !d) {
 std::cerr << "spawnCube: model/data not initialized.\n";
 return;
 }

 // box config
 const double halfSize[3] = { kBoxHalf, kBoxHalf, kBoxHalf }; // cube half-size
 const double boxEdge =2.0 * kBoxHalf; // one box width
 const double gap =0.05 * boxEdge; //5% gap

 // find world body
 mjsBody* world = mjs_findBody(spec, "world");
 if (!world) {
 std::cerr << "spawnCube: could not find world body in spec.\n";
 return;
 }

 // parent is world for the first cube (static), otherwise the last spawned cube
 mjsBody* parent = g_lastBody ? g_lastBody : world;
 
 // track obstruction and target
 bool obstructed = false;
 std::string targetBodyName;
 int targetBodyId = -1;
 
 // create a new body under parent
 mjsBody* body = mjs_addBody(parent, nullptr);
 if (!body) {
 std::cerr << "spawnCube: failed to add body to spec.\n";
 return;
 }
 
 // give it a unique name
 std::string bodyName = "cube_" + std::to_string(++g_nextCubeId);
 if (mjs_setName(body->element, bodyName.c_str()) !=0) {
 std::cerr << "spawnCube: failed to set body name: " << bodyName << "\n";
 }
 
 // set body pose:
 // - first cube: at {0,0,1} in world (stationary; welded to world)
 // - next cubes: offset along selected face axis by one edge plus5% gap relative to parent,
 // or two edges+gap if probe says obstructed
 int spawnAxis =0, spawnSign = +1;
 FaceToAxisSign(g_spawnFace, spawnAxis, spawnSign);
 double delta = (boxEdge + gap);
 if (g_lastBody && g_probeRect) {
 int hitBodyId = ProbeGetHitBodyId();
 if (hitBodyId >=0) {
 // try to resolve to chain index/name for fast reference
 auto it = g_bodyIdToIndex.find(hitBodyId);
 const char* hitName = mj_id2name(m, mjOBJ_BODY, hitBodyId);
 if (it != g_bodyIdToIndex.end()) {
 std::cout << "Probe hit: bodyId=" << hitBodyId << ", chainIndex=" << it->second
 << ", name=" << (hitName ? hitName : "<noname>") << "\n";
 } else {
 std::cout << "Probe hit: bodyId=" << hitBodyId << ", name=" << (hitName ? hitName : "<noname>") << "\n";
 }
 delta = (2.0 * boxEdge + gap); // push one extra box width away if obstructed
 obstructed = true;
 targetBodyId = hitBodyId;
 if (hitName) targetBodyName = hitName;
 }
 }
 
 if (!g_lastBody) {
 body->pos[0] =0.0; body->pos[1] =0.0; body->pos[2] =1.0;
 } else {
 const double off = spawnSign * delta;
 body->pos[0] = (spawnAxis ==0) ? off :0.0;
 body->pos[1] = (spawnAxis ==1) ? off :0.0;
 body->pos[2] = (spawnAxis ==2) ? off :0.0;
 }
 body->quat[0] =1.0; body->quat[1] =0.0; body->quat[2] =0.0; body->quat[3] =0.0;
 
 // joints: always add physics joints, but gravity may be off until 'P' pressed
 if (!g_lastBody) {
 // root: free joint to world
 if (mjsJoint* fj = mjs_addJoint(body, nullptr)) {
 fj->type = mjJNT_FREE;
 fj->damping = kHingeDamping;
 std::string jname = bodyName + "_free";
 mjs_setName(fj->element, jname.c_str());
 }
 } else {
 // If obstructed, we will connect via equality constraints to the target body;
 // still allow local DOFs by adding a ball/hinge only when not obstructed
 if (!obstructed) {
 // common anchor: child's face toward the parent (opposite the spawn direction)
 int faceAxis =0, faceSign = +1;
 FaceToAxisSign(g_spawnFace, faceAxis, faceSign);
 const double boxEdge2 =2.0 * kBoxHalf;
 const double gap2 =0.05 * boxEdge2;
 double anchor[3] = {0.0,0.0,0.0};
 anchor[faceAxis] = -faceSign * (kBoxHalf +0.5 * gap2);
 
 if (g_jointMode == JOINT_BALL) {
 // Single3-DoF ball joint
 if (mjsJoint* bj = mjs_addJoint(body, nullptr)) {
 bj->type = mjJNT_BALL;
 bj->pos[0] = anchor[0]; bj->pos[1] = anchor[1]; bj->pos[2] = anchor[2];
 bj->damping = kHingeDamping;
 std::string jname = bodyName + "_ball";
 mjs_setName(bj->element, jname.c_str());
 } else {
 std::cerr << "spawnCube: failed to add ball joint.\n";
 return;
 }
 } else {
 // Two hinges: Y then X (with gap-based symmetric limits)
 if (mjsJoint* hjY = mjs_addJoint(body, nullptr)) {
 hjY->type = mjJNT_HINGE;
 hjY->axis[0] =0.0; hjY->axis[1] =1.0; hjY->axis[2] =0.0;
 hjY->pos[0] = anchor[0]; hjY->pos[1] = anchor[1]; hjY->pos[2] = anchor[2];
 hjY->damping = kHingeDamping;
 
 const double r_perp_y = std::sqrt(kBoxHalf*kBoxHalf + kBoxHalf*kBoxHalf);
 double theta_max_y = std::atan2(std::max(1e-6,0.5*gap2), std::max(1e-6, r_perp_y));
 const double th_min = mjPI *5.0 /180.0;
 const double th_max = mjPI *80.0 /180.0;
 theta_max_y = std::min(std::max(theta_max_y, th_min), th_max);
 hjY->limited = mjLIMITED_TRUE;
 hjY->range[0] = -theta_max_y;
 hjY->range[1] = theta_max_y;
 
 std::string jnameY = bodyName + "_hinge_y";
 mjs_setName(hjY->element, jnameY.c_str());
 } else {
 std::cerr << "spawnCube: failed to add Y hinge joint.\n";
 return;
 }
 
 if (mjsJoint* hjX = mjs_addJoint(body, nullptr)) {
 hjX->type = mjJNT_HINGE;
 hjX->axis[0] =1.0; hjX->axis[1] =0.0; hjX->axis[2] =0.0;
 hjX->pos[0] = anchor[0]; hjX->pos[1] = anchor[1]; hjX->pos[2] = anchor[2];
 hjX->damping = kHingeDamping;
 
 const double r_perp_x = std::sqrt(kBoxHalf*kBoxHalf + kBoxHalf*kBoxHalf);
 double theta_max_x = std::atan2(std::max(1e-6,0.5*gap2), std::max(1e-6, r_perp_x));
 const double th_min = mjPI *5.0 /180.0;
 const double th_max = mjPI *80.0 /180.0;
 theta_max_x = std::min(std::max(theta_max_x, th_min), th_max);
 hjX->limited = mjLIMITED_TRUE;
 hjX->range[0] = -theta_max_x;
 hjX->range[1] = theta_max_x;
 
 std::string jnameX = bodyName + "_hinge_x";
 mjs_setName(hjX->element, jnameX.c_str());
 } else {
 std::cerr << "spawnCube: failed to add X hinge joint.\n";
 return;
 }
 }
 } else {
 // obstructed: no joint is added here; the connection is handled via
 // equality CONNECT constraints to the target body (ball-joint-like)
 }
 }
 
 // add a box geom to the body
 mjsGeom* geom = mjs_addGeom(body, nullptr);
 if (!geom) {
 std::cerr << "spawnCube: failed to add geom.\n";
 return;
 }
 geom->type = mjGEOM_BOX;
 geom->size[0] = halfSize[0];
 geom->size[1] = halfSize[1];
 geom->size[2] = halfSize[2];
 geom->density =1000.0; // kg/m^3 (water)

 // box color: white
 geom->rgba[0] =1.0;
 geom->rgba[1] =1.0;
 geom->rgba[2] =1.0;
 geom->rgba[3] =1.0;

 // ensure boxes participate in correct collisions per g_boxesCollide
 if (g_boxesCollide) {
 geom->contype =1; // box category
 geom->conaffinity =1 |2 |4; // collide with boxes, floor and probe
 // typical friction
 geom->friction[0] =1.0;
 geom->friction[1] =0.005;
 geom->friction[2] =0.0001;
 } else {
 geom->contype =1; // keep box category
 geom->conaffinity =2 |4; // collide with floor and probe category
 // friction irrelevant for box-box (disabled) but fine to keep
 }

 // soften contacts slightly and add a tiny margin
 geom->margin = kGeomMargin;
 geom->solref[0] = kSolref[0];
 geom->solref[1] = kSolref[1];
 geom->solimp[0] = kSolimp[0];
 geom->solimp[1] = kSolimp[1];
 geom->solimp[2] = kSolimp[2];
 geom->solimp[3] = kSolimp[3];
 geom->solimp[4] = kSolimp[4];

 // add a small red marker sphere on selected face (next spawn direction), partially embedded
 if (mjsGeom* marker = mjs_addGeom(body, nullptr)) {
 marker->type = mjGEOM_SPHERE;

 const double boxWidth =2.0 * kBoxHalf;
 const double sphereRadius =0.1 * boxWidth; //10% of box width (radius)
 marker->size[0] = sphereRadius;

 // place at selected face
 marker->pos[0] =0.0; marker->pos[1] =0.0; marker->pos[2] =0.0;
 int axisIdx =0, sign = +1;
 FaceToAxisSign(g_spawnFace, axisIdx, sign);
 marker->pos[axisIdx] = sign * (kBoxHalf -0.5 * sphereRadius);

 // sphere color: red
 marker->rgba[0] =1.0;
 marker->rgba[1] =0.1;
 marker->rgba[2] =0.1;
 marker->rgba[3] =1.0;

 // render-only: no collisions, no mass contribution
 marker->contype =0; // stays non-colliding
 marker->conaffinity =0;
 marker->density =0.0;

 std::string sname = bodyName + "_spawn_marker";
 mjs_setName(marker->element, sname.c_str());

 // remember marker for interactive face changes
 g_lastMarker = marker;
 }

 // create or move the green probe onto the newest box (single instance)
 // if there was a previous probe, disable its collisions and hide it
 if (g_probeRect) {
 g_probeRect->contype = 0;
 g_probeRect->conaffinity = 0;
 g_probeRect->rgba[3] = 0.0f;
 }
 if (mjsGeom* probe = mjs_addGeom(body, nullptr)) {
    int axisIdx = 0, sign = +1;
    FaceToAxisSign(g_spawnFace, axisIdx, sign);
    const double boxEdge = 2.0 * kBoxHalf;
    const double gap     = 0.05 * boxEdge;

    probe->type = mjGEOM_BOX;
    // same half-sizes as a cube
    probe->size[0] = kBoxHalf;
    probe->size[1] = kBoxHalf;
    probe->size[2] = kBoxHalf;

    // center where the next cube would be if placed at 1x width + gap
    probe->pos[0] = 0.0; probe->pos[1] = 0.0; probe->pos[2] = 0.0;
    probe->pos[axisIdx] = sign * (boxEdge + gap);

    probe->rgba[0] = 0.1f; probe->rgba[1] = 1.0f; probe->rgba[2] = 0.1f; probe->rgba[3] = 0.9f;
    probe->contype = 4;     // probe category
    probe->conaffinity = 1; // collide with boxes
    probe->density = 0.0;
    probe->margin  = kGeomMargin;

    g_probeRect = probe;
    g_probeName = bodyName + "_spawn_probe";
    mjs_setName(probe->element, g_probeName.c_str());
 }

 // If obstructed, add connect equalities to target so that both the previous
 // and the new cube are ball-connected to the target cube at face centers.
 if (obstructed && g_lastBody && !targetBodyName.empty()) {
 // axis/sign already defined by g_spawnFace
 int axisIdx = spawnAxis;
 int sign = spawnSign;
 // anchors in local coordinates
 double prev_anchor[3] = {0,0,0};
 double new_anchor[3] = {0,0,0};
 double tgt_anchor_to_prev[3] = {0,0,0};
 double tgt_anchor_to_new[3] = {0,0,0};
 prev_anchor[axisIdx] = sign * kBoxHalf; // prev face towards target
 new_anchor[axisIdx] = -sign * kBoxHalf; // new face towards target
 tgt_anchor_to_prev[axisIdx] = -sign * kBoxHalf; // target face towards prev
 tgt_anchor_to_new[axisIdx] = sign * kBoxHalf; // target face towards new
 
 // equality: previous <-> target
 if (!g_chain.empty()) {
 const std::string& prevName = g_chain.back().name;
 if (mjsEquality* eq1 = mjs_addEquality(spec, nullptr)) {
 eq1->type = mjEQ_CONNECT;
 eq1->objtype = mjOBJ_BODY;
 eq1->active =1;
 eq1->solref[0] = kSolref[0]; eq1->solref[1] = kSolref[1];
 for (int i=0;i<5;++i) eq1->solimp[i] = kSolimp[i];
 mjs_setString(eq1->name1, prevName.c_str());
 mjs_setString(eq1->name2, targetBodyName.c_str());
 eq1->data[0] = prev_anchor[0]; eq1->data[1] = prev_anchor[1]; eq1->data[2] = prev_anchor[2];
 eq1->data[3] = tgt_anchor_to_prev[0]; eq1->data[4] = tgt_anchor_to_prev[1]; eq1->data[5] = tgt_anchor_to_prev[2];
 std::string ename1 = prevName + "__to__" + targetBodyName + "_connect";
 mjs_setName(eq1->element, ename1.c_str());
 }
 }
 
 // equality: new <-> target
 if (mjsEquality* eq2 = mjs_addEquality(spec, nullptr)) {
 eq2->type = mjEQ_CONNECT;
 eq2->objtype = mjOBJ_BODY;
 eq2->active =1;
 eq2->solref[0] = kSolref[0]; eq2->solref[1] = kSolref[1];
 for (int i=0;i<5;++i) eq2->solimp[i] = kSolimp[i];
 mjs_setString(eq2->name1, bodyName.c_str());
 mjs_setString(eq2->name2, targetBodyName.c_str());
 eq2->data[0] = new_anchor[0]; eq2->data[1] = new_anchor[1]; eq2->data[2] = new_anchor[2];
 eq2->data[3] = tgt_anchor_to_new[0]; eq2->data[4] = tgt_anchor_to_new[1]; eq2->data[5] = tgt_anchor_to_new[2];
 std::string ename2 = bodyName + "__to__" + targetBodyName + "_connect";
 mjs_setName(eq2->element, ename2.c_str());
 }
 }
 
 // remember this body for the next link
 g_lastBody = body;
 g_chain.push_back(ChainEntry{ body, bodyName, -1 });
 
 // recompile in-place, preserving existing state
 int old_nq = m->nq;
 int old_nv = m->nv;
 int old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "mj_recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }
 
 ApplyBuildModeOptions();
 // initialize only the newly-added dofs/activations; keep previous state intact
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;
 
 // refresh derived fields without resetting the simulation
 mj_forward(m, d);
 
 // refresh body-id map after each recompile
 UpdateBodyIdIndexMap();
 
 // rebuild GL contexts to reflect changed model (counts, assets, etc.)
 RebuildRenderContext();
 
 std::cout << "Cube spawned: " << bodyName
 << (parent == world
 ? " (root " + std::string(g_physicsEnabled ? "free" : "static") + ")"
 : (g_physicsEnabled
 ? (g_jointMode == JOINT_BALL ? " (ball)" : " (hinges)")
 : " (welded)"))
 << "\n";
}

// helper: delete all equality constraints that reference the given body name
static void DeleteEqualitiesReferencing(const std::string& bodyName)
{
 if (!spec) return;
 std::vector<mjsElement*> toDelete;
 for (mjsElement* el = mjs_firstElement(spec, mjOBJ_EQUALITY); el; el = mjs_nextElement(spec, el)) {
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

// Delete the most recent box (not the root). Also cleans up branch-around constraints and
// restores marker/probe on the new last box.
static void deleteLastCube()
{
 if (!spec || !m || !d) return;
 if (g_chain.size() <=1) {
 // never delete the first (root) box
 std::cerr << "deleteLastCube: root box cannot be deleted.\n";
 return;
 }

 // last and previous entries
 ChainEntry last = g_chain.back();
 ChainEntry prev = g_chain[g_chain.size() -2];

 // remove equalities that reference the last body (handles branch-around case)
 DeleteEqualitiesReferencing(last.name);

 // delete the last body from spec
 if (last.specBody) {
 mjs_delete(spec, last.specBody->element);
 }

 // update chain / last pointers
 g_chain.pop_back();
 g_lastBody = prev.specBody;

 // reset marker and probe pointers to the previous body's ones (by name)
 g_lastMarker = nullptr;
 g_probeRect = nullptr;
 std::string prevMarkerName = prev.name + std::string("_spawn_marker");
 std::string prevProbeName = prev.name + std::string("_spawn_probe");
 if (mjsElement* mel = mjs_findElement(spec, mjOBJ_GEOM, prevMarkerName.c_str())) {
 g_lastMarker = mjs_asGeom(mel);
 }
 if (mjsElement* pel = mjs_findElement(spec, mjOBJ_GEOM, prevProbeName.c_str())) {
 g_probeRect = mjs_asGeom(pel);
 g_probeName = prevProbeName;
 if (g_probeRect) {
 // re-enable previous probe visualization and collisions
 g_probeRect->contype =4;
 g_probeRect->conaffinity =1;
 g_probeRect->rgba[0] =0.1f; g_probeRect->rgba[1] =1.0f; g_probeRect->rgba[2] =0.1f; g_probeRect->rgba[3] =0.9f;
 g_probeRect->density =0.0;
 g_probeRect->margin = kGeomMargin;
 }
 }

 // recompile to apply deletion
 int old_nv = m->nv;
 int old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "deleteLastCube: mj_recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }
 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;
 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();

 // reposition marker/probe on the new last body according to current face
 UpdateMarkerOnLastBody();
}

// Move the most recently spawned (hinged) cube by +/- one box width along Y, without resetting sim
static void moveLastCubeY(int dir)
{
 if (!spec || !m || !d) {
 std::cerr << "moveLastCubeY: model/spec not initialized.\n";
 return;
 }
 if (g_chain.empty()) {
 std::cerr << "moveLastCubeY: no cubes to move.\n";
 return;
 }
 // Require at least one dynamic (hinged) cube; root may be static
 if (g_chain.size() <2) {
 std::cerr << "moveLastCubeY: only root cube exists (static); spawn another cube first.\n";
 return;
 }

 const double step =2.0 * kBoxHalf; // one box width
 const double dy = (dir >0 ? step : -step);

 // distribute translation along the chain:
 // body i moves by ((i+1)/N) * dy so the last body moves full dy and the root moves dy/N
 const size_t N = g_chain.size();
 const double invN =1.0 / static_cast<double>(N);
 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;
 const double factor = static_cast<double>(i +1) * invN;
 b->pos[1] += dy * factor;
 }

 // recompile in-place, preserve state
 int old_nv = m->nv;
 int old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "moveLastCubeY: mj_recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }
 // zero any newly added dofs/activations (none expected here, but safe)
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] =0;
 for (int i = old_na; i < m->na; ++i) d->act[i] =0;

 ApplyBuildModeOptions();
 // refresh derived data and rendering
 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
}

// Move the most recently spawned (hinged) cube by +/- one box width along X, without resetting sim
static void moveLastCubeX(int dir)
{
 if (!spec || !m || !d) {
 std::cerr << "moveLastCubeX: model/spec not initialized.\n";
 return;
 }
 if (g_chain.empty()) {
 std::cerr << "moveLastCubeX: no cubes to move.\n";
 return;
 }
 if (g_chain.size() <2) {
 std::cerr << "moveLastCubeX: only root cube exists (static); spawn another cube first.\n";
 return;
 }

 const double step =2.0 * kBoxHalf; // one box width
 const double dx = (dir >0 ? step : -step);

 const size_t N = g_chain.size();
 const double invN =1.0 / static_cast<double>(N);
 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;
 const double factor = static_cast<double>(i +1) * invN;
 b->pos[0] += dx * factor;
 }

 int old_nv = m->nv;
 int old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "moveLastCubeX: mj_recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }
 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] = 0;
 for (int i = old_na; i < m->na; ++i) d->act[i] = 0;

 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
}

// Move the most recently spawned (hinged) cube by +/- one box width along Z, without resetting sim
static void moveLastCubeZ(int dir)
{
 if (!spec || !m || !d) {
 std::cerr << "moveLastCubeZ: model/spec not initialized.\n";
 return;
 }
 if (g_chain.empty()) {
 std::cerr << "moveLastCubeZ: no cubes to move.\n";
 return;
 }
 if (g_chain.size() <2) {
 std::cerr << "moveLastCubeZ: only root cube exists (static); spawn another cube first.\n";
 return;
 }

 const double step =2.0 * kBoxHalf; // one box width
 const double dz = (dir >0 ? step : -step);

 const size_t N = g_chain.size();
 const double invN =1.0 / static_cast<double>(N);
 for (size_t i =0; i < N; ++i) {
 mjsBody* b = g_chain[i].specBody;
 if (!b) continue;
 const double factor = static_cast<double>(i +1) * invN;
 b->pos[2] += dz * factor;
 }

 int old_nv = m->nv;
 int old_na = m->na;
 if (mj_recompile(spec, nullptr, m, d) !=0) {
 std::cerr << "moveLastCubeZ: mj_recompile failed: " << mjs_getError(spec) << "\n";
 return;
 }
 ApplyBuildModeOptions();
 for (int i = old_nv; i < m->nv; ++i) d->qvel[i] = 0;
 for (int i = old_na; i < m->na; ++i) d->act[i] = 0;

 mj_forward(m, d);
 UpdateBodyIdIndexMap();
 RebuildRenderContext();
}


// Enable dynamics for all boxes already spawned:
// - root gets a free joint to world
// - other boxes get hinge Y, hinge X, or a ball joint depending on g_jointMode
static void EnablePhysicsForAll()
{
 if (!spec || !m || !d) {
 std::cerr << "EnablePhysicsForAll: model/spec not initialized.\n";
 return;
 }
 if (g_chain.empty()) {
 std::cerr << "EnablePhysicsForAll: no boxes to enable.\n";
 return;
 }
 if (g_physicsEnabled) {
 // already enabled; nothing to do
 return;
 }

 // restore saved gravity and step once
 m->opt.gravity[0] = g_savedGravity[0];
 m->opt.gravity[1] = g_savedGravity[1];
 m->opt.gravity[2] = g_savedGravity[2];
 m->opt.disableflags &= ~mjDSBL_GRAVITY;
 mj_forward(m, d);
 RebuildRenderContext();

 g_physicsEnabled = true;
}

// UI button callback (keyboard alternative remains the primary trigger)
void uiSpawnCube(mjUI* /*ui_*/, int state)
{
 if (state) {
 spawnCube();
 }
}

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
 // let UI consume first
 if (mjui_event(&ui, &uistate, &con)) {
 return;
 }

 // backspace: reset simulation
 if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
 mj_resetData(m, d);
 mj_forward(m, d);
 }

 // Toggle physics for all boxes
 if (act == GLFW_PRESS && key == GLFW_KEY_P) {
 EnablePhysicsForAll();
 return;
 }

 // Delete key: delete the most recent box (but never the root box)
 if (act == GLFW_PRESS && key == GLFW_KEY_DELETE) {
 deleteLastCube();
 return;
 }

 // Face selection:1..6 => +X, -X, +Y, -Y, +Z, -Z (moves the red marker on newest box)
 if (act == GLFW_PRESS) {
 if (key == GLFW_KEY_1) { g_spawnFace = FACE_POSX; UpdateMarkerOnLastBody(); return; }
 if (key == GLFW_KEY_2) { g_spawnFace = FACE_NEGX; UpdateMarkerOnLastBody(); return; }
 if (key == GLFW_KEY_3) { g_spawnFace = FACE_POSY; UpdateMarkerOnLastBody(); return; }
 if (key == GLFW_KEY_4) { g_spawnFace = FACE_NEGY; UpdateMarkerOnLastBody(); return; }
 if (key == GLFW_KEY_5) { g_spawnFace = FACE_POSZ; UpdateMarkerOnLastBody(); return; }
 if (key == GLFW_KEY_6) { g_spawnFace = FACE_NEGZ; UpdateMarkerOnLastBody(); return; }
 }

 // 'C' key: spawn cube at the face indicated by the red marker
 if (act == GLFW_PRESS && key == GLFW_KEY_C) {
 spawnCube();
 }

 // Up/Down arrows: move last cube along +Y/-Y
 if (act == GLFW_PRESS && key == GLFW_KEY_UP) { moveLastCubeY(+1); }
 if (act == GLFW_PRESS && key == GLFW_KEY_DOWN) { moveLastCubeY(-1); }

 // Right/Left arrows: move last cube along +X/-X
 if (act == GLFW_PRESS && key == GLFW_KEY_RIGHT) { moveLastCubeX(+1); }
 if (act == GLFW_PRESS && key == GLFW_KEY_LEFT) { moveLastCubeX(-1); }

 // W/S keys: move last cube along +Z/-Z
 if (act == GLFW_PRESS && key == GLFW_KEY_W) { moveLastCubeZ(+1); }
 if (act == GLFW_PRESS && key == GLFW_KEY_S) { moveLastCubeZ(-1); }

 // Debug probe contacts
 if (act == GLFW_PRESS && key == GLFW_KEY_H) {
 DebugPrintProbeContacts();
 }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
 // update button state
 button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
 button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
 button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

 // current cursor and framebuffer size
 glfwGetCursorPos(window, &lastx, &lasty);
 int fbw =0, fbh =0;
 glfwGetFramebufferSize(window, &fbw, &fbh);

 // populate minimal mjuiState for this mouse event
 uistate.type = (act == GLFW_PRESS) ? mjEVENT_PRESS : mjEVENT_RELEASE;
 uistate.left = button_left;
 uistate.right = button_right;
 uistate.middle = button_middle;
 uistate.button =
 (button == GLFW_MOUSE_BUTTON_LEFT) ? mjBUTTON_LEFT :
 (button == GLFW_MOUSE_BUTTON_RIGHT) ? mjBUTTON_RIGHT :
 (button == GLFW_MOUSE_BUTTON_MIDDLE) ? mjBUTTON_MIDDLE : mjBUTTON_NONE;

 // convert y to bottom-left origin
 uistate.x = lastx;
 uistate.y = fbh - lasty;

 // modifiers
 uistate.shift = (mods & GLFW_MOD_SHIFT) ?1 :0;
 uistate.control = (mods & GLFW_MOD_CONTROL) ?1 :0;
 uistate.alt = (mods & GLFW_MOD_ALT) ?1 :0;

 // describe the UI rect so mjui can hit-testing
 uistate.nrect =1;
 // use actual ui.width (at least g_uiWidth) to avoid clipping UI items
 int uiw = ui.width; if (uiw < g_uiWidth) uiw = g_uiWidth;
 uistate.rect[0] = { fbw - uiw,0, uiw, fbh };
 ui.rectid =0;

 // let UI handle the event
 if (mjuiItem* changed = mjui_event(&ui, &uistate, &con)) {
 if (act == GLFW_PRESS) {
 if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Spawn Cube (C)") ==0) {
 spawnCube();
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Start Physics (P)") ==0) { // NEW
 EnablePhysicsForAll();
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Move Last +Y (Up)") ==0) {
 moveLastCubeY(+1);
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Move Last -Y (Down)") ==0) {
 moveLastCubeY(-1);
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Move Last +X (Right)") ==0) {
 moveLastCubeX(+1);
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Move Last -X (Left)") ==0) {
 moveLastCubeX(-1);
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Move Last +Z (W)") ==0) {
 moveLastCubeZ(+1);
 } else if (changed->type == mjITEM_BUTTON &&
 std::strcmp(changed->name, "Move Last -Z (S)") ==0) {
 moveLastCubeZ(-1);
 }
 }
 return; // UI consumed the event
 }
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
 // pass to UI first
 if (mjui_event(&ui, &uistate, &con)) {
 return;
 }

 // no buttons down: nothing to do
 if (!button_left && !button_middle && !button_right) {
 return;
 }

 // compute mouse displacement, save
 double dx = xpos - lastx;
 double dy = ypos - lasty;
 lastx = xpos;
 lasty = ypos;

 // get current window size
 int width, height;
 glfwGetWindowSize(window, &width, &height);

 // get shift key state
 bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
 glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

 // determine action based on mouse button
 mjtMouse action;
 if (button_right) {
 action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
 }
 else if (button_left) {
 action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
 }
 else {
 action = mjMOUSE_ZOOM;
 }

 // move camera
 mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
 // let UI consume first (scroll over UI)
 if (mjui_event(&ui, &uistate, &con)) {
 return;
 }

 // emulate vertical mouse motion =5% of window height
 mjv_moveCamera(m, mjMOUSE_ZOOM,0, -0.05 * yoffset, &scn, &cam);
}

int main(int argc, const char** argv)
{
 // check command-line arguments
 std::string modelfile = "cube_3x3x3.xml";
 if (argc ==2) {
 modelfile = argv[1];
 std::cout << "printing argv[1]: " << argv[1] << std::endl;
 } else {
 std::printf(" USAGE: basic cube file\n");
 }

 // init GLFW
 if (!glfwInit()) {
 mju_error("Could not initialize GLFW");
 }

 // create window, make OpenGL context current, request v-sync
 GLFWwindow* window = glfwCreateWindow(1200,900, "ProGUI - Cube Spawner", NULL, NULL);
 glfwMakeContextCurrent(window);
 glfwSwapInterval(1);

 // initialize visualization data structures
 mjv_defaultCamera(&cam);
 mjv_defaultOption(&opt);
 mjv_defaultScene(&scn);
 mjr_defaultContext(&con);

 // load model through mjSpec (XML) to enable runtime editing; fallback to direct load for .mjb
 char error[1000] = "Could not load model";
 if (modelfile.size() >4 && !std::strcmp(modelfile.c_str() + modelfile.size() -4, ".mjb")) {
 std::cout << "loading mjb file (runtime editing disabled)" << std::endl;
 m = mj_loadModel(modelfile.c_str(),0);
 if (!m) {
 mju_error("Could not load mjb model");
 }
 d = mj_makeData(m);
 } else {
 std::cout << "loading xml file (runtime editing enabled)" << std::endl;
 spec = mj_parseXML(modelfile.c_str(), nullptr, error, sizeof(error));
 if (!spec) {
 mju_error("Load spec error: %s", error);
 }
 m = mj_compile(spec, nullptr);
 if (!m) {
 mju_error("Compile spec error: %s", mjs_getError(spec));
 }
 d = mj_makeData(m);
 }

 // start a new chain on each (re)load
 g_lastBody = nullptr;
 g_nextCubeId =0;
 g_chain.clear();

 // cache gravity and disable it for build mode
 g_savedGravity[0] = m->opt.gravity[0];
 g_savedGravity[1] = m->opt.gravity[1];
 g_savedGravity[2] = m->opt.gravity[2];
 m->opt.gravity[0] =0.0;
 m->opt.gravity[1] =0.0;
 m->opt.gravity[2] =0.0;
 mj_forward(m, d);

 // create scene and context
 mjv_makeScene(m, &scn,2000);
 mjr_makeContext(m, &con, mjFONTSCALE_150);

 // initialize UI - zero structures
 memset(&uistate,0, sizeof(mjuiState));
 memset(&ui,0, sizeof(mjUI));

 // use readable theme
 ui.spacing = mjui_themeSpacing(1);
 ui.color = mjui_themeColor(1);
 ui.predicate = nullptr;
 ui.rectid =1;
 ui.auxid =0;
 ui.radiocol =0;

 // accent the button color so it stands out
 ui.color.button[0] =0.15f; // R
 ui.color.button[1] =0.55f; // G
 ui.color.button[2] =0.95f; // B

 // create UI section for controls
 mjuiDef defControl[] = {
 { mjITEM_SECTION, "Controls",1, nullptr, "" },
 { mjITEM_BUTTON, "Spawn Cube (C)",2, nullptr, "" },
 { mjITEM_BUTTON, "Start Physics (P)",2, nullptr, "" }, // renamed
 { mjITEM_BUTTON, "Move Last +Y (Up)",2, nullptr, "" },
 { mjITEM_BUTTON, "Move Last -Y (Down)",2, nullptr, "" },
 { mjITEM_BUTTON, "Move Last +X (Right)",2, nullptr, "" },
 { mjITEM_BUTTON, "Move Last -X (Left)",2, nullptr, "" },
 { mjITEM_BUTTON, "Move Last +Z (W)",2, nullptr, "" },
 { mjITEM_BUTTON, "Move Last -Z (S)",2, nullptr, "" },
 { mjITEM_END }
 };

 // add UI section and compute sizes
 mjui_add(&ui, defControl);
 mjui_resize(&ui, &con);

 // allocate the UI aux buffer used for drawing the UI image
 mjr_addAux(ui.auxid, ui.width, ui.maxheight, ui.spacing.samples, &con);

 // install GLFW mouse and keyboard callbacks
 glfwSetKeyCallback(window, keyboard);
 glfwSetCursorPosCallback(window, mouse_move);
 glfwSetMouseButtonCallback(window, mouse_button);
 glfwSetScrollCallback(window, scroll);

 // main loop
 while (!glfwWindowShouldClose(window)) {
 // advance interactive simulation for1/60 sec
 mjtNum simstart = d->time;
 if (g_physicsEnabled) {
 while (d->time - simstart <1.0 /60.0) {
 mj_step(m, d);
 }
 } else {
 // build mode: keep state frozen, only update transforms and contacts once per frame
 mj_kinematics(m, d);
 mj_collision(m, d);
 }

 // get framebuffer viewport
 mjrRect viewport = {0,0,0,0 };
 glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

 // Layout UI (right side) using actual ui.width (at least g_uiWidth)
 ui.rectid =0;
 uistate.nrect =1;
 int uiw = ui.width; if (uiw < g_uiWidth) uiw = g_uiWidth;
 uistate.rect[0] = { viewport.width - uiw,0, uiw, viewport.height };

 // update UI image into its aux buffer, then render scene
 mjui_update(-1, -1, &ui, &uistate, &con);

 mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
 mjr_render(viewport, &scn, &con);

 // render UI overlay by blitting from the aux buffer
 mjui_render(&ui, &uistate, &con);

 // swap OpenGL buffers (blocking call due to v-sync)
 glfwSwapBuffers(window);

 // process pending GUI events, call GLFW callbacks
 glfwPollEvents();
 }

 // free visualization storage
 mjv_freeScene(&scn);
 mjr_freeContext(&con);

 // free MuJoCo model and data
 mj_deleteData(d);
 mj_deleteModel(m);

 // free spec if used
 if (spec) {
 mj_deleteSpec(spec);
 spec = nullptr;
 }

 // terminate GLFW
#if defined(__APPLE__) || defined(_WIN32)
 glfwTerminate();
#endif

 return EXIT_SUCCESS;
}

