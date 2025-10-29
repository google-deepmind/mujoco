#pragma once
#include <mujoco/mujoco.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <GLFW/glfw3.h>
#include <string>
#include <vector>
#include <unordered_map>

// Global MuJoCo handles
extern mjModel* m;
extern mjData* d;
extern mjSpec* spec;
extern mjvCamera cam;
extern mjvOption opt;
extern mjvScene scn;
extern mjrContext con;

// UI state
extern mjuiState uistate;
extern mjUI ui;

// Mouse state
extern bool button_left;
extern bool button_middle;
extern bool button_right;
extern double lastx;
extern double lasty;

// Build / physics toggles
extern bool g_boxesCollide;
extern int g_jointMode; //0: hinges,1: ball
extern bool g_physicsEnabled;
extern mjtNum g_savedGravity[3];

// Chain structures
struct ChainEntry {
 mjsBody* specBody;
 std::string name;
 int bodyId;
};
extern mjsBody* g_lastBody;
extern std::vector<ChainEntry> g_chain;
extern std::unordered_map<int, size_t> g_bodyIdToIndex;

// Marker and probe
extern mjsGeom* g_lastMarker;
extern mjsGeom* g_probeRect;
extern std::string g_probeName;

// Constants / config
extern const double kHingeDamping;
extern const double kGeomMargin;
extern const double kSolref[2];
extern const double kSolimp[5];
extern const double kBoxHalf;

// UI layout
extern int g_uiWidth;

// Spawn faces
enum SpawnFace {
 FACE_POSX =0, FACE_NEGX =1,
 FACE_POSY =2, FACE_NEGY =3,
 FACE_POSZ =4, FACE_NEGZ =5
};
extern int g_spawnFace;

// Helpers shared across translation units
void RebuildRenderContext();
void ApplyBuildModeOptions();
void FaceToAxisSign(int face, int& axisIdx, int& sign);
void UpdateBodyIdIndexMap();
void UpdateMarkerOnLastBody();
int ProbeGetHitBodyId();
void DebugPrintProbeContacts();

// Chain ops
void spawnCube();
void deleteLastCube();
void moveLastCubeX(int dir);
void moveLastCubeY(int dir);
void moveLastCubeZ(int dir);
void EnablePhysicsForAll();

// UI handlers
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);
