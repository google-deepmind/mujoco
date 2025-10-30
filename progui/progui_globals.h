#pragma once
#include <GLFW/glfw3.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <string>
#include <unordered_map>
#include <vector>

// Global MuJoCo handles
extern mjModel *m;
extern mjData *d;
extern mjSpec *spec;
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
struct ChainEntry
{
 mjsBody *specBody;
 std::string name;
 int bodyId;
 int axis; // spawn axis this body was placed along relative to its parent
 int sign; // +1 or -1 direction along axis
};

extern mjsBody *g_lastBody;
extern std::vector<ChainEntry> g_chain;
extern std::unordered_map<int, size_t> g_bodyIdToIndex;

// Marker and probe
extern mjsGeom *g_lastMarker;
extern mjsGeom *g_probeRect;
extern std::string g_probeName;

// Gap control
extern double g_gapRatio; // ratio of gap to box width, default0.05
extern char g_gapRatioLabel[64];
void IncreaseGap();
void DecreaseGap();

// Simple IO filename buffer for save/load
extern char g_ioFilename[256];
extern int g_fileIoMode; //0: none,1: save,2: load

// Pre-physics saved state (positions/orientations per body)
struct SavedBodyPos
{
 std::string name;
 double pos[3];
 double quat[4];
};

extern bool g_hasSavedPrePhysicsState;
extern std::vector<SavedBodyPos> g_savedPrePhysicsChain;

// Softer defaults and constants
extern const double kHingeDamping;
extern const double kGeomMargin;
extern const double kSolref[2];
extern const double kSolimp[5];
extern const double kBoxHalf;

// UI layout
extern int g_uiWidth;

// Spawn faces
enum SpawnFace
{
 FACE_POSX =0,
 FACE_NEGX =1,
 FACE_POSY =2,
 FACE_NEGY =3,
 FACE_POSZ =4,
 FACE_NEGZ =5,
};

extern int g_spawnFace;

// Shared helpers
void RebuildRenderContext();
void ApplyBuildModeOptions();
void FaceToAxisSign(int face, int &axisIdx, int &sign);
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

// Save/load full chain via XML spec
bool SaveChainToFile(const char* filename);
bool LoadChainFromFile(const char* filename);

// New feature: save/restore chain state around physics start
void SaveChainPrePhysicsState();
void ResetChainToSavedState();

// UI handlers
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow *window, int button, int act, int mods);
void mouse_move(GLFWwindow *window, double xpos, double ypos);
void scroll(GLFWwindow *window, double xoffset, double yoffset);

// Global button labels to keep UI text and handlers in sync
extern const char* kBtnSpawnCube;
extern const char* kBtnStartPhysics;
extern const char* kBtnResetChain;
extern const char* kBtnIncreaseGap;
extern const char* kBtnDecreaseGap;
extern const char* kBtnSaveChain;
extern const char* kBtnLoadChain;
// Additional direct file actions used in handlers
extern const char* kBtnSaveToFile;
extern const char* kBtnLoadFromFile;
