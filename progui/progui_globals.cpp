#include "progui_globals.h"
#include <fstream>
#include <sstream>

// Definitions of globals
mjModel* m = nullptr;
mjData* d = nullptr;
mjSpec* spec = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

mjuiState uistate;
mjUI ui;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx =0.0;
double lasty =0.0;

bool g_boxesCollide = false;
int g_jointMode =1; // JOINT_BALL by default
bool g_physicsEnabled = false;
mjtNum g_savedGravity[3] = {0,0,0};

mjsBody* g_lastBody = nullptr;
std::vector<ChainEntry> g_chain;
std::unordered_map<int, size_t> g_bodyIdToIndex;

mjsGeom* g_lastMarker = nullptr;
mjsGeom* g_probeRect = nullptr;
std::string g_probeName;

// gap control
double g_gapRatio =0.05; // default5% of box width
char g_gapRatioLabel[64] = "gap:0.05";

// IO filename buffer
char g_ioFilename[256] = "chain_save.xml";
int g_fileIoMode =0;

// pre-physics state
bool g_hasSavedPrePhysicsState = false;
std::vector<SavedBodyPos> g_savedPrePhysicsChain;

// Softer defaults: increase constraint time constant and reduce joint damping
const double kHingeDamping =0.15; // was1.0
const double kGeomMargin =0.001;
const double kSolref[2] = {0.15,0.7}; // was {0.03,1.0}
const double kSolimp[5] = {0.9,0.95,0.001,0.5,2.0};
const double kBoxHalf =0.05;

int g_uiWidth =220;

int g_spawnFace = FACE_POSX;

// Global button label definitions
const char* kBtnSpawnCube = "Spawn Cube (C)";
const char* kBtnStartPhysics= "Start Physics (P)";
const char* kBtnResetChain = "Reset Chain (R)";
const char* kBtnIncreaseGap = "Increase Gap (+)";
const char* kBtnDecreaseGap = "Decrease Gap (-)";
const char* kBtnSaveChain = "Save Chain (Ctrl+S)";
const char* kBtnLoadChain = "Load Chain (Ctrl+L)";
const char* kBtnSaveToFile = "Save To File";
const char* kBtnLoadFromFile= "Load From File";
