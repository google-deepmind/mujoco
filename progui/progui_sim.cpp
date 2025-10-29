#include "progui_globals.h"
#include <algorithm>
#include <iostream>

void RebuildRenderContext()
{
 // free old
 mjv_freeScene(&scn);
 mjr_freeContext(&con);
 // recreate
 mjv_defaultScene(&scn);
 mjr_defaultContext(&con);
 mjv_makeScene(m, &scn,2000);
 mjr_makeContext(m, &con, mjFONTSCALE_150);
 // recompute UI sizes as font atlas changed
 mjui_resize(&ui, &con);
 // re-create the UI aux buffer after context recreation
 mjr_addAux(ui.auxid, ui.width, ui.maxheight, ui.spacing.samples, &con);
}

void ApplyBuildModeOptions()
{
 if (!m) return;
 if (!g_physicsEnabled) {
 m->opt.gravity[0] =0.0;
 m->opt.gravity[1] =0.0;
 m->opt.gravity[2] =0.0;
 m->opt.disableflags |= mjDSBL_GRAVITY;
 }
}

void FaceToAxisSign(int face, int& axisIdx, int& sign)
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

void UpdateBodyIdIndexMap()
{
 g_bodyIdToIndex.clear();
 for (size_t i =0; i < g_chain.size(); ++i) {
 int id = mj_name2id(m, mjOBJ_BODY, g_chain[i].name.c_str());
 g_chain[i].bodyId = id;
 if (id >=0) g_bodyIdToIndex[id] = i;
 }
}

void DebugPrintProbeContacts()
{
 if (!m || !d || g_probeName.empty()) return;
 const int probeGeomId = mj_name2id(m, mjOBJ_GEOM, g_probeName.c_str());
 if (probeGeomId <0) { std::cout << "Probe geom not found\n"; return; }
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
 } else {
 std::cout << "Probe contact with bodyId=" << otherBody
 << " body=" << (bname ? bname : "<noname>")
 << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", "
 << (gname2 ? gname2 : "<g2>") << ")\n";
 }
 }
}

int ProbeGetHitBodyId()
{
 if (!m || !d) return -1;
 if (g_probeName.empty()) return -1;
 int probeId = mj_name2id(m, mjOBJ_GEOM, g_probeName.c_str());
 if (probeId <0) return -1;
 int newestBodyId = -1;
 if (!g_chain.empty()) newestBodyId = mj_name2id(m, mjOBJ_BODY, g_chain.back().name.c_str());
 mj_kinematics(m, d);
 mj_collision(m, d);
 for (int i =0; i < d->ncon; ++i) {
 const mjContact& c = d->contact[i];
 int g1 = c.geom1; int g2 = c.geom2;
 if (g1 == probeId || g2 == probeId) {
 int otherGeom = (g1 == probeId ? g2 : g1);
 if (otherGeom >=0 && otherGeom < m->ngeom) {
 int otherBody = m->geom_bodyid[otherGeom];
 if ((m->geom_contype[otherGeom] &1) && otherBody != newestBodyId) {
 return otherBody;
 }
 }
 }
 }
 return -1;
}
