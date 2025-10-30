#include "progui_globals.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

void RebuildRenderContext()
{
    // free old
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // recreate
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // recompute UI sizes as font atlas changed
    mjui_resize(&ui, &con);

    // re-create the UI aux buffer after context recreation
    mjr_addAux(ui.auxid, ui.width, ui.maxheight, ui.spacing.samples, &con);
}

void ApplyBuildModeOptions()
{
    if (!m)
        return;

    if (!g_physicsEnabled)
    {
        m->opt.gravity[0] = 0.0;
        m->opt.gravity[1] = 0.0;
        m->opt.gravity[2] = 0.0;
        m->opt.disableflags |= mjDSBL_GRAVITY;
    }
}

void FaceToAxisSign(int face, int &axisIdx, int &sign)
{
    switch (face)
    {
    case FACE_POSX:
        axisIdx = 0;
        sign = +1;
        break;

    case FACE_NEGX:
        axisIdx = 0;
        sign = -1;
        break;

    case FACE_POSY:
        axisIdx = 1;
        sign = +1;
        break;

    case FACE_NEGY:
        axisIdx = 1;
        sign = -1;
        break;

    case FACE_POSZ:
        axisIdx = 2;
        sign = +1;
        break;

    case FACE_NEGZ:
        axisIdx = 2;
        sign = -1;
        break;

    default:
        axisIdx = 0;
        sign = +1;
        break;
    }
}

void UpdateBodyIdIndexMap()
{
    g_bodyIdToIndex.clear();

    for (size_t i = 0; i < g_chain.size(); ++i)
    {
        int id = mj_name2id(m, mjOBJ_BODY, g_chain[i].name.c_str());
        g_chain[i].bodyId = id;
        if (id >= 0)
            g_bodyIdToIndex[id] = i;
    }
}

void DebugPrintProbeContacts()
{
    if (!m || !d || g_probeName.empty())
        return;

    const int probeGeomId = mj_name2id(m, mjOBJ_GEOM, g_probeName.c_str());
    if (probeGeomId < 0)
    {
        std::cout << "Probe geom not found\n";
        return;
    }

    mj_kinematics(m, d);
    mj_collision(m, d);

    std::cout << "contacts total: " << d->ncon << "\n";
    for (int i = 0; i < d->ncon; ++i)
    {
        const mjContact &c = d->contact[i];
        const bool involvesProbe = (c.geom1 == probeGeomId || c.geom2 == probeGeomId);
        if (!involvesProbe)
            continue;

        const int otherGeom = (c.geom1 == probeGeomId ? c.geom2 : c.geom1);
        if (otherGeom < 0 || otherGeom >= m->ngeom)
            continue;

        const int otherBody = m->geom_bodyid[otherGeom];
        const char *gname1 = mj_id2name(m, mjOBJ_GEOM, c.geom1);
        const char *gname2 = mj_id2name(m, mjOBJ_GEOM, c.geom2);
        const char *bname = mj_id2name(m, mjOBJ_BODY, otherBody);

        auto it = g_bodyIdToIndex.find(otherBody);
        if (it != g_bodyIdToIndex.end())
        {
            std::cout << "Probe contact with chainIndex=" << it->second << " body=" << (bname ? bname : "<noname>")
                      << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", " << (gname2 ? gname2 : "<g2>") << ")\n";
        }
        else
        {
            std::cout << "Probe contact with bodyId=" << otherBody << " body=" << (bname ? bname : "<noname>")
                      << " geoms=(" << (gname1 ? gname1 : "<g1>") << ", " << (gname2 ? gname2 : "<g2>") << ")\n";
        }
    }
}

int ProbeGetHitBodyId()
{
    if (!m || !d)
        return -1;

    if (g_probeName.empty())
        return -1;

    int probeId = mj_name2id(m, mjOBJ_GEOM, g_probeName.c_str());
    if (probeId < 0)
        return -1;

    int newestBodyId = -1;
    if (!g_chain.empty())
        newestBodyId = mj_name2id(m, mjOBJ_BODY, g_chain.back().name.c_str());

    mj_kinematics(m, d);
    mj_collision(m, d);

    for (int i = 0; i < d->ncon; ++i)
    {
        const mjContact &c = d->contact[i];
        int g1 = c.geom1;
        int g2 = c.geom2;

        if (g1 == probeId || g2 == probeId)
        {
            int otherGeom = (g1 == probeId ? g2 : g1);
            if (otherGeom >= 0 && otherGeom < m->ngeom)
            {
                int otherBody = m->geom_bodyid[otherGeom];
                if ((m->geom_contype[otherGeom] & 1) && otherBody != newestBodyId)
                {
                    return otherBody;
                }
            }
        }
    }

    return -1;
}

static void RecomputeJointLimitsForChain()
{
    if (!spec || !m || !d)
        return;

    const double boxEdge = 2.0 * kBoxHalf;
    const double gap = g_gapRatio * boxEdge;
    const double r_perp = std::sqrt(kBoxHalf * kBoxHalf + kBoxHalf * kBoxHalf);

    // iterate all joints in spec
    for (mjsElement *jel = mjs_firstElement(spec, mjOBJ_JOINT); jel; jel = mjs_nextElement(spec, jel))
    {
        mjsJoint *j = mjs_asJoint(jel);
        if (!j)
            continue;
        if (j->type == mjJNT_BALL)
        {
            const double a = kBoxHalf;
            const double c = 1.0 + (a > 0.0 ? (gap / a) : 0.0);
            double phi = 0.0;
            const double root2 = std::sqrt(2.0);
            if (c >= root2)
                phi = mjPI / 2.0;
            else
            {
                double s = c / root2;
                s = std::max(0.0, std::min(1.0, s));
                phi = std::asin(s) - (mjPI / 4.0);
            }
            const double th_min = mjPI * 5.0 / 180.0;
            const double th_max = mjPI * 80.0 / 180.0;
            phi = std::max(th_min, std::min(th_max, phi));
            j->limited = mjLIMITED_TRUE;
            j->range[0] = 0.0;
            j->range[1] = phi;
        }
        else if (j->type == mjJNT_HINGE)
        {
            double theta = std::atan2(std::max(1e-6, 0.5 * gap), std::max(1e-6, r_perp));
            const double th_min = mjPI * 5.0 / 180.0;
            const double th_max = mjPI * 80.0 / 180.0;
            theta = std::max(th_min, std::min(th_max, theta));
            j->limited = mjLIMITED_TRUE;
            j->range[0] = -theta;
            j->range[1] = +theta;
        }
    }
}

static void ApplyGapToChainLayout()
{
    if (!spec)
        return;
    const double boxEdge = 2.0 * kBoxHalf;
    const double gap = g_gapRatio * boxEdge;
    // update positions for all non-root bodies using stored axis/sign
    if (g_chain.size() <= 1)
        return;
    for (size_t i = 1; i < g_chain.size(); ++i)
    {
        ChainEntry &e = g_chain[i];
        mjsBody *b = e.specBody;
        if (!b)
            continue;
        const int axis = e.axis;
        const int sign = e.sign;
        b->pos[0] = 0.0;
        b->pos[1] = 0.0;
        b->pos[2] = 0.0;
        double off = sign * (boxEdge + gap);
        if (axis == 0)
            b->pos[0] = off;
        else if (axis == 1)
            b->pos[1] = off;
        else
            b->pos[2] = off;

        // update joint anchors on this body (if they exist) by name convention
        // ball
        {
            std::string jn = e.name + "_ball";
            if (mjsElement *jel = mjs_findElement(spec, mjOBJ_JOINT, jn.c_str()))
            {
                mjsJoint *j = mjs_asJoint(jel);
                if (j && j->type == mjJNT_BALL)
                {
                    double anchor[3] = {0, 0, 0};
                    int faceAxis = axis, faceSign = sign;
                    anchor[faceAxis] = -faceSign * (kBoxHalf + 0.5 * gap);
                    j->pos[0] = anchor[0];
                    j->pos[1] = anchor[1];
                    j->pos[2] = anchor[2];
                }
            }
        }
        // hinges
        {
            std::string jx = g_chain[i].name + "_hinge_x";
            if (mjsElement *jel = mjs_findElement(spec, mjOBJ_JOINT, jx.c_str()))
            {
                mjsJoint *j = mjs_asJoint(jel);
                if (j && j->type == mjJNT_HINGE)
                {
                    double anchor[3] = {0, 0, 0};
                    anchor[e.axis] = -e.sign * (kBoxHalf + 0.5 * gap);
                    j->pos[0] = anchor[0];
                    j->pos[1] = anchor[1];
                    j->pos[2] = anchor[2];
                }
            }
            std::string jy = g_chain[i].name + "_hinge_y";
            if (mjsElement *jel = mjs_findElement(spec, mjOBJ_JOINT, jy.c_str()))
            {
                mjsJoint *j = mjs_asJoint(jel);
                if (j && j->type == mjJNT_HINGE)
                {
                    double anchor[3] = {0, 0, 0};
                    anchor[e.axis] = -e.sign * (kBoxHalf + 0.5 * gap);
                    j->pos[0] = anchor[0];
                    j->pos[1] = anchor[1];
                    j->pos[2] = anchor[2];
                }
            }
        }
    }
}

void IncreaseGap()
{
    g_gapRatio = std::min(1.0, g_gapRatio + 0.05);
    ApplyGapToChainLayout();
    RecomputeJointLimitsForChain();
    if (g_probeRect)
    {
        int axisIdx = 0, sign = +1;
        FaceToAxisSign(g_spawnFace, axisIdx, sign);
        const double boxEdge = 2.0 * kBoxHalf;
        const double gap = g_gapRatio * boxEdge;
        g_probeRect->pos[0] = 0;
        g_probeRect->pos[1] = 0;
        g_probeRect->pos[2] = 0;
        g_probeRect->pos[axisIdx] = sign * (boxEdge + gap);
    }
    int old_nv = m->nv, old_na = m->na;
    if (mj_recompile(spec, nullptr, m, d) != 0)
        return;
    for (int i = old_nv; i < m->nv; ++i) d->qvel[i] = 0;
    for (int i = old_na; i < m->na; ++i) d->act[i] = 0;
    ApplyBuildModeOptions();
    mj_forward(m, d);
    RebuildRenderContext();
}

void DecreaseGap()
{
    g_gapRatio = std::max(0.0, g_gapRatio - 0.05);
    ApplyGapToChainLayout();
    RecomputeJointLimitsForChain();
    if (g_probeRect)
    {
        int axisIdx = 0, sign = +1;
        FaceToAxisSign(g_spawnFace, axisIdx, sign);
        const double boxEdge = 2.0 * kBoxHalf;
        const double gap = g_gapRatio * boxEdge;
        g_probeRect->pos[0] = 0;
        g_probeRect->pos[1] = 0;
        g_probeRect->pos[2] = 0;
        g_probeRect->pos[axisIdx] = sign * (boxEdge + gap);
    }
    int old_nv = m->nv, old_na = m->na;
    if (mj_recompile(spec, nullptr, m, d) != 0)
        return;
    for (int i = old_nv; i < m->nv; ++i) d->qvel[i] = 0;
    for (int i = old_na; i < m->na; ++i) d->act[i] = 0;
    ApplyBuildModeOptions();
    mj_forward(m, d);
    RebuildRenderContext();
}
