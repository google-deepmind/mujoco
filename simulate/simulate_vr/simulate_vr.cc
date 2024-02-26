#include "simulate_vr.h"

#include "glad\glad.h"

#pragma comment(lib, "openvr_api.lib")

#include <GLFW/glfw3.h>

using namespace vr;

// initialization funcitons
void mjvrHMD::initHmd() {
   int n;

   // check for headset 
   if (!vr::VR_IsHmdPresent()) {
       mju_warning("VR Headset not found");
       return;
   }

   // initialize runtime
   vr::EVRInitError err = vr::VRInitError_None;
   system = VR_Init(&err, vr::VRApplication_Scene);
   if (err != vr::VRInitError_None) {
       mju_warning("Could not init VR runtime: %s", VR_GetVRInitErrorAsEnglishDescription(err));
       return;
   }

   // initialize compositor, set to seated
   if (!vr::VRCompositor())
   {
       vr::VR_Shutdown();
       mju_warning("Could not init VR Compositor.");
       return;
   }

   // get recommended image size
   system->GetRecommendedRenderTargetSize(&width, &height);

   // check all devices, find hmd
   device_id = -1;
   for (n = 0; n < vr::k_unMaxTrackedDeviceCount; n++)
   {
       vr::ETrackedDeviceClass cls = system->GetTrackedDeviceClass(n);

       // found HMD
       if (cls == vr::TrackedDeviceClass_HMD)
           device_id = n;
   }

   // require HMD
   if (device_id < 0) {
       mju_warning("VR headset not found.");
       return;
   }

   // init HMD pose data
   for (n = 0; n < 9; n++)
   {
       roommat[n] = 0;
       if (n < 3)
           roompos[n] = 0;
   }
   roommat[0] = 1;
   roommat[4] = 1;
   roommat[8] = 1;

   // get HMD eye-to-head offsets (no rotation)
   for (n = 0; n < 2; n++)
   {
       vr::HmdMatrix34_t tmp = system->GetEyeToHeadTransform((vr::EVREye)n);
       eyeoffset[n][0] = tmp.m[0][3];
       eyeoffset[n][1] = tmp.m[1][3];
       eyeoffset[n][2] = tmp.m[2][3];
   }

   if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
       mju_warning("Failed to initialize OpenGL context for VR.");
       return;
   }
   initialized = true;
}

void mjvrHMD::initTextures(mjvScene& scn) {
    if (!initialized)
        return;
    // create vr texture
    glActiveTexture(GL_TEXTURE2);
    glGenTextures(1, &idtex);
    glBindTexture(GL_TEXTURE_2D, idtex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 2 * width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
}

void mjvrHMD::copyPose(const vr::TrackedDevicePose_t* pose, float* roompos, float* roommat)
{
   // nothing to do if not tracked
   if (!pose->bPoseIsValid)
       return;

   // pointer to data for convenience
   const HmdMatrix34_t* p = &pose->mDeviceToAbsoluteTracking;

   // raw data: room
   roompos[0] = p->m[0][3];
   roompos[1] = p->m[1][3];
   roompos[2] = p->m[2][3];
   roommat[0] = p->m[0][0];
   roommat[1] = p->m[0][1];
   roommat[2] = p->m[0][2];
   roommat[3] = p->m[1][0];
   roommat[4] = p->m[1][1];
   roommat[5] = p->m[1][2];
   roommat[6] = p->m[2][0];
   roommat[7] = p->m[2][1];
   roommat[8] = p->m[2][2];
}


// update vr poses and controller states
void mjvrHMD::update(mjvScene& scn)
{
    if (!initialized)
        return;
   int n, i;
   //mjvGeom* g;

   // get new poses
   TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount];
   VRCompositor()->WaitGetPoses(poses, k_unMaxTrackedDeviceCount, NULL, 0);

   // copy hmd pose
   mjvrHMD::copyPose(poses + device_id, roompos, roommat);

   // adjust OpenGL scene cameras to match hmd pose
   for (n = 0; n < 2; n++)
   {
       // assign position, apply eye-to-head offset
       for (i = 0; i < 3; i++)
           scn.camera[n].pos[i] = roompos[i] +
           eyeoffset[n][0] * roommat[3 * i + 0] +
           eyeoffset[n][1] * roommat[3 * i + 1] +
           eyeoffset[n][2] * roommat[3 * i + 2];

       // assign forward and up
       scn.camera[n].forward[0] = -roommat[2];
       scn.camera[n].forward[1] = -roommat[5];
       scn.camera[n].forward[2] = -roommat[8];
       scn.camera[n].up[0] = roommat[1];
       scn.camera[n].up[1] = roommat[4];
       scn.camera[n].up[2] = roommat[7];
   }

}


// render to vr and window
void mjvrHMD::render(mjrContext& con, mjuiState& uistate)
{
    if (!initialized)
        return;
   // resolve multi-sample offscreen buffer
   glBindFramebuffer(GL_READ_FRAMEBUFFER, con.offFBO);
   glReadBuffer(GL_COLOR_ATTACHMENT0);
   glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
   glDrawBuffer(GL_COLOR_ATTACHMENT0);
   glBlitFramebuffer(
       0, 0, 2 * width, height,
       0, 0, 2 * width, height,
       GL_COLOR_BUFFER_BIT, GL_NEAREST);

   // blit to vr texture
   glActiveTexture(GL_TEXTURE2);
   glBindFramebuffer(GL_DRAW_FRAMEBUFFER, con.offFBO_r);
   glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, idtex, 0);
   glDrawBuffer(GL_COLOR_ATTACHMENT1);
   glBlitFramebuffer(
       0, 0, 2 * width, height,
       0, 0, 2 * width, height,
       GL_COLOR_BUFFER_BIT, GL_NEAREST);
   glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, 0, 0);
   glDrawBuffer(GL_COLOR_ATTACHMENT0);

   // submit to vr
   const VRTextureBounds_t boundLeft = { 0, 0, 0.5, 1 };
   const VRTextureBounds_t boundRight = { 0.5, 0, 1, 1 };
   Texture_t vTex = { (void*)idtex, TextureType_OpenGL, ColorSpace_Gamma };
   VRCompositor()->Submit(Eye_Left, &vTex, &boundLeft);
   VRCompositor()->Submit(Eye_Right, &vTex, &boundRight);

   // display onscreen
   mjrRect rect = uistate.rect[3];
   mjrRect rectVR;
   rectVR.left = 0;
   rectVR.bottom = 0;
   rectVR.width = (int)width;
   rectVR.height = (int)height;
   mjr_blitBuffer(rectVR, rect, 1, 0, &con);

}


// close vr
void mjvrHMD::close()
{
    // crash inside OpenVR ???
   glDeleteTextures(1, &idtex);

   VR_Shutdown();
}

void mjvrHMD::transform(mjvScene& scn, mjModel* m)
{
    if (!initialized)
        return;

    scn.stereo = mjSTEREO_SIDEBYSIDE;

    // set MuJoCo OpenGL frustum to match Vive
    for (int n = 0; n < 2; n++)
    {
        // get frustum from vr
        float left, right, top, bottom, znear = 0.05f, zfar = 50.0f;
        system->GetProjectionRaw((EVREye)n, &left, &right, &top, &bottom);

        // set in MuJoCo
        scn.camera[n].frustum_bottom = -bottom * znear;
        scn.camera[n].frustum_top = -top * znear;
        scn.camera[n].frustum_center = 0.5f * (left + right) * znear;
        scn.camera[n].frustum_near = znear;
        scn.camera[n].frustum_far = zfar;
    }

    // initialize model transform
    scn.enabletransform = 1;

    // if any vr_coords were found, apply them,
    // otherwise use default
    if (n_vr_transforms > 0)
        vr_transform_i = 0;
    else
        vr_transform_i = -1;

    applyVrTransform(scn, m);
}

void mjvrHMD::setOBuffer(mjVisual& vis)
{
    if (!initialized)
        return;
    vis.global.offwidth = (int)(2 * width);
    vis.global.offheight = (int) height;
    vis.quality.offsamples = 8;
}

void mjvrHMD::findVrTransformations(mjModel* m)
{
    if (!initialized)
        return;
    n_vr_transforms = 0;
    if (!m)
        return;
    for (size_t i_num = 0; i_num < m->nnumeric; i_num++) {
        const char* arr_name = mj_id2name(m, mjOBJ_NUMERIC, i_num);
        // checks for validity during load
        if (!strncmp(arr_name, "VR_", 3) && (
                m->numeric_size[i_num] == 3 || m->numeric_size[i_num] == 7 || m->numeric_size[i_num] == 8)) {
        }
    }
}

void mjvrHMD::applyDefaultVrTransform(mjvScene& scn)
{
    scn.translate[0] = 0.f;     // right-left  
    scn.translate[1] = 1.f;     // up-down
    scn.translate[2] = -1.f;    // back-forward
    scn.rotate[0] = (float)cos(-0.25 * mjPI);
    scn.rotate[1] = (float)sin(-0.25 * mjPI);
    scn.rotate[2] = 0.f;
    scn.rotate[3] = 0.f;
    scn.scale = 1.f;
}

void mjvrHMD::applyVrTransform(mjvScene& scn, mjModel* m)
{
    if (!initialized)
        return;
    applyDefaultVrTransform(scn);
    if (!m)
        return;
    if (vr_transform_i >= 0 && vr_transform_i < n_vr_transforms) {
        int num_adr = m->numeric_adr[vr_transform_inums[vr_transform_i]];
        // translation
        for (size_t i_tr = 0; i_tr < 3; i_tr++)
            scn.translate[i_tr] = (float)m->numeric_data[num_adr + i_tr];
        // rotation - quaternion
        if (m->numeric_size[vr_transform_inums[vr_transform_i]] > 3)
            for (size_t i_tr = 0; i_tr < 4; i_tr++)
                scn.rotate[i_tr] = (float)m->numeric_data[num_adr + i_tr + 3];
        // scale
        if (m->numeric_size[vr_transform_inums[vr_transform_i]] > 7)
            scn.scale = (float)m->numeric_data[num_adr + 7];
    }
}

void mjvrHMD::applyNextVrTransform(mjvScene& scn, mjModel* m)
{
    if (vr_transform_i >= 0) {
        if (++vr_transform_i >= n_vr_transforms)
            vr_transform_i = 0;
        applyVrTransform(scn, m);
    }
}

void mjvrHMD::applyPrevVrTransform(mjvScene& scn, mjModel* m)
{
    if (vr_transform_i >= 0) {
        if (--vr_transform_i < 0)
            vr_transform_i = n_vr_transforms - 1;
        applyVrTransform(scn, m);
    }
}

bool mjvrHMD::isInitialized()
{
    return initialized;
}
