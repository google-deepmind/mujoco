#include "progui_globals.h"
#include <cstdio>
#include <cstring>
#include <iostream>

int main(int argc, const char **argv)
{
    // check command-line arguments
    std::string modelfile = "cube_3x3x3.xml";
    if (argc == 2)
    {
        modelfile = argv[1];
        std::cout << "printing argv[1]: " << argv[1] << std::endl;
    }
    else
    {
        std::printf(" USAGE: basic cube file\n");
    }

    if (!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    GLFWwindow *window = glfwCreateWindow(1200, 900, "ProGUI - Cube Spawner", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    char error[1000] = "Could not load model";
    if (modelfile.size() > 4 && !std::strcmp(modelfile.c_str() + modelfile.size() - 4, ".mjb"))
    {
        std::cout << "loading mjb file (runtime editing disabled)" << std::endl;
        m = mj_loadModel(modelfile.c_str(), 0);
        if (!m)
            mju_error("Could not load mjb model");
        d = mj_makeData(m);
    }
    else
    {
        std::cout << "loading xml file (runtime editing enabled)" << std::endl;
        spec = mj_parseXML(modelfile.c_str(), nullptr, error, sizeof(error));
        if (!spec)
            mju_error("Load spec error: %s", error);
        m = mj_compile(spec, nullptr);
        if (!m)
            mju_error("Compile spec error: %s", mjs_getError(spec));
        d = mj_makeData(m);
    }

    g_lastBody = nullptr;
    g_chain.clear();

    g_savedGravity[0] = m->opt.gravity[0];
    g_savedGravity[1] = m->opt.gravity[1];
    g_savedGravity[2] = m->opt.gravity[2];

    m->opt.gravity[0] = 0.0;
    m->opt.gravity[1] = 0.0;
    m->opt.gravity[2] = 0.0;
    mj_forward(m, d);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    std::memset(&uistate, 0, sizeof(mjuiState));
    std::memset(&ui, 0, sizeof(mjUI));
    ui.spacing = mjui_themeSpacing(1);
    ui.color = mjui_themeColor(1);
    ui.predicate = nullptr;
    ui.rectid = 1;
    ui.auxid = 0;
    ui.radiocol = 0;
    ui.color.button[0] = 0.15f;
    ui.color.button[1] = 0.55f;
    ui.color.button[2] = 0.95f;

    mjuiDef defControl[] = {
        {mjITEM_SECTION, "Controls", 1, nullptr, ""},           {mjITEM_BUTTON, "Spawn Cube (C)", 2, nullptr, ""},
        {mjITEM_BUTTON, "Start Physics (P)", 2, nullptr, ""},   {mjITEM_BUTTON, "Reset Chain (R)", 2, nullptr, ""},
        {mjITEM_BUTTON, "Increase Gap (+)", 2, nullptr, ""},   {mjITEM_BUTTON, "Decrease Gap (-)", 2, nullptr, ""},
        {mjITEM_SEPARATOR, "", 2, nullptr, ""},
        {mjITEM_BUTTON, "Move Last +Y (Up)", 2, nullptr, ""},
        {mjITEM_BUTTON, "Move Last -Y (Down)", 2, nullptr, ""}, {mjITEM_BUTTON, "Move Last +X (Right)", 2, nullptr, ""},
        {mjITEM_BUTTON, "Move Last -X (Left)", 2, nullptr, ""}, {mjITEM_BUTTON, "Move Last +Z (W)", 2, nullptr, ""},
        {mjITEM_BUTTON, "Move Last -Z (S)", 2, nullptr, ""},    {mjITEM_END}};

    mjui_add(&ui, defControl);
    mjui_resize(&ui, &con);
    mjr_addAux(ui.auxid, ui.width, ui.maxheight, ui.spacing.samples, &con);

    // spawn a root cube on startup so the user doesn't have to
    spawnCube();

    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;
        if (g_physicsEnabled)
        {
            while (d->time - simstart < 1.0 / 60.0)
            {
                mj_step(m, d);
            }
        }
        else
        {
            mj_kinematics(m, d);
            mj_collision(m, d);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        ui.rectid = 0;
        uistate.nrect = 1;
        int uiw = ui.width;
        if (uiw < g_uiWidth)
            uiw = g_uiWidth;
        uistate.rect[0] = {viewport.width - uiw, 0, uiw, viewport.height};

        mjui_update(-1, -1, &ui, &uistate, &con);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // draw gap ratio in the upper-left corner as simple text overlay
        char gapText[64];
        std::snprintf(gapText, sizeof(gapText), "gap ratio: %.3f", g_gapRatio);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, gapText, NULL, &con);
        mjui_render(&ui, &uistate, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    if (spec)
    {
        mj_deleteSpec(spec);
        spec = nullptr;
    }

#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return EXIT_SUCCESS;
}
