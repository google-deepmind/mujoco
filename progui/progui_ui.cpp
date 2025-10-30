#include "progui_globals.h"
#include <cstdio>
#include <cstring>
#include <iostream>

// helper to append a character to filename buffer
static void AppendCharToFilename(char c)
{
    size_t len = std::strlen(g_ioFilename);
    if (len +1 < sizeof(g_ioFilename))
    {
        g_ioFilename[len] = c;
        g_ioFilename[len +1] = '\0';
    }
}

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (mjui_event(&ui, &uistate, &con))
        return;

    // simple filename capture mode for Save/Load
    // g_fileIoMode:0 none,1 save,2 load
    extern int g_fileIoMode;
    if (g_fileIoMode !=0 && act == GLFW_PRESS)
    {
        if (key == GLFW_KEY_ESCAPE)
        {
            g_fileIoMode =0; // cancel
            return;
        }
        if (key == GLFW_KEY_ENTER)
        {
            if (g_fileIoMode ==1)
            {
                SaveChainToFile(g_ioFilename);
            }
            else if (g_fileIoMode ==2)
            {
                LoadChainFromFile(g_ioFilename);
            }
            g_fileIoMode =0;
            return;
        }
        if (key == GLFW_KEY_BACKSPACE)
        {
            size_t len = std::strlen(g_ioFilename);
            if (len >0)
                g_ioFilename[len -1] = '\0';
            return;
        }
        // allow alnum and a few filename characters
        if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
        {
            char c = (char)('a' + (key - GLFW_KEY_A));
            if (mods & GLFW_MOD_SHIFT) c = (char)std::toupper(c);
            AppendCharToFilename(c);
            return;
        }
        if (key >= GLFW_KEY_0 && key <= GLFW_KEY_9)
        {
            char c = (char)('0' + (key - GLFW_KEY_0));
            AppendCharToFilename(c);
            return;
        }
        if (key == GLFW_KEY_PERIOD) { AppendCharToFilename('.'); return; }
        if (key == GLFW_KEY_MINUS) { AppendCharToFilename('-'); return; }
#ifdef _WIN32
        if (key == GLFW_KEY_BACKSLASH) { AppendCharToFilename('\\'); return; }
#endif
        if (key == GLFW_KEY_SLASH) { AppendCharToFilename('/'); return; }
        if (key == GLFW_KEY_SPACE) { AppendCharToFilename(' '); return; }
        // ignore others while in capture mode
        return;
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }

    // shortcuts for save/load
    if ((mods & GLFW_MOD_CONTROL) && act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        extern int g_fileIoMode; g_fileIoMode =1; // start capture for save
        return;
    }
    if ((mods & GLFW_MOD_CONTROL) && act == GLFW_PRESS && key == GLFW_KEY_L)
    {
        extern int g_fileIoMode; g_fileIoMode =2; // start capture for load
        return;
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_P)
    {
        // save chain state before enabling physics
        SaveChainPrePhysicsState();
        EnablePhysicsForAll();
        return;
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_R)
    {
        ResetChainToSavedState();
        return;
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_EQUAL) // '+' usually shares '=' key without shift info here
    {
        IncreaseGap();
        return;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_MINUS)
    {
        DecreaseGap();
        return;
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_DELETE)
    {
        deleteLastCube();
        return;
    }

    if (act == GLFW_PRESS)
    {
        if (key == GLFW_KEY_1)
        {
            g_spawnFace = FACE_POSX;
            UpdateMarkerOnLastBody();
            return;
        }

        if (key == GLFW_KEY_2)
        {
            g_spawnFace = FACE_NEGX;
            UpdateMarkerOnLastBody();
            return;
        }

        if (key == GLFW_KEY_3)
        {
            g_spawnFace = FACE_POSY;
            UpdateMarkerOnLastBody();
            return;
        }

        if (key == GLFW_KEY_4)
        {
            g_spawnFace = FACE_NEGY;
            UpdateMarkerOnLastBody();
            return;
        }

        if (key == GLFW_KEY_5)
        {
            g_spawnFace = FACE_POSZ;
            UpdateMarkerOnLastBody();
            return;
        }

        if (key == GLFW_KEY_6)
        {
            g_spawnFace = FACE_NEGZ;
            UpdateMarkerOnLastBody();
            return;
        }
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_C)
    {
        spawnCube();
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_H)
    {
        DebugPrintProbeContacts();
    }
}

void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);

    int fbw =0, fbh =0;
    glfwGetFramebufferSize(window, &fbw, &fbh);

    uistate.type = (act == GLFW_PRESS) ? mjEVENT_PRESS : mjEVENT_RELEASE;
    uistate.left = button_left;
    uistate.right = button_right;
    uistate.middle = button_middle;
    uistate.button = (button == GLFW_MOUSE_BUTTON_LEFT)     ? mjBUTTON_LEFT
                     : (button == GLFW_MOUSE_BUTTON_RIGHT)  ? mjBUTTON_RIGHT
                     : (button == GLFW_MOUSE_BUTTON_MIDDLE) ? mjBUTTON_MIDDLE
                                                            : mjBUTTON_NONE;
    uistate.x = lastx;
    uistate.y = fbh - lasty;
    uistate.shift = (mods & GLFW_MOD_SHIFT) ?1 :0;
    uistate.control = (mods & GLFW_MOD_CONTROL) ?1 :0;
    uistate.alt = (mods & GLFW_MOD_ALT) ?1 :0;
    uistate.nrect =1;

    int uiw = ui.width;
    if (uiw < g_uiWidth)
        uiw = g_uiWidth;

    uistate.rect[0] = {fbw - uiw,0, uiw, fbh};
    ui.rectid =0;

    if (mjuiItem *changed = mjui_event(&ui, &uistate, &con))
    {
        if (act == GLFW_PRESS)
        {
            if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnSpawnCube) ==0)
            {
                spawnCube();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnStartPhysics) ==0)
            {
                SaveChainPrePhysicsState();
                EnablePhysicsForAll();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnResetChain) ==0)
            {
                ResetChainToSavedState();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnIncreaseGap) ==0)
            {
                IncreaseGap();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnDecreaseGap) ==0)
            {
                DecreaseGap();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnSaveChain) ==0)
            {
                extern int g_fileIoMode; g_fileIoMode =1;
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnLoadChain) ==0)
            {
                extern int g_fileIoMode; g_fileIoMode =2;
            }
            // new direct file actions using the edit field
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnSaveToFile) ==0)
            {
                SaveChainToFile(g_ioFilename);
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, kBtnLoadFromFile) ==0)
            {
                LoadChainFromFile(g_ioFilename);
            }
        }
        return;
    }
}

void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    if (mjui_event(&ui, &uistate, &con))
        return;

    if (!button_left && !button_middle && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width =0, height =0;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    if (mjui_event(&ui, &uistate, &con))
        return;

    mjv_moveCamera(m, mjMOUSE_ZOOM,0, -0.05 * yoffset, &scn, &cam);
}
