#include "progui_globals.h"
#include <cstring>
#include <iostream>

void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (mjui_event(&ui, &uistate, &con))
        return;

    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        mj_forward(m, d);
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

    if (act == GLFW_PRESS && key == GLFW_KEY_UP)
    {
        moveLastCubeY(+1);
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_DOWN)
    {
        moveLastCubeY(-1);
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_RIGHT)
    {
        moveLastCubeX(+1);
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_LEFT)
    {
        moveLastCubeX(-1);
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_W)
    {
        moveLastCubeZ(+1);
    }

    if (act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        moveLastCubeZ(-1);
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

    int fbw = 0, fbh = 0;
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
    uistate.shift = (mods & GLFW_MOD_SHIFT) ? 1 : 0;
    uistate.control = (mods & GLFW_MOD_CONTROL) ? 1 : 0;
    uistate.alt = (mods & GLFW_MOD_ALT) ? 1 : 0;
    uistate.nrect = 1;

    int uiw = ui.width;
    if (uiw < g_uiWidth)
        uiw = g_uiWidth;

    uistate.rect[0] = {fbw - uiw, 0, uiw, fbh};
    ui.rectid = 0;

    if (mjuiItem *changed = mjui_event(&ui, &uistate, &con))
    {
        if (act == GLFW_PRESS)
        {
            if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Spawn Cube (C)") == 0)
            {
                spawnCube();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Start Physics (P)") == 0)
            {
                SaveChainPrePhysicsState();
                EnablePhysicsForAll();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Reset Chain (R)") == 0)
            {
                ResetChainToSavedState();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Increase Gap (+)") == 0)
            {
                IncreaseGap();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Decrease Gap (-)") == 0)
            {
                DecreaseGap();
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Move Last +Y (Up)") == 0)
            {
                moveLastCubeY(+1);
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Move Last -Y (Down)") == 0)
            {
                moveLastCubeY(-1);
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Move Last +X (Right)") == 0)
            {
                moveLastCubeX(+1);
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Move Last -X (Left)") == 0)
            {
                moveLastCubeX(-1);
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Move Last +Z (W)") == 0)
            {
                moveLastCubeZ(+1);
            }
            else if (changed->type == mjITEM_BUTTON && std::strcmp(changed->name, "Move Last -Z (S)") == 0)
            {
                moveLastCubeZ(-1);
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

    int width = 0, height = 0;
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

    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}
