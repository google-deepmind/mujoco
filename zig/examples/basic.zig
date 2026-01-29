const std = @import("std");
const lib_mujoco = @import("lib_z_mujoco");
const c = lib_mujoco.c;
const glfw = lib_mujoco.glfw;

// MuJoCo data structures
var m: ?*c.mjModel = null;                  // MuJoCo model
var d: ?*c.mjData = null;                   // MuJoCo data
var cam: c.mjvCamera = undefined;           // abstract camera
var opt: c.mjvOption = undefined;           // visualization options
var scn: c.mjvScene = undefined;            // abstract scene
var con: c.mjrContext = undefined;          // custom GPU context

// mouse interaction
var button_left: bool = false;
var button_middle: bool = false;
var button_right: bool = false;
var lastx: f64 = 0;
var lasty: f64 = 0;

// keyboard callback
fn keyboard(window: *glfw.Window, key: glfw.Key, scancode: i32, act: glfw.Action, mods: glfw.Mods) callconv(.c) void {
    _ = window;
    _ = scancode;
    _ = mods;
    // backspace: reset simulation
    if (act == glfw.Action.press and key == glfw.Key.backspace) {
        if (m != null and d != null) {
            c.mj_resetData(m.?, d.?);
            c.mj_forward(m.?, d.?);
        }
    }
}

// mouse button callback
fn mouse_button(window: *glfw.Window, button: glfw.MouseButton, act: glfw.Action, mods: glfw.Mods) callconv(.c) void {
    _ = button;
    _ = act;
    _ = mods;
    // update button state
    button_left = (glfw.getMouseButton(window, glfw.MouseButton.left) == glfw.Action.press);
    button_middle = (glfw.getMouseButton(window, glfw.MouseButton.middle) == glfw.Action.press);
    button_right = (glfw.getMouseButton(window, glfw.MouseButton.right) == glfw.Action.press);

    // update mouse position
    glfw.getCursorPos(window, &lastx, &lasty);
}

// mouse move callback
fn mouse_move(window: *glfw.Window, xpos: f64, ypos: f64) callconv(.c) void {
    // no buttons down: nothing to do
    if (!button_left and !button_middle and !button_right) {
        return;
    }

    // compute mouse displacement, save
    const dx = xpos - lastx;
    const dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    var width: i32 = undefined;
    var height: i32 = undefined;
    glfw.getWindowSize(window, &width, &height);

    // get shift key state
    const mod_shift = (glfw.getKey(window, glfw.Key.left_shift) == glfw.Action.press or
                     glfw.getKey(window, glfw.Key.right_shift) == glfw.Action.press);

    // determine action based on mouse button
    var action: c_int = undefined;
    if (button_right) {
        action = if (mod_shift) c.mjMOUSE_MOVE_H else c.mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = if (mod_shift) c.mjMOUSE_ROTATE_H else c.mjMOUSE_ROTATE_V;
    } else {
        action = c.mjMOUSE_ZOOM;
    }

    // move camera
    if (m != null and d != null) {
        c.mjv_moveCamera(m.?, @as(c_int, action), dx / @as(f64, @floatFromInt(height)), dy / @as(f64, @floatFromInt(height)), &scn, &cam);
    }
}

// scroll callback
fn scroll(window: *glfw.Window, xoffset: f64, yoffset: f64) callconv(.c) void {
    _ = window;
    _ = xoffset;
    // emulate vertical mouse motion = 5% of window height
    if (m != null and d != null) {
        c.mjv_moveCamera(m.?, c.mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
    }
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer {
        const deinit_status = gpa.deinit();
        if (deinit_status == .leak) {
            std.log.err("Memory leak detected!", .{});
        }
    }

    // Check command-line arguments
    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len != 2) {
        std.debug.print("USAGE: basic modelfile\n", .{});
        return error.InvalidArguments;
    }

    const xml_path = args[1];
    const c_xml_path = try allocator.dupe(u8, xml_path);
    defer allocator.free(c_xml_path);

    var error_buffer: [1000]u8 = undefined;
    const error_sz_val: i32 = @intCast(error_buffer.len);

    // Load and compile model
    if (std.mem.endsWith(u8, xml_path, ".mjb")) {
        m = c.mj_loadModel(c_xml_path.ptr, null);
    } else {
        m = c.mj_loadXML(c_xml_path.ptr, null, &error_buffer[0], error_sz_val);
    }

    if (m == null) {
        std.log.err("Failed to load XML model: {s}", .{error_buffer});
        return error.ModelLoadFailed;
    }
    defer c.mj_deleteModel(m.?);

    // Make data
    d = c.mj_makeData(m.?);
    if (d == null) {
        return error.DataMakeFailed;
    }
    defer c.mj_deleteData(d.?);

    // Init GLFW
    try glfw.init();
    defer glfw.terminate();

    // Create window, make OpenGL context current, request v-sync
    const window_width: i32 = 1200;
    const window_height: i32 = 900;
    const window_title = "MuJoCo Basic Example (Zig)";
    const window = glfw.createWindow(window_width, window_height, window_title, null) catch |err| {
        std.log.err("Could not create GLFW window: {}", .{err});
        return error.GLFWWindowCreationFailed;
    };
    defer glfw.destroyWindow(window);

    glfw.makeContextCurrent(window);
    glfw.swapInterval(1);

    // Initialize visualization data structures
    c.mjv_defaultCamera(&cam);
    c.mjv_defaultOption(&opt);
    c.mjv_defaultScene(&scn);
    c.mjr_defaultContext(&con);

    // Create scene and context
    c.mjv_makeScene(m.?, &scn, 2000);
    defer c.mjv_freeScene(&scn);
    c.mjr_makeContext(m.?, &con, c.mjFONTSCALE_150);
    defer c.mjr_freeContext(&con);

    // Install GLFW mouse and keyboard callbacks
    _ = glfw.setKeyCallback(window, keyboard);
    _ = glfw.setCursorPosCallback(window, mouse_move);
    _ = glfw.setMouseButtonCallback(window, mouse_button);
    _ = glfw.setScrollCallback(window, scroll);

    // Run main loop, target real-time simulation and 60 fps rendering
    while (!glfw.windowShouldClose(window)) {
        // Advance interactive simulation for 1/60 sec
        const simstart = d.?.time;
        while (d.?.time - simstart < 1.0 / 60.0) {
            c.mj_step(m.?, d.?);
        }

        // Get framebuffer viewport
        var viewport: c.mjrRect = undefined;
        var fb_width: i32 = undefined;
        var fb_height: i32 = undefined;
        glfw.getFramebufferSize(window, &fb_width, &fb_height);
        viewport.width = fb_width;
        viewport.height = fb_height;
        viewport.left = 0;
        viewport.bottom = 0;

        // Update scene and render
        c.mjv_updateScene(m.?, d.?, &opt, null, &cam, c.mjCAT_ALL, &scn);
        c.mjr_render(viewport, &scn, &con);

        // Swap OpenGL buffers (blocking call due to v-sync)
        glfw.swapBuffers(window);

        // Process pending GUI events, call GLFW callbacks
        glfw.pollEvents();
    }

    return error.Success;
}

const InvalidArguments = error.InvalidArguments;
const ModelLoadFailed = error.ModelLoadFailed;
const DataMakeFailed = error.DataMakeFailed;
const GLFWInitFailed = error.GLFWInitFailed;
const GLFWWindowCreationFailed = error.GLFWWindowCreationFailed;
const Success = error.Success;
