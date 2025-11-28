const std = @import("std");
pub const glfw = @import("glfw");
pub const c = @cImport({
    @cInclude("mujoco/mujoco.h");
});
