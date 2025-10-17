const std = @import("std");

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    const lib = b.addLibrary(.{
        .name = "lib_mujoco",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libcpp = true,
        }),
    });

    const zglfw_dep = b.dependency("zglfw", .{ .target = target, .optimize = optimize, .shared = false });

    lib.linkLibrary(zglfw_dep.artifact("glfw"));

    const ccd_config_h = b.addWriteFile("ccd/config.h", "#define CCD_DOUBLE\n");
    lib.addIncludePath(ccd_config_h.getDirectory());

    lib.root_module.addCSourceFiles(.{
        .files = &.{ 
            "src/user/user_init.c",
            "src/ui/ui_main.c",
            "src/render/render_util.c",
            "src/render/render_gl3.c",
            "src/render/render_gl2.c",
            "src/render/render_context.c",
            "src/render/glad/glad.c",
            "src/engine/engine_vis_visualize.c",
            "src/engine/engine_init.c",
            "src/engine/engine_vis_interact.c",
            "src/engine/engine_vis_init.c",
            "src/engine/engine_util_spatial.c",
            "src/engine/engine_util_sparse.c",
            "src/engine/engine_util_solve.c",
            "src/engine/engine_util_misc.c",
            "src/engine/engine_util_errmem.c",
            "src/engine/engine_util_container.c",
            "src/engine/engine_util_blas.c",
            "src/engine/engine_support.c",
            "src/engine/engine_solver.c",
            "src/engine/engine_setconst.c",
            "src/engine/engine_sensor.c",
            "src/engine/engine_ray.c",
            "src/engine/engine_print.c",
            "src/engine/engine_passive.c",
            "src/engine/engine_name.c",
            "src/engine/engine_memory.c",
            "src/engine/engine_island.c",
            "src/engine/engine_io.c",
            "src/engine/engine_inverse.c",
            "src/engine/engine_forward.c",
            "src/engine/engine_derivative_fd.c",
            "src/engine/engine_derivative.c",
            "src/engine/engine_core_util.c",
            "src/engine/engine_core_smooth.c",
            "src/engine/engine_core_constraint.c",
            "src/engine/engine_collision_sdf.c",
            "src/engine/engine_collision_primitive.c",
            "src/engine/engine_collision_gjk.c",
            "src/engine/engine_collision_driver.c",
            "src/engine/engine_collision_convex.c",
            "src/engine/engine_collision_box.c",
            "src/engine/engine_callback.c",
            "third_party/ccd/src/support.c",
            "third_party/ccd/src/mpr.c",
            "third_party/ccd/src/vec3.c",
            "third_party/ccd/src/polytope.c",
            "third_party/ccd/src/ccd.c",
            "third_party/qhull/src/libqhull_r/global_r.c",
            "third_party/qhull/src/libqhull_r/stat_r.c",
            "third_party/qhull/src/libqhull_r/geom2_r.c",
            "third_party/qhull/src/libqhull_r/poly2_r.c",
            "third_party/qhull/src/libqhull_r/merge_r.c",
            "third_party/qhull/src/libqhull_r/libqhull_r.c",
            "third_party/qhull/src/libqhull_r/geom_r.c",
            "third_party/qhull/src/libqhull_r/poly_r.c",
            "third_party/qhull/src/libqhull_r/qset_r.c",
            "third_party/qhull/src/libqhull_r/mem_r.c",
            "third_party/qhull/src/libqhull_r/random_r.c",
            "third_party/qhull/src/libqhull_r/usermem_r.c",
            "third_party/qhull/src/libqhull_r/userprintf_r.c",
            "third_party/qhull/src/libqhull_r/io_r.c",
            "third_party/qhull/src/libqhull_r/user_r.c",
            "third_party/qhull/src/libqhull_r/accessors_r.c",
            "third_party/qhull/src/libqhull_r/rboxlib_r.c",
            "third_party/qhull/src/libqhull_r/userprintf_rbox_r.c",
        },
        .flags = &.{ 
            "-std=c11",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
    //        "-DCCD_DOUBLE",
        },
    });

    lib.root_module.addCSourceFiles(.{
        .files = &.{ 
            "src/xml/xml_util.cc",
            "src/xml/xml_urdf.cc",
            "src/xml/xml_numeric_format.cc",
            "src/xml/xml_native_writer.cc",
            "src/xml/xml_native_reader.cc",
            "src/xml/xml_base.cc",
            "src/xml/xml_api.cc",
            "src/xml/xml.cc",
            "src/user/user_vfs.cc",
            "src/user/user_util.cc",
            "src/user/user_resource.cc",
            "src/user/user_objects.cc",
            "src/user/user_model.cc",
            "src/user/user_mesh.cc",
            "src/user/user_flexcomp.cc",
            "src/user/user_composite.cc",
            "src/user/user_cache.cc",
            "src/user/user_api.cc",
            "src/thread/thread_task.cc",
            "src/thread/thread_pool.cc",
            "src/render/glad/loader.cc",
            "src/engine/engine_plugin.cc",
            "src/engine/engine_crossplatform.cc",
            "third_party/lodepng/lodepng.cpp",
            "third_party/tinyxml2/tinyxml2.cpp",
            "third_party/tinyobjloader/tiny_obj_loader.cc",
        },
        .flags = &.{ 
            "-std=c++17",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    lib.addIncludePath(b.path("include"));
    lib.addIncludePath(b.path("src"));
    lib.addIncludePath(b.path("third_party/ccd/src"));
    lib.addIncludePath(b.path("third_party/ccd/src/ccd"));
    lib.addIncludePath(b.path("third_party/qhull/src"));
    lib.addIncludePath(b.path("third_party/qhull/src/libqhull_r"));
    lib.addIncludePath(b.path("third_party/lodepng"));
    lib.addIncludePath(b.path("third_party/tinyxml2"));
    lib.addIncludePath(b.path("third_party/tinyobjloader"));
    lib.addIncludePath(b.path("third_party/marchingcubecpp"));
    lib.addIncludePath(b.path("third_party/TriangleMeshDistance"));

    lib.linkLibC();

    b.installArtifact(lib);

    // --- Create and export the lib_zmujoco Zig module ---
    const zmujoco_module = b.createModule(.{
        .root_source_file = b.path("zig/c.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{ 
            .{ .name = "glfw", .module = zglfw_dep.module("root") },
        },
    });

    zmujoco_module.addIncludePath(b.path("include"));
    zmujoco_module.linkLibrary(lib);

    b.modules.put("lib_zmujoco", zmujoco_module) catch @panic("failed to register lib_zmujoco module");
}