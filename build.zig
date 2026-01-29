const std = @import("std");

pub fn build(b: *std.Build) !void {
    const optimize = b.standardOptimizeOption(.{});
    const target = b.standardTargetOptions(.{});

    // Create the main MuJoCo library
    const lib = b.addLibrary(.{
        .name = "lib_mujoco",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libcpp = true,
        }),
    });
    
    // zGLFW
    const zglfw_dep = b.dependency("zglfw", .{
        .target = target,
        .optimize = optimize,
        .shared = false,
    });
    lib.linkLibrary(zglfw_dep.artifact("glfw"));

    // libccd (raw C source)
    const libccd_dep = b.dependency("libccd", .{
        .target = target,
        .optimize = optimize,
    });

    // qhull (raw C source)
    const qhull_dep = b.dependency("qhull", .{
        .target = target,
        .optimize = optimize,
    });

    // lodepng (raw C++ source)
    const lodepng_dep = b.dependency("lodepng", .{
        .target = target,
        .optimize = optimize,
    });

    // tinyxml2 (raw C++ source)
    const tinyxml2_dep = b.dependency("tinyxml2", .{
        .target = target,
        .optimize = optimize,
    });

    // tinyobjloader (raw C++ source)
    const tinyobjloader_dep = b.dependency("tinyobjloader", .{
        .target = target,
        .optimize = optimize,
    });

    // marchingcubecpp (header-only)
    const marchingcubecpp_dep = b.dependency("marchingcubecpp", .{
        .target = target,
        .optimize = optimize,
    });

    // trianglemeshdistance (header-only)
    const trianglemeshdistance_dep = b.dependency("trianglemeshdistance", .{
        .target = target,
        .optimize = optimize,
    });

    // === Generate CCD config.h ===
    const ccd_config_h = b.addWriteFile("ccd/config.h", "#define CCD_DOUBLE\n");
    lib.addIncludePath(ccd_config_h.getDirectory());

    // === Add MuJoCo C Source Files ===
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
            "src/engine/engine_sleep.c",
        },
        .flags = &.{
            "-std=c11",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add libccd C source files ===
    // Files are in src/ directory at the dependency root
    lib.root_module.addCSourceFiles(.{
        .root = libccd_dep.path("src"),
        .files = &.{
            "support.c",
            "mpr.c",
            "vec3.c",
            "polytope.c",
            "ccd.c",
        },
        .flags = &.{
            "-std=c11",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add qhull C source files ===
    // Files are in src/libqhull_r/ directory
    lib.root_module.addCSourceFiles(.{
        .root = qhull_dep.path("src/libqhull_r"),
        .files = &.{
            "global_r.c",
            "stat_r.c",
            "geom2_r.c",
            "poly2_r.c",
            "merge_r.c",
            "libqhull_r.c",
            "geom_r.c",
            "poly_r.c",
            "qset_r.c",
            "mem_r.c",
            "random_r.c",
            "usermem_r.c",
            "userprintf_r.c",
            "io_r.c",
            "user_r.c",
            "rboxlib_r.c",
            "userprintf_rbox_r.c",
        },
        .flags = &.{
            "-std=c11",
            "-D_GNU_SOURCE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add MuJoCo C++ Source Files ===
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
            "src/xml/xml_global.cc",
        },
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add third-party C++ files from fetched dependencies ===
    
    // lodepng (lodepng.cpp is at root)
    lib.root_module.addCSourceFiles(.{
        .root = lodepng_dep.path(""),
        .files = &.{"lodepng.cpp"},
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // tinyxml2 (tinyxml2.cpp is at root)
    lib.root_module.addCSourceFiles(.{
        .root = tinyxml2_dep.path(""),
        .files = &.{"tinyxml2.cpp"},
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // tinyobjloader (tiny_obj_loader.cc is at root)
    lib.root_module.addCSourceFiles(.{
        .root = tinyobjloader_dep.path(""),
        .files = &.{"tiny_obj_loader.cc"},
        .flags = &.{
            "-std=c++20",
            "-D_GNU_SOURCE",
            "-DCCD_STATIC_DEFINE",
            "-DMUJOCO_DLL_EXPORTS",
            "-DMC_IMPLEM_ENABLE",
        },
    });

    // === Add Include Paths ===
    
    // Local MuJoCo includes
    lib.addIncludePath(b.path("include"));
    lib.addIncludePath(b.path("src"));

    // libccd includes
    // Headers are in src/ and src/ccd/
    lib.addIncludePath(libccd_dep.path("src"));
    lib.addIncludePath(libccd_dep.path("src/ccd"));

    // qhull includes
    // Headers are in src/ and src/libqhull_r/
    lib.addIncludePath(qhull_dep.path("src"));
    lib.addIncludePath(qhull_dep.path("src/libqhull_r"));

    // lodepng includes
    // lodepng.h is at root
    lib.addIncludePath(lodepng_dep.path(""));

    // tinyxml2 includes
    // tinyxml2.h is at root
    lib.addIncludePath(tinyxml2_dep.path(""));

    // tinyobjloader includes
    // tiny_obj_loader.h is at root
    lib.addIncludePath(tinyobjloader_dep.path(""));

    // marchingcubecpp includes (header-only)
    // MC.h is at root
    lib.addIncludePath(marchingcubecpp_dep.path(""));

    // trianglemeshdistance includes (header-only)
    // Headers are in TriangleMeshDistance/include/tmd/
    // But the source uses: #include <TriangleMeshDistance/include/tmd/TriangleMeshDistance.h>
    // So we need to add the root path
    lib.addIncludePath(trianglemeshdistance_dep.path(""));

    lib.linkLibC();

    b.installArtifact(lib);

    // === Create and export the lib_z_mujoco Zig module ===
    const lib_mujoco_module = b.createModule(.{
        .root_source_file = b.path("zig/mujoco.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "glfw", .module = zglfw_dep.module("root") },
        },
    });

    lib_mujoco_module.addIncludePath(b.path("include"));
    lib_mujoco_module.linkLibrary(lib);

    b.modules.put("lib_z_mujoco", lib_mujoco_module) catch @panic("failed to register lib_z_mujoco module");

    // --- Examples ---
    const examples_dir_path = "zig/examples";
    var examples_dir = std.fs.cwd().openDir(examples_dir_path, .{ .iterate = true }) catch |err| {
        if (err == error.FileNotFound) return;
        return err;
    };
    defer examples_dir.close();

    var dir_iter = examples_dir.iterate();
    while (try dir_iter.next()) |entry| {
        if (entry.kind != std.fs.Dir.Entry.Kind.file or !std.mem.endsWith(u8, entry.name, ".zig")) {
            continue;
        }

        const exe_name = std.fs.path.stem(entry.name);
        const root_source_path = b.path(b.fmt("zig/examples/{s}", .{entry.name}));

        const example_module = b.createModule(.{
            .root_source_file = root_source_path,
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "lib_z_mujoco", .module = lib_mujoco_module },
            },
            .link_libcpp = true,
        });
        example_module.addIncludePath(b.path("include"));

        const exe = b.addExecutable(.{
            .name = exe_name,
            .root_module = example_module,
        });

        b.installArtifact(exe);

        const run_exe_step = b.addRunArtifact(exe);
        const run_step_name = b.fmt("run-{s}", .{exe_name});
        const run_step_desc = b.fmt("Run the {s} example", .{exe_name});
        const run_step = b.step(run_step_name, run_step_desc);
        run_step.dependOn(&run_exe_step.step);
    }

}
