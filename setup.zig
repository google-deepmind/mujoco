const std = @import("std");
const fs = std.fs;
const process = std.process;

const Submodule = struct {
    url: []const u8,
    path: []const u8,
};

const Library = struct {
    name: []const u8,
    sources: []const []const u8,
    include_dirs: []const []const u8,
};

const submodules = [_]Submodule{
    .{ .url = "https://github.com/danfis/libccd.git", .path = "third_party/upstream/ccd" },
    .{ .url = "https://github.com/qhull/qhull.git", .path = "third_party/upstream/qhull" },
    .{ .url = "https://github.com/lvandeve/lodepng.git", .path = "third_party/upstream/lodepng" },
    .{ .url = "https://github.com/leethomason/tinyxml2.git", .path = "third_party/upstream/tinyxml2" },
    .{ .url = "https://github.com/tinyobjloader/tinyobjloader.git", .path = "third_party/upstream/tinyobjloader" },
    .{ .url = "https://github.com/InteractiveComputerGraphics/TriangleMeshDistance.git", .path = "third_party/upstream/TriangleMeshDistance" },
    .{ .url = "https://github.com/aparis69/MarchingCubeCpp.git", .path = "third_party/upstream/marchingcubecpp" },
    .{ .url = "https://github.com/zig-gamedev/zglfw.git", .path = "third_party/upstream/zglfw" },
};

const libraries = [_]Library{
    .{
        .name = "ccd",
        .sources = &[_][]const u8{
            "ccd/src/support.c",
            "ccd/src/mpr.c",
            "ccd/src/vec3.c",
            "ccd/src/polytope.c",
            "ccd/src/ccd.c",
        },
        .include_dirs = &[_][]const u8{ "ccd/src", "ccd/src/ccd" },
    },
    .{
        .name = "qhull",
        .sources = &[_][]const u8{
            "qhull/src/libqhull_r/global_r.c",
            "qhull/src/libqhull_r/stat_r.c",
            "qhull/src/libqhull_r/geom2_r.c",
            "qhull/src/libqhull_r/poly2_r.c",
            "qhull/src/libqhull_r/merge_r.c",
            "qhull/src/libqhull_r/libqhull_r.c",
            "qhull/src/libqhull_r/geom_r.c",
            "qhull/src/libqhull_r/poly_r.c",
            "qhull/src/libqhull_r/qset_r.c",
            "qhull/src/libqhull_r/mem_r.c",
            "qhull/src/libqhull_r/random_r.c",
            "qhull/src/libqhull_r/usermem_r.c",
            "qhull/src/libqhull_r/userprintf_r.c",
            "qhull/src/libqhull_r/io_r.c",
            "qhull/src/libqhull_r/user_r.c",
            "qhull/src/libqhull_r/accessors_r.c",
            "qhull/src/libqhull_r/rboxlib_r.c",
            "qhull/src/libqhull_r/userprintf_rbox_r.c",
        },
        .include_dirs = &[_][]const u8{ "qhull/src", "qhull/src/libqhull_r" },
    },
    .{
        .name = "lodepng",
        .sources = &[_][]const u8{"lodepng/lodepng.cpp"},
        .include_dirs = &[_][]const u8{"lodepng"},
    },
    .{
        .name = "tinyxml2",
        .sources = &[_][]const u8{"tinyxml2/tinyxml2.cpp"},
        .include_dirs = &[_][]const u8{"tinyxml2"},
    },
    .{
        .name = "tinyobjloader",
        .sources = &[_][]const u8{"tinyobjloader/tiny_obj_loader.cc"},
        .include_dirs = &[_][]const u8{"tinyobjloader"},
    },
    .{
        .name = "marchingcubecpp",
        .sources = &[_][]const u8{},
        .include_dirs = &[_][]const u8{"marchingcubecpp"},
    },
    .{
        .name = "TriangleMeshDistance",
        .sources = &[_][]const u8{},
        .include_dirs = &[_][]const u8{"TriangleMeshDistance"},
    },
    .{
        .name = "zglfw",
        .sources = &[_][]const u8{},
        .include_dirs = &[_][]const u8{"zglfw"},
    },
};

fn runCommand(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var child = process.Child.init(args, allocator);
    child.stdout_behavior = .Inherit;
    child.stderr_behavior = .Inherit;
    const term = try child.spawnAndWait();
    
    switch (term) {
        .Exited => |code| if (code != 0) return error.CommandFailed,
        else => return error.CommandFailed,
    }
}

fn copyFile(src_path: []const u8, dest_path: []const u8) !void {
    if (fs.path.dirname(dest_path)) |dir_path| {
        try fs.cwd().makePath(dir_path);
    }

    // Use copyFile for simpler, more efficient copying
    try fs.cwd().copyFile(src_path, fs.cwd(), dest_path, .{});
}

fn copyDirRecursive(allocator: std.mem.Allocator, src_path: []const u8, dest_path: []const u8) !void {
    try fs.cwd().makePath(dest_path);

    var dir = try fs.cwd().openDir(src_path, .{ .iterate = true });
    defer dir.close();

    var it = dir.iterate();
    while (try it.next()) |entry| {
        const entry_src_path = try fs.path.join(allocator, &[_][]const u8{ src_path, entry.name });
        defer allocator.free(entry_src_path);

        const entry_dest_path = try fs.path.join(allocator, &[_][]const u8{ dest_path, entry.name });
        defer allocator.free(entry_dest_path);

        switch (entry.kind) {
            .directory => try copyDirRecursive(allocator, entry_src_path, entry_dest_path),
            .file => try copyFile(entry_src_path, entry_dest_path),
            else => {},
        }
    }
}

fn vendorLibrary(allocator: std.mem.Allocator, lib: Library) !void {
    std.debug.print("Vendoring {s}...\n", .{lib.name});

    for (lib.sources) |src_file| {
        const dest_file = try fs.path.join(allocator, &[_][]const u8{ "third_party", src_file });
        defer allocator.free(dest_file);
        const upstream_src = try fs.path.join(allocator, &[_][]const u8{ "third_party", "upstream", src_file });
        defer allocator.free(upstream_src);
        try copyFile(upstream_src, dest_file);
    }

    for (lib.include_dirs) |include_dir| {
        const dest_dir = try fs.path.join(allocator, &[_][]const u8{ "third_party", include_dir });
        defer allocator.free(dest_dir);
        const upstream_dir = try fs.path.join(allocator, &[_][]const u8{ "third_party", "upstream", include_dir });
        defer allocator.free(upstream_dir);
        try copyDirRecursive(allocator, upstream_dir, dest_dir);
    }
}

fn isSubmoduleTracked(allocator: std.mem.Allocator, path: []const u8) !bool {
    const status_args = &[_][]const u8{ "git", "submodule", "status", path };
    var status_child = process.Child.init(status_args, allocator);
    status_child.stdout_behavior = .Ignore;
    status_child.stderr_behavior = .Ignore;
    const status_term = try status_child.spawnAndWait();

    return switch (status_term) {
        .Exited => |code| code == 0,
        else => false,
    };
}

fn addSubmodule(allocator: std.mem.Allocator, url: []const u8, path: []const u8) !void {
    if (try isSubmoduleTracked(allocator, path)) {
        std.debug.print("Submodule {s} already tracked, skipping add.\n", .{path});
        return;
    }

    std.debug.print("Adding submodule: {s}\n", .{path});
    try runCommand(allocator, &[_][]const u8{ "git", "submodule", "add", url, path });
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    std.debug.print("Adding submodules...\n", .{});
    for (submodules) |submodule| {
        try addSubmodule(allocator, submodule.url, submodule.path);
    }

    std.debug.print("Initializing and updating submodules...\n", .{});
    try fs.cwd().makePath("third_party/upstream");
    try runCommand(allocator, &[_][]const u8{ "git", "submodule", "update", "--init", "--recursive", "--force" });

    std.debug.print("Submodule status:\n", .{});
    try runCommand(allocator, &[_][]const u8{ "git", "submodule", "status" });

    std.debug.print("Vendoring dependencies...\n", .{});
    for (libraries) |lib| {
        try vendorLibrary(allocator, lib);
    }

    std.debug.print("Setup complete.\n", .{});
}
