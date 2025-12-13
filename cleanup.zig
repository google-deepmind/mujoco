const std = @import("std");
const fs = std.fs;

const UPSTREAM_SUBMODULES = [_][]const u8{
    "third_party/upstream/ccd",
    "third_party/upstream/qhull",
    "third_party/upstream/lodepng",
    "third_party/upstream/tinyxml2",
    "third_party/upstream/tinyobjloader",
    "third_party/upstream/TriangleMeshDistance",
    "third_party/upstream/marchingcubecpp",
    "third_party/upstream/zglfw",
};

/// Clears the contents of a directory recursively, but leaves the directory itself.
/// Preserves .git directories to maintain submodule structure.
fn clearDirContents(path: []const u8) !void {
    var dir = fs.cwd().openDir(path, .{ .iterate = true }) catch |err| {
        // If directory doesn't exist, nothing to clear
        if (err == error.FileNotFound) return;
        return err;
    };
    defer dir.close();

    var it = dir.iterate();
    while (try it.next()) |entry| {
        // Preserve .git directories to maintain submodule structure
        if (std.mem.eql(u8, entry.name, ".git")) continue;

        switch (entry.kind) {
            .directory => try dir.deleteTree(entry.name),
            .file, .sym_link => try dir.deleteFile(entry.name),
            else => {}, // Skip other types (pipes, sockets, etc.)
        }
    }
}

/// Clears the contents of all upstream submodule directories.
fn clearUpstreamContents() !void {
    std.debug.print("Clearing upstream submodule contents...\n", .{});
    
    for (UPSTREAM_SUBMODULES) |submodule_path| {
        std.debug.print("  Clearing {s}...", .{submodule_path});
        clearDirContents(submodule_path) catch |err| {
            std.debug.print(" failed: {}\n", .{err});
            continue;
        };
        std.debug.print(" done\n", .{});
    }
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();
    
    // Get arguments to check for --force flag
    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);
    
    const force = blk: {
        for (args[1..]) |arg| {
            if (std.mem.eql(u8, arg, "--force") or std.mem.eql(u8, arg, "-f")) {
                break :blk true;
            }
        }
        break :blk false;
    };
    
    if (!force) {
        std.debug.print("WARNING: This will delete all contents (except .git) from upstream submodule directories.\n", .{});
        std.debug.print("Run with --force or -f to proceed.\n", .{});
        return;
    }

    try clearUpstreamContents();
    std.debug.print("Upstream cleanup complete.\n", .{});
}
