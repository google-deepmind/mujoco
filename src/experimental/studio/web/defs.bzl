"""Stage the Studio browser client with its runtime assets."""

def _web_dist_impl(ctx):
    outputs = []
    entries = [(file, file.basename) for file in ctx.files.client]
    entries += [(ctx.file.index, "index.html"), (ctx.file.icon, "favicon.ico")]
    entries += [(file, "assets/" + file.basename) for file in ctx.files.assets]
    for source, relative_path in entries:
        output = ctx.actions.declare_file(ctx.attr.directory + "/" + relative_path)
        ctx.actions.symlink(output = output, target_file = source)
        outputs.append(output)
    return [DefaultInfo(files = depset(outputs), runfiles = ctx.runfiles(files = outputs))]

web_dist = rule(
    implementation = _web_dist_impl,
    attrs = {
        "assets": attr.label_list(allow_files = True),
        "client": attr.label(allow_files = True, mandatory = True),
        "directory": attr.string(mandatory = True),
        "icon": attr.label(allow_single_file = True, mandatory = True),
        "index": attr.label(allow_single_file = True, mandatory = True),
    },
)

def _package_web_dist_impl(ctx):
    outputs = []
    for source in ctx.files.src:
        parts = source.short_path.split("src/experimental/studio/web/dist/", 1)
        if len(parts) != 2:
            fail("Expected a Studio browser distribution file: " + source.short_path)
        relative_path = parts[1]
        output = ctx.actions.declare_file("mujoco/experimental/studio/web/dist/" + relative_path)
        ctx.actions.symlink(output = output, target_file = source)
        outputs.append(output)
    return [DefaultInfo(files = depset(outputs), runfiles = ctx.runfiles(files = outputs))]

package_web_dist = rule(
    implementation = _package_web_dist_impl,
    attrs = {
        "src": attr.label(default = "//src/experimental/studio/web:dist"),
    },
)
