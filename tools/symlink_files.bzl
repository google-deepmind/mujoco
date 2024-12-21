load("@aspect_bazel_lib//lib:paths.bzl", "to_repository_relative_path")

_symlink_files_attrs = {
    "srcs": attr.label_list(
        allow_files = True,
        doc = "A list of input files for which symbolic links will be created.",
    ),
    "strip_prefix": attr.string(
        doc = """This pecifies a prefix that, if matched at the beginning of each file’s path, will
            be stripped from the file path.""",
    ),
}

def _symlink_files_impl(ctx):
    outputs = []
    for src in ctx.files.srcs:
        workspace_relpath = to_repository_relative_path(src)
        if workspace_relpath.startswith(ctx.attr.strip_prefix):
            workspace_relpath = workspace_relpath[len(ctx.attr.strip_prefix):]
        link = ctx.actions.declare_file(workspace_relpath)
        ctx.actions.symlink(output = link, target_file = src)
        outputs.append(link)

    return [
        DefaultInfo(
            files = depset(outputs),
            runfiles = ctx.runfiles(outputs),
        ),
    ]

symlink_files = rule(
    doc = "Symlink files to the output directory.",
    implementation = _symlink_files_impl,
    attrs = _symlink_files_attrs,
    provides = [DefaultInfo],
)
