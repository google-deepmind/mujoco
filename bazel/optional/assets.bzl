"""Compile Filament materials and stage renderer assets in build outputs."""

def _material_impl(ctx):
    output = ctx.actions.declare_file("assets/" + ctx.label.name + ".filamat")
    args = ctx.actions.args()
    args.add_all(["--platform=all", "--api=vulkan", "--api=opengl", "--variant-filter", "skinning", "--optimize-size"])
    args.add("--output", output)
    args.add(ctx.file.src)
    ctx.actions.run(
        executable = ctx.file._matc,
        arguments = [args],
        inputs = [ctx.file.src],
        outputs = [output],
        mnemonic = "FilamentMaterial",
    )
    return [DefaultInfo(files = depset([output]))]

filament_material = rule(
    implementation = _material_impl,
    attrs = {
        "src": attr.label(allow_single_file = [".mat"], mandatory = True),
        "_matc": attr.label(default = Label("@mujoco_filament//:matc"), allow_single_file = True, cfg = "exec"),
    },
)

def _stage_assets_impl(ctx):
    outputs = []
    for source in ctx.files.srcs:
        output = ctx.actions.declare_file("assets/" + source.basename)
        ctx.actions.symlink(output = output, target_file = source)
        outputs.append(output)
    return [DefaultInfo(files = depset(outputs), runfiles = ctx.runfiles(files = outputs))]

stage_assets = rule(
    implementation = _stage_assets_impl,
    attrs = {"srcs": attr.label_list(allow_files = True)},
)
