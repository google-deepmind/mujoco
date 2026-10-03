"""Native Python extension and package file rules."""

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

def _package_files_impl(ctx):
    outputs = []
    for source in ctx.files.srcs:
        relative = source.basename
        if ctx.attr.preserve_paths:
            relative = source.short_path
            if relative.startswith("../"):
                relative = relative.split("/", 2)[2]
            if ctx.attr.strip_prefix:
                prefix = ctx.attr.strip_prefix + "/"
                if not relative.startswith(prefix):
                    fail("%s is outside %s" % (relative, prefix))
                relative = relative.removeprefix(prefix)
        output = ctx.actions.declare_file(ctx.attr.directory + "/" + relative)
        ctx.actions.symlink(output = output, target_file = source)
        outputs.append(output)
    return [DefaultInfo(files = depset(outputs), runfiles = ctx.runfiles(files = outputs))]

package_files = rule(
    implementation = _package_files_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "directory": attr.string(mandatory = True),
        "preserve_paths": attr.bool(default = False),
        "strip_prefix": attr.string(),
    },
)

def _extension_file_impl(ctx):
    output = ctx.actions.declare_file(ctx.attr.out)
    ctx.actions.symlink(output = output, target_file = ctx.file.src)
    return [DefaultInfo(files = depset([output]), runfiles = ctx.runfiles(files = [output]))]

extension_file = rule(
    implementation = _extension_file_impl,
    attrs = {
        "src": attr.label(allow_single_file = True, mandatory = True),
        "out": attr.string(mandatory = True),
    },
)

def _wheel_script_impl(ctx):
    script = ctx.actions.declare_file(ctx.attr.out)
    ctx.actions.expand_template(
        template = ctx.file.src,
        output = script,
        substitutions = {"#!/usr/bin/env python": "#!python"},
        is_executable = True,
    )
    return [DefaultInfo(files = depset([script]))]

wheel_script = rule(
    implementation = _wheel_script_impl,
    attrs = {
        "src": attr.label(allow_single_file = True, mandatory = True),
        "out": attr.string(mandatory = True),
    },
)

def mujoco_extension(name, srcs, deps = [], module_name = None, package = "mujoco", cxx_standard = "c++17", linkopts = [], tags = []):
    """Build an extension against the shared MuJoCo library.

    Args:
        name: Target name for the packaged extension.
        srcs: Extension source files.
        deps: Libraries required by the extension.
        module_name: Import name, defaulting to `name`.
        package: Destination directory inside the Python package.
        cxx_standard: C++ language standard for non-Windows compilers.
        linkopts: Additional linker options.
        tags: Tags applied to compilation and packaging targets.
    """
    module_name = module_name or name
    library_path = "/".join([".."] * (len(package.split("/")) - 1)) or "."
    for suffix, compatible in [
        ("so", select({
            "@platforms//os:windows": ["@platforms//:incompatible"],
            "//conditions:default": [],
        })),
        ("pyd", select({
            "@platforms//os:windows": [],
            "//conditions:default": ["@platforms//:incompatible"],
        })),
    ]:
        cc_binary(
            name = name + "_native." + suffix,
            srcs = srcs,
            deps = [":binding_headers", "//:mujoco_shared"] + deps + select({
                "@platforms//os:windows": ["@rules_python//python/cc:current_py_cc_libs"],
                "//conditions:default": [],
            }),
            copts = select({
                "@platforms//os:windows": ["/std:c++20"],
                "//conditions:default": ["-std=" + cxx_standard, "-fvisibility=hidden"],
            }),
            local_defines = ["EIGEN_MPL2_ONLY"],
            linkopts = select({
                "@platforms//os:macos": ["-Wl,-undefined,dynamic_lookup", "-Wl,-rpath,@loader_path/" + library_path],
                "@platforms//os:windows": ["/EXPORT:PyInit_" + module_name],
                "//conditions:default": ["-Wl,-rpath,$$ORIGIN/" + library_path],
            }) + linkopts,
            features = ["-windows_export_all_symbols"],
            target_compatible_with = compatible,
            linkshared = True,
            linkstatic = True,
            tags = tags,
        )
    extension_file(
        name = name,
        src = select({
            "@platforms//os:windows": name + "_native.pyd",
            "//conditions:default": name + "_native.so",
        }),
        out = select({
            "@platforms//os:windows": package + "/" + module_name + ".pyd",
            "//conditions:default": package + "/" + module_name + ".so",
        }),
        tags = tags,
    )
