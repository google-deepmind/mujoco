"""
Shared configurations for MuJoCo.
"""

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

# Common tags for MuJoCo tests.
mujoco_test_tags = ["mujoco"]
MJ_ROOT = Label("//:BUILD.bazel").workspace_root or "."

def _cc_headers_impl(ctx):
    package = ctx.label.package
    headers = depset(transitive = [
        dep[CcInfo].compilation_context.headers
        for dep in ctx.attr.deps
    ])
    files = [
        header
        for header in headers.to_list()
        if header.owner.workspace_name == ctx.label.workspace_name and
           (not package or header.owner.package == package or
            header.owner.package.startswith(package + "/"))
    ]
    return [DefaultInfo(
        files = depset(files),
        runfiles = ctx.runfiles(files = files),
    )]

# The headers of C++ libraries and everything they depend on, as files, e.g. to
# find what they declare with MJAPI. Only headers from this package and its
# subpackages are included, not those of other dependencies.
cc_headers = rule(
    implementation = _cc_headers_impl,
    attrs = {"deps": attr.label_list(providers = [CcInfo])},
)

def mj_models_filegroup(name, extra_srcs = (), **kwargs):
    """Returns a list of test model files in the current package."""
    return native.filegroup(
        name = name,
        srcs = native.glob(
            [
                "**/*.xml",
                "**/*.obj",
                "**/*.msh",
                "**/*.skn",
                "**/*.stl",
                "**/*.png",
                "**/*.mtl",
                "**/*.ktx",
                "**/*.usda",
            ],
            allow_empty = True,
        ) + list(extra_srcs),
        testonly = True,
        visibility = [
            "//test:__subpackages__",
        ],
        **kwargs
    )

def mujoco_ui_binary(name, **kwargs):
    """Creates a cc_binary target that can run locally and through CRD, with GPU acceleration.

    Args:
      name: The name of the binary target.
      **kwargs: Additional keyword arguments passed to the underlying cc_binary.
    """

    cc_binary(
        name = name,
        **kwargs
    )
