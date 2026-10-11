"""
Shared configurations for MuJoCo.
"""

load("@rules_cc//cc:cc_binary.bzl", "cc_binary")

# Common tags for MuJoCo tests.
mujoco_test_tags = ["mujoco"]
MJ_ROOT = Label("//:BUILD.bazel").workspace_root or "."

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
