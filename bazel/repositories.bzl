"""CMake dependency pins without compatible BCR replacements."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _dependencies_impl(module_ctx):
    # BCR `ccd@2.1.0.bcr.2` omits `CCD_STATIC_DEFINE`, required by the Windows static build.
    # https://bcr.bazel.build/modules/ccd/metadata.json
    http_archive(
        name = "mujoco_deps_ccd",
        urls = ["https://codeload.github.com/danfis/libccd/tar.gz/7931e764a19ef6b21b443376c699bbc9c6d4fba8"],
        type = "tar.gz",
        strip_prefix = "libccd-7931e764a19ef6b21b443376c699bbc9c6d4fba8",
        sha256 = "479994a86d32e2effcaad64204142000ee6b6b291fd1859ac6710aee8d00a482",
        build_file = Label("//bazel/deps:ccd.BUILD.bazel"),
    )

    # CMake pins `v8.1-alpha6`; BCR `qhull@8.0.2.bcr.2` uses `v8.0.2`.
    # https://bcr.bazel.build/modules/qhull/metadata.json
    http_archive(
        name = "mujoco_deps_qhull",
        urls = ["https://codeload.github.com/qhull/qhull/tar.gz/d1c2fc0caa5f644f3a0f220290d4a868c68ed4f6"],
        type = "tar.gz",
        strip_prefix = "qhull-d1c2fc0caa5f644f3a0f220290d4a868c68ed4f6",
        sha256 = "421177cc21a7dcb4c1bbc51f65bc16f21d1f157814116bb5c341d694e23d154d",
        build_file = Label("//bazel/deps:qhull.BUILD.bazel"),
    )

    # CMake pins a revision after `11.0.0`, the BCR `tinyxml2` release.
    # https://bcr.bazel.build/modules/tinyxml2/metadata.json
    http_archive(
        name = "mujoco_deps_tinyxml2",
        urls = ["https://codeload.github.com/leethomason/tinyxml2/tar.gz/e6caeae85799003f4ca74ff26ee16a789bc2af48"],
        type = "tar.gz",
        strip_prefix = "tinyxml2-e6caeae85799003f4ca74ff26ee16a789bc2af48",
        sha256 = "ab1a6700074ab4d468e46535545bb33aa4a74d794ab514fac64cc297fc7a2545",
        build_file = Label("//bazel/deps:tinyxml2.BUILD.bazel"),
    )

    # CMake pins `v2.0.0rc13`; BCR `tinyobjloader@2.0.0-rc1.bcr.2` uses `v2.0-rc1`.
    # https://bcr.bazel.build/modules/tinyobjloader/metadata.json
    http_archive(
        name = "mujoco_deps_tinyobjloader",
        urls = ["https://codeload.github.com/tinyobjloader/tinyobjloader/tar.gz/2945a967c5303b2c8c14174117c45f3302591150"],
        type = "tar.gz",
        strip_prefix = "tinyobjloader-2945a967c5303b2c8c14174117c45f3302591150",
        sha256 = "988cc315f69c9319499fcb20e4feb54e058ea99d1b69d18e9a05984ce884f49c",
        build_file = Label("//bazel/deps:tinyobjloader.BUILD.bazel"),
    )

    # CMake pins `v3.1.0`; BCR `pybind11_bazel@3.0.1` fetches `v3.0.1`.
    # https://bcr.bazel.build/modules/pybind11_bazel/metadata.json
    http_archive(
        name = "pybind11",
        urls = ["https://codeload.github.com/pybind/pybind11/tar.gz/97bf890db679505a14dfe547a5e77bb2bd05dc90"],
        type = "tar.gz",
        strip_prefix = "pybind11-97bf890db679505a14dfe547a5e77bb2bd05dc90",
        sha256 = "6368877d3f241f283e010d858af27a6934ac45f7cbb5c68d2518b11bf64a113f",
        build_file = Label("//bazel/deps:pybind11.BUILD.bazel"),
    )

    # CMake pins an unreleased revision absent from BCR `eigen@5.0.1.bcr.2`.
    # https://bcr.bazel.build/modules/eigen/metadata.json
    http_archive(
        name = "eigen",
        urls = ["https://codeload.github.com/eigen-mirror/eigen/tar.gz/ea13a98decd497a8c5588fb5de71b57bcf10d864"],
        type = "tar.gz",
        strip_prefix = "eigen-ea13a98decd497a8c5588fb5de71b57bcf10d864",
        sha256 = "35c6126e246585d9cf6600b65471582c2701aae64b784a6fd19168a90cfc841e",
        build_file = Label("//bazel/deps:eigen.BUILD.bazel"),
    )

    # CMake pins an unreleased revision absent from BCR `google_benchmark@1.9.5`.
    # https://bcr.bazel.build/modules/google_benchmark/metadata.json
    http_archive(
        name = "google_benchmark",
        urls = ["https://codeload.github.com/google/benchmark/tar.gz/834a61fc65e8b7885fcf177f1230ae4b897118fa"],
        type = "tar.gz",
        strip_prefix = "benchmark-834a61fc65e8b7885fcf177f1230ae4b897118fa",
        sha256 = "3b40cf7e4884bb1502669254b8e3dfd99035ace61a0dccdf41837e1777139c66",
        build_file = Label("//bazel/deps:google_benchmark.BUILD.bazel"),
    )
    return module_ctx.extension_metadata(reproducible = True)

mujoco_dependencies = module_extension(implementation = _dependencies_impl)
