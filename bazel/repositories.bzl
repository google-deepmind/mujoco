"""Pinned source archives shared by MuJoCo build targets."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _dependencies_impl(module_ctx):
    http_archive(
        name = "mujoco_deps_ccd",
        urls = ["https://codeload.github.com/danfis/libccd/tar.gz/7931e764a19ef6b21b443376c699bbc9c6d4fba8"],
        type = "tar.gz",
        strip_prefix = "libccd-7931e764a19ef6b21b443376c699bbc9c6d4fba8",
        sha256 = "479994a86d32e2effcaad64204142000ee6b6b291fd1859ac6710aee8d00a482",
        build_file = Label("//bazel/deps:ccd.BUILD.bazel"),
        patches = [Label("//cmake:ccd-support-emscripten.patch")],
        patch_args = ["-p1"],
    )
    http_archive(
        name = "mujoco_deps_qhull",
        urls = ["https://codeload.github.com/qhull/qhull/tar.gz/d1c2fc0caa5f644f3a0f220290d4a868c68ed4f6"],
        type = "tar.gz",
        strip_prefix = "qhull-d1c2fc0caa5f644f3a0f220290d4a868c68ed4f6",
        sha256 = "421177cc21a7dcb4c1bbc51f65bc16f21d1f157814116bb5c341d694e23d154d",
        build_file = Label("//bazel/deps:qhull.BUILD.bazel"),
        patches = [Label("//cmake:qhull-support-emscripten.patch")],
        patch_args = ["-p1"],
    )
    http_archive(
        name = "mujoco_deps_tinyxml2",
        urls = ["https://codeload.github.com/leethomason/tinyxml2/tar.gz/e6caeae85799003f4ca74ff26ee16a789bc2af48"],
        type = "tar.gz",
        strip_prefix = "tinyxml2-e6caeae85799003f4ca74ff26ee16a789bc2af48",
        sha256 = "ab1a6700074ab4d468e46535545bb33aa4a74d794ab514fac64cc297fc7a2545",
        build_file = Label("//bazel/deps:tinyxml2.BUILD.bazel"),
    )
    http_archive(
        name = "mujoco_deps_tinyobjloader",
        urls = ["https://codeload.github.com/tinyobjloader/tinyobjloader/tar.gz/2945a967c5303b2c8c14174117c45f3302591150"],
        type = "tar.gz",
        strip_prefix = "tinyobjloader-2945a967c5303b2c8c14174117c45f3302591150",
        sha256 = "988cc315f69c9319499fcb20e4feb54e058ea99d1b69d18e9a05984ce884f49c",
        build_file = Label("//bazel/deps:tinyobjloader.BUILD.bazel"),
    )
    http_archive(
        name = "mujoco_deps_lodepng",
        urls = ["https://codeload.github.com/lvandeve/lodepng/tar.gz/17d08dd26cac4d63f43af217ebd70318bfb8189c"],
        type = "tar.gz",
        strip_prefix = "lodepng-17d08dd26cac4d63f43af217ebd70318bfb8189c",
        sha256 = "83d828c5478ffe7bad0e8ed80678ef826206becbd8cf70097b6cc4d29549389b",
        build_file = Label("//bazel/deps:lodepng.BUILD.bazel"),
    )
    http_archive(
        name = "mujoco_deps_miniz",
        urls = ["https://codeload.github.com/richgel999/miniz/tar.gz/d10b03cc73475af673df40f06e5cefd1d5f940d9"],
        type = "tar.gz",
        strip_prefix = "miniz-d10b03cc73475af673df40f06e5cefd1d5f940d9",
        sha256 = "f7a9a89e63300e66c8fa324f8e849e0f348153007a5033f53302632d229e6c80",
        build_file = Label("//bazel/deps:miniz.BUILD.bazel"),
    )
    http_archive(
        name = "mujoco_deps_marchingcubecpp",
        urls = ["https://codeload.github.com/aparis69/MarchingCubeCpp/tar.gz/f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d"],
        type = "tar.gz",
        strip_prefix = "MarchingCubeCpp-f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d",
        sha256 = "227c10b2cffe886454b92a0e9ef9f0c9e8e001d00ea156cc37c8fc43055c9ca6",
        build_file = Label("//bazel/deps:marchingcubecpp.BUILD.bazel"),
    )
    http_archive(
        name = "pybind11",
        urls = ["https://codeload.github.com/pybind/pybind11/tar.gz/97bf890db679505a14dfe547a5e77bb2bd05dc90"],
        type = "tar.gz",
        strip_prefix = "pybind11-97bf890db679505a14dfe547a5e77bb2bd05dc90",
        sha256 = "6368877d3f241f283e010d858af27a6934ac45f7cbb5c68d2518b11bf64a113f",
        build_file = Label("//bazel/deps:pybind11.BUILD.bazel"),
    )
    http_archive(
        name = "eigen",
        urls = ["https://codeload.github.com/eigen-mirror/eigen/tar.gz/ea13a98decd497a8c5588fb5de71b57bcf10d864"],
        type = "tar.gz",
        strip_prefix = "eigen-ea13a98decd497a8c5588fb5de71b57bcf10d864",
        sha256 = "35c6126e246585d9cf6600b65471582c2701aae64b784a6fd19168a90cfc841e",
        build_file = Label("//bazel/deps:eigen.BUILD.bazel"),
    )
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
