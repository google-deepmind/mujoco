load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def pybind11_repository(name = "pybind11"):
    version = "2.13.5"
    http_archive(
        name = name,
        build_file = "@mujoco//tools/workspace/pybind11:package.BUILD.bazel",
        sha256 = "b1e209c42b3a9ed74da3e0b25a4f4cd478d89d5efbb48f04b277df427faf6252",
        strip_prefix = "pybind11-{}".format(version),
        url = "https://github.com/pybind/pybind11/archive/refs/tags/v{}.tar.gz".format(version),
    )
