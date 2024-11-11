load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def tinyobjloader_repository(name = "tinyobjloader"):
    commit = "1421a10d6ed9742f5b2c1766d22faa6cfbc56248"
    http_archive(
        name = name,
        sha256 = "e334b2900380efdc19a0ea42e5e966a6a6a04831dd830dd42a80e28ce6d1e9be",
        strip_prefix = "tinyobjloader-{}".format(commit),
        build_file = "@mujoco//tools/workspace/tinyobjloader:package.BUILD.bazel",
        url = "https://github.com/tinyobjloader/tinyobjloader/archive/{}.tar.gz".format(commit),
    )
