load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def libccd_repository(name = "libccd"):
    commit = "7931e764a19ef6b21b443376c699bbc9c6d4fba8"
    http_archive(
        name = name,
        sha256 = "479994a86d32e2effcaad64204142000ee6b6b291fd1859ac6710aee8d00a482",
        strip_prefix = "libccd-{}".format(commit),
        build_file = "@mujoco//tools/workspace/libccd:package.BUILD.bazel",
        url = "https://github.com/danfis/libccd/archive/{}.tar.gz".format(commit),
    )
