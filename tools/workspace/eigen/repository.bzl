load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def eigen_repository(name = "eigen"):
    commit = "33d0937c6bdf5ec999939fb17f2a553183d14a74"
    http_archive(
        name = name,
        sha256 = "1a7c7d2e2052642acf78b32ef89dc5b6d12dcb8e552b66407c3e67d7f8e1d73e",
        strip_prefix = "eigen-{}".format(commit),
        build_file = "@mujoco//tools/workspace/eigen:package.BUILD.bazel",
        url = "https://gitlab.com/libeigen/eigen/-/archive/{}/eigen-{}.tar.bz2".format(commit, commit),
    )
