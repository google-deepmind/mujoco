load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def fmt_repository(name):
    version = "10.2.1"
    http_archive(
        name = name,
        strip_prefix = "fmt-{}".format(version),
        sha256 = "1250e4cc58bf06ee631567523f48848dc4596133e163f02615c97f78bab6c811",
        url = "https://github.com/fmtlib/fmt/archive/refs/tags/{}.tar.gz".format(version),
        build_file = "@mujoco//tools/workspace/fmt:package.BUILD",
    )
