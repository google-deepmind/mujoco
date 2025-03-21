load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def spdlog_repository(name = "spdlog"):
    version = "1.14.1"
    http_archive(
        name = name,
        sha256 = "1586508029a7d0670dfcb2d97575dcdc242d3868a259742b69f100801ab4e16b",
        strip_prefix = "spdlog-{}".format(version),
        build_file = "@mujoco//tools/workspace/spdlog:package.BUILD.bazel",
        url = "https://github.com/gabime/spdlog/archive/refs/tags/v{}.tar.gz".format(version),
    )
