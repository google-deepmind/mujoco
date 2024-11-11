load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def googletest_repository(name = "googletest"):
    version = "1.15.2"
    http_archive(
        name = name,
        sha256 = "7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926",
        strip_prefix = "googletest-{}".format(version),
        url = "https://github.com/google/googletest/releases/download/v{}/googletest-{}.tar.gz".format(version, version),
    )
