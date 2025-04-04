load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def SdfLib_repository(name = "SdfLib"):
    commit = "1927bee6bb8225258a39c8cbf14e18a4d50409ae"
    http_archive(
        name = name,
        sha256 = "ff6081954d05041f5f583be69dfc9749a71030d8da64b4b9a7576350911b806a",
        strip_prefix = "SdfLib-{}".format(commit),
        build_file = "@mujoco//tools/workspace/SdfLib:package.BUILD.bazel",
        url = "https://github.com/UPC-ViRVIG/SdfLib/archive/{}.tar.gz".format(commit),
    )
