load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def MarchingCubeCpp_repository(name = "MarchingCubeCpp"):
    commit = "f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d"
    http_archive(
        name = name,
        sha256 = "227c10b2cffe886454b92a0e9ef9f0c9e8e001d00ea156cc37c8fc43055c9ca6",
        strip_prefix = "MarchingCubeCpp-{}".format(commit),
        build_file = "@mujoco//tools/workspace/MarchingCubeCpp:package.BUILD.bazel",
        url = "https://github.com/aparis69/MarchingCubeCpp/archive/{}.tar.gz".format(commit),
    )
