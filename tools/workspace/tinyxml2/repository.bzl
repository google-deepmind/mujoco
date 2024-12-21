load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def tinyxml2_repository(name = "tinyxml2"):
    commit = "9a89766acc42ddfa9e7133c7d81a5bda108a0ade"
    http_archive(
        name = name,
        sha256 = "562d77a5f3b1631c289e6856a440c2f459ba5e120a529ce70a08279c2d3d5667",
        strip_prefix = "tinyxml2-{}".format(commit),
        build_file = "@mujoco//tools/workspace/tinyxml2:package.BUILD.bazel",
        url = "https://github.com/leethomason/tinyxml2/archive/{}.tar.gz".format(commit),
    )
