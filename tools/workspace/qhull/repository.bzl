load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def qhull_repository(name = "qhull"):
    commit = "0c8fc90d2037588024d9964515c1e684f6007ecc"
    http_archive(
        name = name,
        sha256 = "227afadbfe4b934ef7da385dac0c7e7abf5d977fb3d4734b299e7b38a334a45a",
        strip_prefix = "qhull-{}".format(commit),
        build_file = "@mujoco//tools/workspace/qhull:package.BUILD.bazel",
        url = "https://github.com/qhull/qhull/archive/{}.tar.gz".format(commit),
    )
