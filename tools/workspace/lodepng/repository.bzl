load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def lodepng_repository(name = "lodepng"):
    commit = "b4ed2cd7ecf61d29076169b49199371456d4f90b"
    http_archive(
        name = name,
        sha256 = "b67e466ba659c07ac775d07dbd97af319cde449ce14abed9ae596df29d888603",
        strip_prefix = "lodepng-{}".format(commit),
        build_file = "@mujoco//tools/workspace/lodepng:package.BUILD.bazel",
        url = "https://github.com/lvandeve/lodepng/archive/{}.tar.gz".format(commit),
    )
