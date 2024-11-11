load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def cereal_repository(name = "cereal"):
    commit = "ebef1e929807629befafbb2918ea1a08c7194554"
    http_archive(
        name = name,
        sha256 = "ce52ae6abcdbda5fecd63abbaa7ab57e54b814b24d7cb6f5096f5753d1975d24",
        strip_prefix = "cereal-{}".format(commit),
        build_file = "@mujoco//tools/workspace/cereal:package.BUILD.bazel",
        url = "https://github.com/USCiLab/cereal/archive/{}.tar.gz".format(commit),
    )
