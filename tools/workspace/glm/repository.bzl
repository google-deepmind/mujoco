load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def glm_repository(name = "glm"):
    commit = "454d480ceb4ac10ca76ac24b49836b7d5a35f786"
    http_archive(
        name = name,
        sha256 = "ce6894e2bc38e5415a01827e23406776cd6d632c65e60fd2333b13621ab0ef43",
        strip_prefix = "glm-{}".format(commit),
        build_file = "@mujoco//tools/workspace/glm:package.BUILD.bazel",
        url = "https://github.com/g-truc/glm/archive/{}.tar.gz".format(commit),
    )
