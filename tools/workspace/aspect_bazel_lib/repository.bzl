load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def aspect_bazel_lib_repository(name = "aspect_bazel_lib"):
    version = "2.9.3"
    http_archive(
        name = name,
        sha256 = "a272d79bb0ac6b6965aa199b1f84333413452e87f043b53eca7f347a23a478e8",
        strip_prefix = "bazel-lib-{}".format(version),
        url = "https://github.com/bazel-contrib/bazel-lib/releases/download/v{}/bazel-lib-v{}.tar.gz".format(version, version),
    )
