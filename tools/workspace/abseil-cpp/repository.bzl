load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def abseil_cpp_repository(name = "abseil-cpp"):
    version = "20240722.0"
    http_archive(
        name = name,
        sha256 = "f50e5ac311a81382da7fa75b97310e4b9006474f9560ac46f54a9967f07d4ae3",
        strip_prefix = "abseil-cpp-{}".format(version),
        url = "https://github.com/abseil/abseil-cpp/releases/download/{}/abseil-cpp-{}.tar.gz".format(version, version),
    )
