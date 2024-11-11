load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def google_benchmark_repository(name = "google_benchmark"):
    version = "1.8.4"
    http_archive(
        name = name,
        sha256 = "3e7059b6b11fb1bbe28e33e02519398ca94c1818874ebed18e504dc6f709be45",
        strip_prefix = "benchmark-{}".format(version),
        url = "https://github.com/google/benchmark/archive/refs/tags/v{}.tar.gz".format(version),
    )
