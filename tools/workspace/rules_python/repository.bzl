load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def rules_python_repository(name = "rules_python"):
    version = "0.37.0"
    http_archive(
        name = name,
        sha256 = "0cc05ddb27614baecace068986931e2a6e9f69114e6115fc5dc58250faf56e0f",
        strip_prefix = "rules_python-{}".format(version),
        url = "https://github.com/bazelbuild/rules_python/releases/download/{}/rules_python-{}.tar.gz".format(version, version),
    )
