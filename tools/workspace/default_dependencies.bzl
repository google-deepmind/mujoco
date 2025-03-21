load("@rules_python//python:repositories.bzl", "py_repositories")
load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies")

def add_default_dependencies(excludes = []):
    if "rules_python" not in excludes:
        py_repositories()
    if "aspect_bazel_lib" not in excludes:
        aspect_bazel_lib_dependencies()
