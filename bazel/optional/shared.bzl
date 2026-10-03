"""Expose optional renderer libraries through shared-library imports."""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_import", "cc_library")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _headers_only_impl(ctx):
    return [CcInfo(compilation_context = ctx.attr.library[CcInfo].compilation_context)]

headers_only = rule(
    implementation = _headers_only_impl,
    attrs = {"library": attr.label(mandatory = True, providers = [CcInfo])},
)

def optional_shared_library(name, deps, headers = [], dynamic_deps = []):
    """Declare a shared library and its public linking interface.

    Args:
        name: Public target name and library basename.
        deps: Libraries linked into the shared library.
        headers: Header targets exposed to consumers.
        dynamic_deps: Shared dependencies whose implementations stay outside this library.
    """
    for suffix, constraints, linkopts in [
        ("so", ["@platforms//os:linux"], ["-Wl,-soname,lib" + name + ".so", "-Wl,-rpath,\'$ORIGIN\'", "-Wl,-rpath,\'$ORIGIN/../../lib\'"]),
        ("dylib", ["@platforms//os:macos"], ["-Wl,-install_name,@rpath/lib" + name + ".dylib", "-Wl,-rpath,@loader_path", "-Wl,-rpath,@loader_path/../../lib"]),
    ]:
        cc_binary(
            name = "lib" + name + "." + suffix,
            deps = deps,
            dynamic_deps = dynamic_deps,
            linkshared = True,
            linkstatic = True,
            linkopts = linkopts,
            tags = ["manual"],
            target_compatible_with = constraints,
        )

    native.alias(
        name = name + "_binary",
        actual = select({
            "//bazel:macos": ":lib" + name + ".dylib",
            "//conditions:default": ":lib" + name + ".so",
        }),
        visibility = ["//visibility:public"],
    )
    cc_import(
        name = name + "_import",
        shared_library = ":" + name + "_binary",
    )
    cc_library(
        name = name,
        deps = [":" + name + "_import"] + headers,
        visibility = ["//visibility:public"],
    )
