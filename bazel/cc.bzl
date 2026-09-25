"""C++ dependency adapters."""

load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _angle_includes_impl(ctx):
    roots = depset([dep.label.workspace_root for dep in ctx.attr.deps])
    includes = CcInfo(compilation_context = cc_common.create_compilation_context(
        system_includes = roots,
    ))
    return [cc_common.merge_cc_infos(cc_infos = [dep[CcInfo] for dep in ctx.attr.deps] + [includes])]

angle_includes = rule(
    implementation = _angle_includes_impl,
    attrs = {"deps": attr.label_list(providers = [CcInfo])},
)
