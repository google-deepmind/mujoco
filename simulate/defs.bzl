"""Expose header dependencies for viewer builds and SDK packaging."""

load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _headers_only_impl(ctx):
    return [CcInfo(compilation_context = ctx.attr.library[CcInfo].compilation_context)]

headers_only = rule(
    implementation = _headers_only_impl,
    attrs = {"library": attr.label(mandatory = True, providers = [CcInfo])},
)

def _install_headers_impl(ctx):
    headers = [
        header
        for header in ctx.attr.library[CcInfo].compilation_context.headers.to_list()
        if any([header.short_path.endswith(suffix) for suffix in ctx.attr.suffixes])
    ]
    return [DefaultInfo(files = depset(headers))]

install_headers = rule(
    implementation = _install_headers_impl,
    attrs = {
        "library": attr.label(mandatory = True, providers = [CcInfo]),
        "suffixes": attr.string_list(mandatory = True),
    },
)
