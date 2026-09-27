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

def _static_link_impl(ctx):
    info = ctx.attr.dep[CcInfo]
    inputs = []
    archives = []
    for linker_input in info.linking_context.linker_inputs.to_list():
        libraries = []
        for library in linker_input.libraries:
            if not library.static_library and not library.pic_static_library:
                fail("Static archive required for %s" % linker_input.owner)
            libraries.append(cc_common.create_library_to_link(
                actions = ctx.actions,
                static_library = library.static_library,
                pic_static_library = library.pic_static_library,
                alwayslink = library.alwayslink,
            ))
            archives.extend([archive for archive in [library.static_library, library.pic_static_library] if archive])
        inputs.append(cc_common.create_linker_input(
            owner = linker_input.owner,
            libraries = depset(libraries),
            user_link_flags = linker_input.user_link_flags,
            additional_inputs = depset(linker_input.additional_inputs),
        ))
    return [
        DefaultInfo(files = depset(archives)),
        CcInfo(
            compilation_context = info.compilation_context,
            linking_context = cc_common.create_linking_context(linker_inputs = depset(inputs)),
        ),
    ]

static_link = rule(
    implementation = _static_link_impl,
    attrs = {"dep": attr.label(mandatory = True, providers = [CcInfo])},
    doc = "Expose static archives even when a consumer sets `linkstatic = False`.",
)
