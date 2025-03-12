def _append(a, b):
    if b == None:
        return a
    else:
        return a + b

def _get_default_copts():
    return [
        "-fvisibility=hidden",
        "-Werror",
        "-Wall",
        "-Wpedantic",
        "-Wimplicit-fallthrough",
        "-Wunused",
        "-Wvla",
        "-Wno-int-in-bool-context",
        "-Wno-sign-compare",
        "-Wno-unknown-pragmas",
    ] + select({
            "//tools:gcc": [
                "-Wno-maybe-uninitialized",
            ],
            "//tools:clang": [
                "-Wno-unused-but-set-variable",
            ],
            "//conditions:default": [],
        })

def _get_default_defines():
    return []

def _get_default_deps():
    return []

def _get_default_linkopts():
    return [
        "-Wl,--gc-sections",
    ]


def mj_cc_library(name, deps = None, srcs = None, hdrs = None, copts = None, defines = None, linkopts = None, **kwargs):
    native.cc_library(
        name = name,
        deps = _append(_get_default_deps(), deps),
        srcs = srcs,
        hdrs = hdrs,
        copts = _append(_get_default_copts(), copts),
        defines = _append(_get_default_defines(), defines),
        linkopts = _append(_get_default_linkopts(), linkopts),
        **kwargs
    )
