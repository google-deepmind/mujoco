"""Compile the static and shared library implementations separately."""

load("@rules_cc//cc:defs.bzl", "cc_library")

def mujoco_core(c_srcs, cc_srcs):
    """Declare the engine libraries and their public static interface.

    Args:
        c_srcs: Engine and renderer C sources.
        cc_srcs: Engine, compiler, and decoder C++ sources.
    """
    copts = select({
        "//bazel:windows": ["/Gy", "/Gw", "/Oi"],
        "//conditions:default": ["-fvisibility=hidden", "-ffunction-sections", "-fdata-sections"],
    }) + select({
        "//bazel:avx_windows": ["/arch:AVX"],
        "//bazel:avx_posix": ["-mavx"],
        "//conditions:default": [],
    }) + select({
        "//bazel:harden_windows": ["/GS"],
        "//bazel:harden_enabled": ["-D_FORTIFY_SOURCE=2", "-fstack-protector"],
        "//conditions:default": [],
    }) + select({
        "//bazel:macos": ["-mmacosx-version-min=11", "-Werror=partial-availability", "-Werror=unguarded-availability"],
        "//conditions:default": [],
    }) + select({
        "//bazel:lto_windows": ["/GL"],
        "//bazel:lto_enabled": ["-flto"],
        "//conditions:default": [],
    })
    deps = [
        ":internal",
        "@mujoco_deps_ccd//:ccd",
        "@lodepng//:lodepng",
        "@mujoco_deps_qhull//:qhull",
        "@mujoco_deps_tinyxml2//:tinyxml2",
        "@mujoco_deps_tinyobjloader//:tinyobjloader",
        "@miniz//:miniz",
        "@marchingcubecpp//:marchingcubecpp",
    ]
    for kind in ["static", "shared"]:
        local_defines = ["CCD_STATIC_DEFINE", "TINYOBJLOADER_IMPLEMENTATION", "MC_IMPLEM_ENABLE"] + (["MJ_STATIC"] if kind == "static" else ["MUJOCO_DLL_EXPORTS"])
        local_defines += select({
            "//bazel:windows": ["_CRT_SECURE_NO_WARNINGS", "_CRT_SECURE_NO_DEPRECATE"],
            "//conditions:default": ["_GNU_SOURCE"],
        }) + select({
            "//bazel:intrinsics": ["mjUSEPLATFORMSIMD"],
            "//conditions:default": [],
        })
        cc_library(
            name = "core_c_" + kind,
            srcs = c_srcs,
            copts = copts + select({"//bazel:windows": ["/std:c11"], "//conditions:default": ["-std=c11"]}),
            local_defines = local_defines,
            deps = deps,
            alwayslink = True,
            linkstatic = True,
        )
        cc_library(
            name = "core_" + kind,
            srcs = cc_srcs,
            copts = copts + select({"//bazel:windows": ["/std:c++20"], "//conditions:default": ["-std=c++20"]}) + select({"//bazel:wasm": ["-fexceptions"], "//conditions:default": []}),
            local_defines = local_defines,
            deps = deps + [":core_c_" + kind],
            alwayslink = True,
            linkstatic = True,
            linkopts = select({
                "//bazel:windows": [],
                "//bazel:wasm": [],
                "//bazel:macos": ["-pthread", "-mmacosx-version-min=11", "-Wl,-no_weak_imports"],
                "//conditions:default": ["-pthread", "-ldl", "-lm"],
            }) + select({
                "//bazel:lto_windows": ["/LTCG"],
                "//bazel:lto_enabled": ["-flto"],
                "//conditions:default": [],
            }) + select({
                "//bazel:harden_linux": ["-Wl,-z,relro", "-Wl,-z,now"],
                "//bazel:harden_macos": ["-Wl,-bind_at_load"],
                "//conditions:default": [],
            }),
        )
    cc_library(
        name = "mujoco",
        deps = [":headers"],
        implementation_deps = [":core_static"],
        defines = ["MJ_STATIC"],
        linkstatic = True,
        visibility = ["//visibility:public"],
    )
