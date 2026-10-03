"""Fetch optional renderer, Studio, and WebAssembly dependencies."""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@emsdk//:emscripten_build_file.bzl", "EMSCRIPTEN_BUILD_FILE_CONTENT_TEMPLATE")
load(":filament_packages.bzl", "FILAMENT_PACKAGE_FILES")

def _filament_impl(ctx):
    ctx.download_and_extract(
        type = "tar.gz",
        url = "https://codeload.github.com/google/filament/tar.gz/ae6ebcabaa8d17d063272886bbf06780fb10fd91",
        sha256 = "35c5e7f57c67e731aa74303587919735e79060e19fd1ae0d0b67323a5c97eebc",
        stripPrefix = "filament-ae6ebcabaa8d17d063272886bbf06780fb10fd91",
    )
    ctx.patch(ctx.attr.windows_patch, strip = 1)
    abseil_wrapper = ctx.read("third_party/abseil/tnt/CMakeLists.txt")
    abseil_dummy = ctx.read("third_party/abseil/tnt/dummy.c")
    ctx.delete("third_party/abseil")
    ctx.download_and_extract(
        type = "tar.gz",
        url = "https://codeload.github.com/abseil/abseil-cpp/tar.gz/2065f4ded0558c6f89fee67c8e5228feb4eb960e",
        sha256 = "7f4240fe135c0b0dcdd2efa664f1393b1da7e25031e17560515f452182aa0c5e",
        stripPrefix = "abseil-cpp-2065f4ded0558c6f89fee67c8e5228feb4eb960e",
        output = "third_party/abseil",
    )
    ctx.file("third_party/abseil/tnt/CMakeLists.txt", abseil_wrapper)
    ctx.file("third_party/abseil/tnt/dummy.c", abseil_dummy)
    for path in FILAMENT_PACKAGE_FILES:
        ctx.delete(path)
    ctx.file("CMakeLists.txt", ctx.read("CMakeLists.txt") + "\n" + ctx.read(ctx.attr.install_rules))
    ctx.download(
        url = "https://raw.githubusercontent.com/chaotic-cx/mesa-mirror/ba92104b564514117a0bcc4120dff6089965eabc/include/GL/glx.h",
        sha256 = "253068af8ea6c3a8f1043d174457d003a48ecf40bed6663f4d21eb0e0d566de0",
        output = "bazel-headers/GL/glx.h",
    )
    ctx.symlink(ctx.attr.build_file, "BUILD.bazel")

_filament = repository_rule(
    implementation = _filament_impl,
    attrs = {
        "build_file": attr.label(default = Label("//bazel/optional:filament.BUILD.bazel")),
        "install_rules": attr.label(default = Label("//bazel/optional:filament-install.cmake")),
        "windows_patch": attr.label(default = Label("//:cmake/filament-allow-clang-windows.patch")),
    },
)

def _emscripten_impl(ctx):
    ctx.download_and_extract(
        url = "https://storage.googleapis.com/webassembly/emscripten-releases-builds/linux/8103ffedfb0c42d231c6af6859a5a1a832260b43/wasm-binaries.tar.xz",
        sha256 = "0183f887b56c3f8d4b45826cb49856a3324afb66236ad3c13944c0fd2550cbbc",
        stripPrefix = "install",
    )
    ctx.download_and_extract(
        url = "https://registry.npmjs.org/typescript/-/typescript-5.8.2.tgz",
        sha256 = "ef938a45323df5775664ea5d55e8bc0ab2027a40db1ff857bb957fe7bbaa4434",
        stripPrefix = "package",
        output = "emscripten/node_modules/typescript",
    )
    ctx.file("emscripten/node_modules/.bin/tsc", "require('../typescript/lib/tsc.js');\n")
    for template in ctx.attr.templates:
        ctx.file("emscripten_toolchain/" + template.name, ctx.read(template), executable = template.name.endswith(".sh"))
    config = ctx.read(ctx.attr.config_rule)
    if "/lib/clang/24/include" not in config:
        fail("The pinned Emscripten toolchain resource-header path has changed.")
    config = config.replace("/lib/clang/24/include", "/lib/clang/21/include")
    config = config.replace('load(":platform_info.bzl",', "load(" + repr(str(ctx.attr.platform_info_bzl)) + ",")
    ctx.file("emscripten_toolchain/toolchain.bzl", config)
    ctx.file("BUILD.bazel", EMSCRIPTEN_BUILD_FILE_CONTENT_TEMPLATE.format(bin_extension = ""))
    ctx.file("emscripten_toolchain/BUILD.bazel", "load(" + repr(str(ctx.attr.toolchain_rule)) + ", 'mujoco_emscripten_toolchain')\npackage(default_visibility = ['//visibility:public'])\nmujoco_emscripten_toolchain(name = 'toolchain')\n")

_emscripten = repository_rule(
    implementation = _emscripten_impl,
    attrs = {
        "config_rule": attr.label(default = Label("@emsdk//emscripten_toolchain:toolchain.bzl")),
        "platform_info_bzl": attr.label(default = Label("@emsdk//emscripten_toolchain:platform_info.bzl")),
        "templates": attr.label_list(default = [
            Label("@emsdk//emscripten_toolchain:" + name)
            for name in ["emcc.sh", "emcc_link.sh", "emar.sh", "env.sh", "link_wrapper.py", "default_config"]
        ]),
        "toolchain_rule": attr.label(default = Label("//bazel/optional:emscripten.bzl")),
    },
)

def _wasm_npm_impl(ctx):
    packages = json.decode(ctx.read(ctx.attr.lockfile))["packages"]
    for path, package in packages.items():
        if not path:
            continue
        if not path.startswith("node_modules/") or ".." in path.split("/"):
            fail("Invalid npm package path: " + path)
        ctx.download_and_extract(
            url = package["resolved"],
            integrity = package["integrity"],
            stripPrefix = path.removeprefix("node_modules/@types/") if path.startswith("node_modules/@types/") else "package",
            output = path,
        )
    ctx.file("BUILD.bazel", "package(default_visibility = ['//visibility:public'])\nexports_files(['node_modules/typescript/package.json'])\nfilegroup(name = 'node_modules', srcs = glob(['node_modules/**']))\n")

_wasm_npm = repository_rule(
    implementation = _wasm_npm_impl,
    attrs = {"lockfile": attr.label(default = Label("//wasm:package-lock.json"))},
)

def _optional_dependencies_impl(_ctx):
    _filament(name = "mujoco_filament")
    _emscripten(name = "mujoco_emscripten")
    _wasm_npm(name = "mujoco_wasm_npm")
    for name, project, version, sha256 in [
        ("imgui", "ocornut/imgui", "b48d1afbe8ee8b238e2961dc363a949dd7304e23", "747c5a465a05c4ef5b4a09e2e0e3d0ce2bc76b31420f0f5af33df470907cf893"),
        ("implot", "epezent/implot", "524f9fcd48d76c13fdf94c5ffbba8787a1ff7e39", "1a8f01f7676070b98af8316eb91b02f462c0c613343b8ba847ddfce363b9b9df"),
        ("sdl2", "libsdl-org/SDL", "98d1f3a45aae568ccd6ed5fec179330f47d4d356", "be55f7015a5599faa869b20078de83df7e83c35585ee1c3ee203a0e58eefebbd"),
        ("libwebp", "webmproject/libwebp", "v1.6.0", "93a852c2b3efafee3723efd4636de855b46f9fe1efddd607e1f42f60fc8f2136"),
    ]:
        http_archive(
            type = "tar.gz",
            name = "mujoco_" + name,
            urls = ["https://codeload.github.com/" + project + "/tar.gz/" + version],
            sha256 = sha256,
            strip_prefix = project.split("/")[1] + "-" + version.removeprefix("v"),
            build_file = Label("//bazel/optional:" + name + ".BUILD.bazel"),
            patches = [Label("//:cmake/libwebp-apple-float16.patch")] if name == "libwebp" else [],
            patch_args = ["-p1"],
        )
    for name, project, version, sha256 in [
        ("font_next", "googlefonts/atkinson-hyperlegible-next", "5d633f80fc654ef5fffa7cfc257528685158dcef", "4d03375ce61dac53bc2d0ed31a172efb50361333dceec5a4f8bf4d9e662c2967"),
        ("font_mono", "googlefonts/atkinson-hyperlegible-next-mono", "154d50362016cc3e873eb21d242cd0772384c8f9", "d8b50ca876781ef6c2f0e1dd1a7ed6896a7f7769242e76be901b98c6d7edfafb"),
        ("font_awesome", "FortAwesome/Font-Awesome", "a8386aae19e200ddb0f6845b5feeee5eb7013687", "d72fbf7ffff357d646d5178a6a94edf686179673271c7f20a067712f46677de8"),
        ("newton_schemas", "newton-physics/newton-usd-schemas", "v0.4.0", "5ede9b2f35da979bea23cf7efad2955030a7b722718d826f34b927c49d37cea0"),
    ]:
        http_archive(
            type = "tar.gz",
            name = "mujoco_" + name,
            urls = ["https://codeload.github.com/" + project + "/tar.gz/" + version],
            sha256 = sha256,
            strip_prefix = project.split("/")[1] + "-" + version.removeprefix("v"),
            build_file_content = "exports_files(glob([\"**\"]))\n",
        )

mujoco_optional_dependencies = module_extension(implementation = _optional_dependencies_impl)
