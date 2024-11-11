load("@mujoco//tools/workspace:deb_archive.bzl", "deb_package")

def libglvnd_repository(name = "libglvnd"):
    deb_package(
        name = name,
        urls = ["http://mirrors.kernel.org/ubuntu/pool/main/libg/libglvnd/libglvnd-dev_1.4.0-1_amd64.deb"],
        sha256s = ["af38c9f0f7f65735d89094cb5bc32afa58012e4bd48bae995653c465dc110165"],
        data_archives = ["data.tar.zst"],
        build_file = "@mujoco//tools/workspace/libglvnd:package.BUILD.bazel",
    )
