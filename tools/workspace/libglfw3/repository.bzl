load("@mujoco//tools/workspace:deb_archive.bzl", "deb_package")

def libglfw3_repository(name = "libglfw3"):
    deb_package(
        name = name,
        urls = [
            "http://mirrors.kernel.org/ubuntu/pool/universe/g/glfw3/libglfw3_3.3.6-1_amd64.deb",
            "http://mirrors.kernel.org/ubuntu/pool/universe/g/glfw3/libglfw3-dev_3.3.6-1_amd64.deb",
        ],
        sha256s = [
            "a7fe08d4ded1c376e8b008fdeef220b8a0335258a05c542982424cdb43b61af3",
            "2b2a68725a34dd69bcce045282806f1d1ffa6c43376aa1023058683f51a31b02",
        ],
        data_archives = [
            "data.tar.zst",
            "data.tar.zst",
        ],
        build_file = "@mujoco//tools/workspace/libglfw3:package.BUILD.bazel",
    )
