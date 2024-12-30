package(default_visibility = [
    "@mujoco//:__subpackages__",
    "@spdlog//:__subpackages__",
])

cc_library(
    name = "fmt",
    srcs = [
        "src/format.cc",
        "src/os.cc",
    ],
    hdrs = [
        "include/fmt/args.h",
        "include/fmt/chrono.h",
        "include/fmt/color.h",
        "include/fmt/compile.h",
        "include/fmt/core.h",
        "include/fmt/format.h",
        "include/fmt/format-inl.h",
        "include/fmt/os.h",
        "include/fmt/ostream.h",
        "include/fmt/printf.h",
        "include/fmt/ranges.h",
        "include/fmt/std.h",
        "include/fmt/xchar.h",
    ],
    data = [":license_filegroup"],
    includes = ["include"],
)

filegroup(
    name = "license_filegroup",
    srcs = ["LICENSE"],
    tags = ["license_filegroup"],
)
