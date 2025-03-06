load("@mujoco//tools/workspace/MarchingCubeCpp:repository.bzl", "MarchingCubeCpp_repository")
load("@mujoco//tools/workspace/cereal:repository.bzl", "cereal_repository")
load("@mujoco//tools/workspace/glm:repository.bzl", "glm_repository")
load("@mujoco//tools/workspace/libccd:repository.bzl", "libccd_repository")
load("@mujoco//tools/workspace/lodepng:repository.bzl", "lodepng_repository")
load("@mujoco//tools/workspace/SdfLib:repository.bzl", "SdfLib_repository")
load("@mujoco//tools/workspace/spdlog:repository.bzl", "spdlog_repository")
load("@mujoco//tools/workspace/tinyobjloader:repository.bzl", "tinyobjloader_repository")
load("@mujoco//tools/workspace/pybind11:repository.bzl", "pybind11_repository")
load("@mujoco//tools/workspace/rules_python:repository.bzl", "rules_python_repository")
load("@mujoco//tools/workspace/abseil-cpp:repository.bzl", "abseil_cpp_repository")
load("@mujoco//tools/workspace/qhull:repository.bzl", "qhull_repository")
load("@mujoco//tools/workspace/tinyxml2:repository.bzl", "tinyxml2_repository")
load("@mujoco//tools/workspace/libglvnd:repository.bzl", "libglvnd_repository")
load("@mujoco//tools/workspace/libglfw3:repository.bzl", "libglfw3_repository")
load("@mujoco//tools/workspace/googletest:repository.bzl", "googletest_repository")
load("@mujoco//tools/workspace/aspect_bazel_lib:repository.bzl", "aspect_bazel_lib_repository")
load("@mujoco//tools/workspace/google_benchmark:repository.bzl", "google_benchmark_repository")
load("@mujoco//tools/workspace/eigen:repository.bzl", "eigen_repository")
load("@mujoco//tools/workspace/fmt:repository.bzl", "fmt_repository")

def add_default_repositories(excludes = []):
    if "MarchingCubeCpp" not in excludes:
        MarchingCubeCpp_repository(name = "MarchingCubeCpp")
    if "cereal" not in excludes:
        cereal_repository(name = "cereal")
    if "glm" not in excludes:
        glm_repository(name = "glm")
    if "libccd" not in excludes:
        libccd_repository(name = "libccd")
    if "lodepng" not in excludes:
        lodepng_repository(name = "lodepng")
    if "SdfLib" not in excludes:
        SdfLib_repository(name = "SdfLib")
    if "spdlog" not in excludes:
        spdlog_repository(name = "spdlog")
    if "tinyobjloader" not in excludes:
        tinyobjloader_repository(name = "tinyobjloader")
    if "pybind11" not in excludes:
        pybind11_repository(name = "pybind11")
    if "rules_python" not in excludes:
        rules_python_repository(name = "rules_python")
    if "abseil-cpp" not in excludes:
        abseil_cpp_repository(name = "abseil-cpp")
    if "qhull" not in excludes:
        qhull_repository(name = "qhull")
    if "tinyxml2" not in excludes:
        tinyxml2_repository(name = "tinyxml2")
    if "libglvnd" not in excludes:
        libglvnd_repository(name = "libglvnd")
    if "libglfw3" not in excludes:
        libglfw3_repository(name = "libglfw3")
    if "googletest" not in excludes:
        googletest_repository(name = "googletest")
    if "aspect_bazel_lib" not in excludes:
        aspect_bazel_lib_repository(name = "aspect_bazel_lib")
    if "google_benchmark" not in excludes:
        google_benchmark_repository(name = "google_benchmark")
    if "eigen" not in excludes:
        eigen_repository(name = "eigen")
    if "fmt" not in excludes:
        fmt_repository(name = "fmt")
