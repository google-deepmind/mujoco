THIN_LTO_FEATURE_IF_CLANG = select({
    "//tools:clang": ["thin_lto"],
    "//conditions:default": [],
})

THIN_LTO_LINKOPTS_IF_CLANG = select({
    "//tools:clang": ["-flto=thin"],
    "//conditions:default": [],
})
