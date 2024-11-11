load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "workspace_and_buildfile")

def _deb_archive_impl(ctx):
    for src in ctx.attr.srcs:
        ctx.extract(src)
    workspace_and_buildfile(ctx)

deb_archive = repository_rule(
    doc = """The deb_archive repository rule extracts .deb archive files, allowing to incorporate
        files from Debian packages into your Bazel workspace.""",
    implementation = _deb_archive_impl,
    attrs = {
        "srcs": attr.label_list(
            mandatory = True,
            allow_empty = False,
            doc = "A list of .deb package files to extract.",
        ),
        "build_file": attr.label(
            allow_single_file = True,
            doc = """ A file containing BUILD definitions. This BUILD file will be placed in the
                root of the extracted repository and allows you to define Bazel targets based on
                the extracted contents.""",
        ),
        "build_file_content": attr.string(
            doc = "Optional inline BUILD.bazel file content",
        ),
        "workspace_file": attr.label(
            allow_single_file = True,
            doc = """A file containing WORKSPACE definitions, which will be added to the
                repository’s root.""",
        ),
        "workspace_file_content": attr.string(
            doc = "Optional inline WORKSPACE file content",
        ),
    },
)

def deb_package(name, urls, sha256s, data_archives = [], **kwargs):
    """
    This function downloads `.deb` file from the URLs, extracts the specified data archives, and
    invokes the `deb_archive` rule to handle extraction and setup within Bazel workspace.

    Args:
        name (string): The name of the Bazel target created for this package.
        urls (list of strings): A list of URLs pointing to the .deb packages you want to download
            and extract. Each URL should point directly to a downloadable .deb file.
        sha256s (list of strings): A list of SHA-256 hashes corresponding to each URL in `urls`.
        data_archives (list of string): Specifies the names of the data archive files within each
            `.deb` package that you want to extract.
        **kwargs: Additional arguments passed to the underlying deb_archive rule.
    """
    if not data_archives:
        data_archives = ["data.tar.zst"] * len(urls)
    srcs = []
    for (url, sha256, data_archive) in zip(urls, sha256s, data_archives):
        _, sep, ext = url.rpartition(".")
        src_name = name + "_" + sha256
        http_archive(
            name = src_name,
            url = url,
            sha256 = sha256,
            build_file_content = """exports_files(["{}"])""".format(data_archive),
        )
        srcs.append("@{}//:{}".format(src_name, data_archive))

    deb_archive(
        name = name,
        srcs = srcs,
        **kwargs
    )
