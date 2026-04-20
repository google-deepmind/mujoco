# Contributing to MuJoCo

We intend for MuJoCo to be a true community-driven project and look forward to
accepting your contributions!

## Before you contribute

### Documentation, forums

Please read MuJoCo's [documentation](https://mujoco.readthedocs.io/) and look
through current topics on our GitHub
[issues](https://github.com/google-deepmind/mujoco/issues) and
[discussions](https://github.com/google-deepmind/mujoco/discussions) pages.

### Contributor License Agreement

Contributions to this project must be accompanied by a Contributor License
Agreement (CLA). You (or your employer) retain the copyright to your
contribution; this simply gives us permission to use and redistribute your
contributions as part of the project. Head over to
<https://cla.developers.google.com/> to see your current agreements on file or
to sign a new one.

You generally only need to submit a CLA once, so if you've already submitted one
(even if it was for a different project), you probably don't need to do it
again.

## Contributing

### Reporting bugs

How to submit a good bug report:

- Use a clear and descriptive title.

- Make it easy to reproduce the problem. If this requires a model, attach it as
a zip file to the bug report. The model and steps required to reproduce the
problem should be *minimal*, in the sense that irrelevant parts are
removed.

- Clearly state what is the expected behavior.

- Include an illustrative screenshot, if relevant.

Try to provide context:

- If the problem is new, see if you can reproduce it in an older version.
What's the most recent version in which the problem doesn't happen?

- Can you reproduce the problem on multiple platforms?

### Suggesting enhancements

Before submitting an enhancement suggestion:

- Check if you're using the [latest
version](https://github.com/google-deepmind/mujoco/releases/latest) of MuJoCo.

- Perform a quick [search](https://github.com/google-deepmind/mujoco/issues) to
see if the enhancement has already been suggested. If it has, add a comment to
the existing issue instead of opening a new one.

How to submit a good enhancement suggestion:

- Use a clear and descriptive title.

- Describe the current behaviour and the behavior which you hope to see instead.

- Explain why this enhancement would be useful.

- Specify the version of MuJoCo and platform/OS you are using.

### Contributing code

- Except for small and straightforward bugfixes, please get in touch with us
before you start working on a contribution so that we can help and possibly
guide you. Coordinating up front makes it much easier to avoid frustration later
on.

- All submissions require review. Please use GitHub pull requests for this
purpose. Please consult
[GitHub Help](https://help.github.com/articles/about-pull-requests/) for more
information on pull requests.

- Write tests. MuJoCo uses [googletest](https://github.com/google/googletest)
for C++ tests, [absltest](https://abseil.io/docs/python/guides/testing) for
Python binding code and [nunit](https://nunit.org/) for C# code in the Unity
plugin. In most cases, a pull request will only be accepted if it includes
tests. MuJoCo's internal codebase is currently lacking in test coverage. If you
want to modify a function that isn't covered by tests, you'll be expected to
contribute tests for the existing functionality, not just your modification. In
fact, writing a test for existing code is a great way to get started with
contributions.

- Resolve compiler warnings.

- All existing tests must pass.

- Follow the [Style Guide](./STYLEGUIDE.md). In particular, adequately comment
your code.

- Make small pull requests. We will likely ask you to split up a large pull
request into self-contained, smaller ones, especially if the PR is trying to
achieve multiple things.

- Respond to reviewers. Please be responsive to any questions and comments.

Once you have met all the requirements, your code will be merged.
Thanks for improving MuJoCo!



### Community guidelines

This project follows Google's
[Open Source Community Guidelines](https://opensource.google/conduct/).


## Regenerating the MJCF XSD

The MJCF XSD shipped with releases lives at `doc/mjcf.xsd` and is
auto-generated from `src/xml/` and `doc/XMLreference.rst`. If your change
touches MJCF parsing, adds an attribute, or edits the reference docs,
regenerate it:

```bash
cmake --build build --target xmlschema
./build/bin/xmlschema /tmp/raw.xsd
./doc/mjcf_schema_enrich.py \
    --in /tmp/raw.xsd --rst doc/XMLreference.rst --out doc/mjcf.xsd \
    --strict --report
```

`doc/mjcf_schema_enrich.py` is a [uv](https://docs.astral.sh/uv/) PEP 723
script; its shebang provisions a matching Python interpreter, so the only
prerequisite is `uv` on `PATH`. Run it as `./doc/mjcf_schema_enrich.py ...`
or `uv run doc/mjcf_schema_enrich.py ...`.

`--strict` exits non-zero on drift between the parser, the attribute table
(`src/xml/xml_native_schema.cc`), and the RST — fix warnings before
committing. See the header comment in `xml_native_schema.cc` for which files
to edit when adding a new attribute.
