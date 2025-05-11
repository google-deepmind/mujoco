# MuJoCo Style Guide

The MuJoCo codebase follows an internally consistent style that values
compactness and readability. Please try to follow the style guide as closely as
possible in your code contributions.

### Scope of this guide

Most of this guide involves C/C++ code. For Python, jump to the section [below](#python-code). For MuJoCo C/C++, code has three main categories:

1. **C code:** MuJoCo's core codebase. It consists of public headers under
`include/` and C source files and internal headers under `src/`. This style
guide primarily concerns itself with this category.

2. **Legacy C++:** Files under `src/user/` and `src/xml/`. These do not
necessarily follow best C++ practices. We intend to gradually replace these with
new code that follows the [Google C++
style](https://google.github.io/styleguide/cppguide.html) over time.

3. **New code:** This includes C++ files under `test/` and `python/` and C#
files under `unity/`. Added by DeepMind engineers, this code adheres to the
[Google style](https://google.github.io/styleguide/).

### General principles

Where any aspect of coding style is not explicitly spelled out in this guide,
the following principle is followed:

| Maximise consistency with the rest of the code. |
| --- |

If there is a contradiction between this guide and existing code, the guide
takes precedence. Additional principles include:

- Follow the [naming conventions](https://mujoco.readthedocs.io/en/latest/programming#naming-convention).
- Be sparing with horizontal space: Try to keep lines short, avoid line-breaks
  where possible.
- Be generous with vertical space: Empty lines between code blocks are good.
- Keep names short.
- Inline comments are part of the code, treat them as such.
- Use American English in comments and documentation.

### Specific rules for C code

Over time, this style guide will be expanded to cover most aspects of C
programming in the MuJoCo codebase. In the meantime, it is usually enough to
inspect existing code and try to follow its example.

If there are any consistent coding patterns that are specific to the MuJoCo
codebase but aren't mentioned in the guide, the guide should be expanded. If you
spot such a pattern, feel free to send a PR to update the guide.

#### Indentation

2-space indents, using space characters rather than tabs.

#### Line length

Line length is 100 characters. In rare situations, like the collision table at
the top of
[engine_collision_driver.c](https://github.com/google-deepmind/mujoco/blob/c8ff7b3d341560e8cc33fbdcaffbcdbc4c32327c/src/engine/engine_collision_driver.c#L36),
longer lines are allowed for readability.

#### Comments

MuJoCo makes generous use of short, one-line comments describing the code block
just below them. They are considered an essential part of the code. Comments
should be:

- As succinct as possible, while maintaining clarity.
- Preceded by an empty line, unless at the top of a block.
- Uncapitalized and not terminated by a full-stop.

A helpful heuristic regarding in-code comments is that the reader should be able
to get a sense of what is happening in a function just by reading the comments.

An exception to the third bullet point above are function declaration comments
in public header files which are considered to be docstrings rather than code
and are therefore capitalized and terminated by a full stop. These docstrings
are required.

#### Braces

- MuJoCo uses
[attached K&R braces](https://en.wikipedia.org/wiki/Indentation_style#Variant:_mandatory_braces),
including for one-line blocks:

  ```C
  // transpose matrix
  void mju_transpose(mjtNum* res, const mjtNum* mat, int nr, int nc) {
    for (int i=0; i < nr; i++) {
      for (int j=0; j < nc; j++) {
        res[j*nr+i] = mat[i*nc+j];
      }
    }
  }
  ```

- Brace-less single line statements are allowed outside of `engine/` code, for
similar, repeated blocks, that do not contain flow control statements (`return`,
`continue`, etc.). For an example of this exception, inspect the [`mjCModel`
destructor](https://github.com/google-deepmind/mujoco/search?q=repo%3Adeepmind%2Fmujoco+filename%3Auser_model.cc).

- Unattached braces are allowed in `if/else` blocks, when inserting a comment
before the `else`:

  ```C
  // rotate vector by quaternion
  void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4]) {
    // null quat: copy vec
    if (quat[0] == 1 && quat[1] == 0 && quat[2] == 0 && quat[3] == 0) {
      mju_copy3(res, vec);
    }

    // regular processing
    else {
      mjtNum mat[9];
      mju_quat2Mat(mat, quat);
      mju_mulMatVec3(res, mat, vec);
    }
  }
  ```

#### Spacing

- MuJoCo encourages judicious use of spacing around operators to promote
readability. For example below, note the lack of spaces around the
multiplication operator, and the aligning spaces in the second and fourth
assignments:

  ```C
  // time-derivative of quaternion, given 3D rotational velocity
  void mju_derivQuat(mjtNum res[4], const mjtNum quat[4], const mjtNum vel[3]) {
    res[0] = 0.5*(-vel[0]*quat[1] - vel[1]*quat[2] - vel[2]*quat[3]);
    res[1] = 0.5*( vel[0]*quat[0] + vel[1]*quat[3] - vel[2]*quat[2]);
    res[2] = 0.5*(-vel[0]*quat[3] + vel[1]*quat[0] + vel[2]*quat[1]);
    res[3] = 0.5*( vel[0]*quat[2] - vel[1]*quat[1] + vel[2]*quat[0]);
  }
  ```

- Spaces are required around comparison operators.

- Spaces are not allowed around operators in array subscripts `[]` or in
  variable initialisation in `for` loops. For example, inspect the
  `mju_transpose` implementation above.

- Three blank lines are required between function implementations in source files.

#### Variable declarations

Historically the MuJoCo C codebase used exclusively C89-style variable
declarations, with all stack variables pre-declared at the top of the function.
We are in the process of migrating the code to the C99 convention of declaring
variables at the narrowest possible scope. For example iterator variables in
for-loops are mostly declared in the narrow scope, as in the `mju_transpose`
example above.

New code should use the C99 convention. When editing an existing function,
please move existing variable declarations into local scope. Pull requests
helping us to complete the migration are very welcome.

### [Python code](#python-code)

For Python code, run `pyink foo.py` to adhere to Google's [Python style guide](https://google.github.io/styleguide/pyguide.html). For sorting and cleaning imports, run `isort foo.py`. Both `pyink` and `isort` can be pip installed via `pip install pyink isort`.
