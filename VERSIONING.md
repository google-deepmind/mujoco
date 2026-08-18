# MuJoCo Versioning

MuJoCo uses custom semantic versioning from 3.5.0 onwards, see below.

### The problem with Semantic Versioning

The common definition of traditional [Semantic Versioning](https://semver.org/)
is

```
1. MAJOR:  breaking changes
2. MINOR:  new features
3. PATCH:  bug fixes
```

With strict adherence to this definition, most MuJoCo releases are `MAJOR`. This
is because MuJoCo has a very large API surface, especially when considering
[MJCF](https://mujoco.readthedocs.io/en/stable/XMLreference.html) and the
related
[mjSpec](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html)
API. Furthermore, the numerical properties of physics state integration mean
that despite being deterministic, numerical reproducibility across versions is
almost guaranteed to
[not hold](https://mujoco.readthedocs.io/en/stable/computation/index.html#reproducibility).

## From 3.5.0 – semantic versioning

From 3.5.0 onwards MuJoCo uses well-defined versioning with the following
semantics.

### MuJoCo versioning semantics

```
1. SUPERMAJOR:      breaking changes and/or significant new features
2. MAJOR:           breaking changes, possibly new features
3. MINOR_OR_PATCH:  new features or bug fixes
```

`MINOR_OR_PATCH` guarantees API backward compatibility, but may or may not
introduce new features; to find out, read the
[changelog](https://mujoco.readthedocs.io/en/stable/changelog.html), which will
detail the nature of the changes. The changelog will also detail pure
ABI-breaking changes, for example removing an unused attribute from a public
struct. Such changes are considered MINOR. As explained
[here](https://mujoco.readthedocs.io/en/stable/computation/index.html#reproducibility),
numerical reproducibility is *never* guaranteed.

Additionally:

-   `mj_versionString()` works as before, returning e.g. `"4.2.1"`.
-   `mj_version()` works as before, returning an integer. The corresponding
    header constant `mjVERSION_HEADER` is calculated from the components as:

    ```
    mjVERSION = (SUPERMAJOR * 1e6) + (MAJOR * 1e3) + MINOR_OR_PATCH
    ```

    For example, 4.2.1 would be `4002001`.

## Prior to 3.5.0 – no semantics

Until 3.5.0, versioning was a sequence of increasing numbers with no
well-defined semantic, with the exception of major versions introducing
significant new features, like
[3.0.0](https://mujoco.readthedocs.io/en/stable/changelog.html#version-3-0-0-october-18-2023)'s
introduction of the JAX-based
[MJX](https://mujoco.readthedocs.io/en/stable/mjx.html) branch. Additionally:

-   The function
    [`const char* mj_versionString()`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-versionstring)
    returns the version in the form of a string, for example `"3.2.7"`.
-   The function
    [`int mj_version()`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-version)
    returns an integer equal to the version digits concatenated,
    for example for "3.2.7" we'd have `mjVERSION = 327`.
