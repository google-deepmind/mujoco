# MuJoCo Plug-in for Unity

## [2.1.2] - 2022-03-15

### Minor modifications

When converting a world geom to a free-moving body, the parent body is created
where the geom is, rather than at the root.

MJCF XML generation is now locale-invariant, addressing a parsing issue on
systems with a decimal comma.

File name changes:

- Added `.Runtime` to `Mujoco.asmdef` and the corresponding test file.
- Reorganized the files under `Runtime/Componenets` into different
  subdirectories.
- File directories for Editor tests are now named with a capital letter.

## [2.1.1] - 2022-02-02

### Initial release
