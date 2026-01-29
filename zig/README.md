# MuJoCo Zig Bindings

These are the canonical Zig bindings for the MuJoCo
physics engine.

This package provides a way to build zig and interact with libmujoco / mujoco.h using Zig direct C interop feature.

> [!IMPORTANT]
> _These bindings are still a WIP. 

## Prerequisites

> [!NOTE]
> Run all the commands in this README from the top-level directory.

- To compile the the library run `zig build`
  All the files in the `zig/examples` directory get comiled by default.
  
  As an example to run them all you need to do is remove the `.zig` extension from the filename.
  
  So newexample.zig can be ran / built with:
  
  ```sh
  zig build run-newexample
  ```

