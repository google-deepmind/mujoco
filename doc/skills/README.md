# MuJoCo Agent Skills

This directory contains workflow guides and guardrails for AI coding agents and
developers working with MuJoCo across C++ and Python.

## Official Documentation (`doc/*.rst`)

These skills focus on concise code skeletons, idiomatic patterns, and
high-frequency gotchas (`# Good:` vs `# Avoid:`). They complement—rather than
duplicate—the official MuJoCo Sphinx manual in `../index.rst`:

- **Local `.rst` files + Sphinx anchors**: Relative links in each skill (such as
  `../../modeling.rst` with `` `.. _CBodyOrdering:` ``) point directly into this
  `doc/` tree. Read short `.rst` chapters (such as `../studio/*.rst`) directly;
  for large reference files like `../XMLreference.rst` (e.g.,
  `` `.. _sensor-contact:` ``) or `../APIreference/functions.rst` (e.g.,
  `` `.. _mj_step:` ``), grep for the literal `.. _Label:` string to jump to the
  target section.
- **Online fallback (standalone skill installs)**: If `doc/skills/` is installed
  outside a MuJoCo checkout, every `doc/<path>.rst` page is published at
  https://mujoco.readthedocs.io/en/stable/ as `<path>.html` (or `/en/latest/`
  for `main` and unreleased chapters like `studio/`;
  `APIreference/functions.rst` renders inside `APIreference/APIfunctions.html`,
  and Sphinx anchors lowercase the label as `#<lowercase-label>`, e.g.
  `.. _StudioDiffSync:` → `#studiodiffsync`).
- **Core manual chapters**: `../overview.rst` | `../modeling.rst` |
  `../XMLreference.rst` | `../computation/index.rst` |
  `../programming/index.rst` | `../python.rst` | `../mjx.rst` |
  `../mjwarp/index.rst` | `../studio/index.rst` | `../APIreference/index.rst`.

## Skill Catalog

| Skill | Directory | Primary Scope | Key APIs & Capabilities |
| :--- | :--- | :--- | :--- |
| **`mujoco-python`** | [`python/`](python/SKILL.md) | Python simulation lifecycle & physics | `MjSpec` → `MjModel` → `MjData`, `mj_step`, `mj_forward`, named access, `bind()`, contact/touch sensors, `mju_*` math, energy diagnostics. |
| **`mujoco-spec-editing`** | [`spec_editing/`](spec_editing/SKILL.md) | Procedural model authoring | Building bodies, geoms, joints, sub-spec attachments with prefixes, actuators (servos, motors), collision bitmasks (`contype`/`conaffinity`), default classes. |
| **`mujoco-rendering`** | [`rendering/`](rendering/SKILL.md) | Offscreen & PBR rendering | Headless RGB, metric depth maps, segmentation masks (`mujoco.Renderer`), Filament PBR (`mujoco.mjrf`, `mujoco.rendering.filament`), C++ `mjr`/`mjrf` pipelines. |
| **`mujoco-accelerated`** | [`accelerated/`](accelerated/SKILL.md) | GPU & TPU batch simulation | High-throughput batch rollouts with MJX (`MJX-JAX`, `MJX-Warp`, `jax.vmap`, `jax.lax.scan`), differentiability, CUDA batching with MJWarp, contact/constraint sizing. |
| **`mujoco-parity`** | [`parity/`](parity/SKILL.md) | Cross-engine physics parity | Comparing CPU (`float64`) vs MJWarp/MJX (`float32`): stage-by-stage `forward` isolation, stiff actuator & `qM` rounding bounds, contact matching, batched 1-step & `float32` baseline rollouts. |
| **`mujoco-gui`** | [`gui/`](gui/SKILL.md) | Dear ImGui & ImPlot UI design | Immediate-mode UI patterns, layout, styling, ID stack rules (`##` vs `###`), list virtualization (`ImGuiListClipper`), automated testing hooks. |
| **`mujoco-studio`** | [`studio/`](studio/SKILL.md) | Studio apps & interactive viewers | Decoupled Sim/Viewer threads, message channels (Snapshots vs Events), plugin architecture in Python and C++, hot reload, WebViewer WASM streaming. |

## How to Use These Skills

- **For AI Coding Agents**: Each skill contains standardized YAML frontmatter
  with `Use when:` triggers and `Don't use for:` exclusions to assist automated
  orchestrators in choosing the relevant guide.
- **For Developers**: Skills pair quick-start recipes and common pitfalls with
  targeted pointers into the `doc/*.rst` reference manual.
