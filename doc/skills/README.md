# MuJoCo Agent Skills

This directory contains specialized guides and skills designed for AI agents
and software engineers developing and simulating with MuJoCo across C++ and
Python.

## Skill Catalog

| Skill | Directory | Primary Scope | Key APIs & Capabilities |
| :--- | :--- | :--- | :--- |
| **`mujoco-python`** | [`python/`](python/SKILL.md) | Python simulation lifecycle & physics | `MjSpec` → `MjModel` → `MjData`, `mj_step`, `mj_forward`, named access, `bind()`, contact/touch sensors, `mju_*` math, energy diagnostics. |
| **`mujoco-spec-editing`** | [`spec_editing/`](spec_editing/SKILL.md) | Procedural model authoring | Building bodies, geoms, joints, sub-spec attachments with prefixes, actuators (servos, motors), collision bitmasks (`contype`/`conaffinity`), default classes. |
| **`mujoco-rendering`** | [`rendering/`](rendering/SKILL.md) | Offscreen image rendering | Headless RGB, metric depth maps, segmentation masks, offscreen buffer sizing, camera positioning, C++ `mjrContext`/`mjvScene` pipelines. |
| **`mujoco-accelerated`** | [`accelerated/`](accelerated/SKILL.md) | GPU & TPU batch simulation | High-throughput batch rollouts with MJX (`jax.vmap`, `jax.lax.scan`), differentiability, CUDA batching with MJWarp, `njmax` contact allocation. |
| **`mujoco-gui`** | [`gui/`](gui/SKILL.md) | Dear ImGui & ImPlot UI design | Immediate-mode UI patterns, layout, styling, ID stack rules (`##` vs `###`), list virtualization (`ImGuiListClipper`), automated testing hooks. |
| **`mujoco-studio`** | [`studio/`](studio/SKILL.md) | Studio apps & interactive viewers | Multi-threaded Sim/Viewer decoupling, message channels (Snapshots vs Events), plugin architecture in C++ and Python, WebViewer WASM streaming. |

## How to Use These Skills

- **For AI Coding Agents**: Each skill contains standardized YAML frontmatter
  with `Use when:` triggers and `Don't use for:` exclusions to assist automated
  orchestrators in choosing the relevant guide.
- **For Developers**: Skills are self-contained guides providing architectural
  overviews, concrete `# Good:` vs `# Avoid:` examples, and common gotchas for
  building robust simulations and tooling.
