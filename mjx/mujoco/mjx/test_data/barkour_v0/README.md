# Google Barkour v0

## MJCF Instructions

The MuJoCo config in `assets/barkour_v0_mjx.xml` was copied from https://github.com/deepmind/mujoco_menagerie/google_barkour_v0. The following edits were made to the MJCF specifically for brax:

* `meshdir` was changed from `assets` to `.`.
* `frictionloss` was removed. `damping` was changed to 0.5239.
* A custom `init_qpos` was added.
* A sphere geom `lowerLegFoot` was added to all feet. All other contacts were turned off.
* The compiler option was changed to `<option timestep="0.002" iterations="4" solver="CG"/>`.
* Non-visual geoms were removed from the torso, to speed up rendering.

## Publications

If you use this work in an academic context, please cite the following publication:

    @misc{caluwaerts2023barkour,
          title={Barkour: Benchmarking Animal-level Agility with Quadruped Robots},
          author={Ken Caluwaerts and Atil Iscen and J. Chase Kew and Wenhao Yu and Tingnan Zhang and Daniel Freeman and Kuang-Huei Lee and Lisa Lee and Stefano Saliceti and Vincent Zhuang and Nathan Batchelor and Steven Bohez and Federico Casarini and Jose Enrique Chen and Omar Cortes and Erwin Coumans and Adil Dostmohamed and Gabriel Dulac-Arnold and Alejandro Escontrela and Erik Frey and Roland Hafner and Deepali Jain and Bauyrjan Jyenis and Yuheng Kuang and Edward Lee and Linda Luu and Ofir Nachum and Ken Oslund and Jason Powell and Diego Reyes and Francesco Romano and Feresteh Sadeghi and Ron Sloat and Baruch Tabanpour and Daniel Zheng and Michael Neunert and Raia Hadsell and Nicolas Heess and Francesco Nori and Jeff Seto and Carolina Parada and Vikas Sindhwani and Vincent Vanhoucke and Jie Tan},
          year={2023},
          eprint={2305.14654},
          archivePrefix={arXiv},
          primaryClass={cs.RO}
    }
