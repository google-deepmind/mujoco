Humanoid
========

Degrees of Freedom: 27
Actuators: 21

This is a clone of the [MuJoCo Humanoid](https://github.com/google-deepmind/mujoco/blob/main/model/humanoid/humanoid.xml)
with the following changes:

* Solver switched to CG with 8 iterations
* Explicit contact pairs for feet and ground (compatible with
  [OpenAI Gym Humanoid](https://gymnasium.farama.org/environments/mujoco/humanoid/)
  environment)

This simplified humanoid model, introduced in [1], is designed for bipedal
locomotion behaviours. While several variants of it exist in the wild, this
version is based on the model in the DeepMind Control Suite [2], which has
fairly realistic actuator gains.

[1] [Synthesis and Stabilization of Complex Behaviors through Online Trajectory Optimization]
     (https://doi.org/10.1109/IROS.2012.6386025).

[2] [DeepMind Control Suite](https://arxiv.org/abs/1801.00690).


![humanoid](humanoid.png)
