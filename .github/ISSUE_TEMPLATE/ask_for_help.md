---
name: Asking for help
about: Request help from the developers and the community
title: ''
labels: question
assignees: ''

---

**How to ask for help**

First, read our quick guide to
[asking good questions](https://github.com/google-deepmind/mujoco#asking-questions).
Below is a template for you to use:

Hi,

I'm a (student / professor / engineer) and I'm trying to use MuJoCo for _____.

I'm looking for some help with ____.

Here is a model which explains my question:

<details>
  <summary>minimal XML</summary>

```XML
<mujoco>
  <worldbody>
    <light pos=".4 -.4 .3" dir="-2 2 -1.5" diffuse=".6 .6 .6"/>
    <light pos="-.2 -.4 .3" dir="1 2 -1.5" diffuse=".6 .6 .6"/>
    <geom type="plane" size=".5 .5 .01"/>
    <body name="ball" pos="0 0 0.05">
      <freejoint/>
      <geom size=".03"/>
    </body>
    <body name="hammer" pos="-.05 0 0.18">
      <joint axis="0 1 0"/>
      <geom type="capsule" size=".01" fromto="0 0 0 -.15 0 0"/>
      <geom type="box" size=".02 .02 .03" pos="-.15 0 0"/>
    </body>
  </worldbody>
</mujoco>
```

</details>

Here is a screenshot / video, illustrating my question:
