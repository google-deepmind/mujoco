# Simulate VR

Simulate VR is an extension of `simulate` that renders the scene for VR/XR/AR headsets.

## Installation

To build simulate with VR, switch the option in [simulate CMakeLists](/simulate/CMakeLists.txt#L51) to ON, then configure, build and install normally. I prefer to keep it in a separate location from usual build and install, because launching with built-in VR takes a bit longer. The VR capability is also fully integrated into `libsimulate`, so should be usable from the loaded library or python bindings (untested).

If the model contains a custom numeric variable starting with `VR_`, it will be used as an origin point/orientation for the headset. For example, see [balloons](/model/balloons/balloons.xml#L108-L117).

## Running

1. Connect the headset to the computer and to the SteamVR. For non-native integration with SteamVR see [Supported devices](#supported-devices).
2. Run simulate as you normally do.

## Used libraries

The code currently uses [OpenVr](https://github.com/ValveSoftware/openvr), and generally works through [SteamVR](https://store.steampowered.com/app/250820/SteamVR/).

## Supported systems

* Windows 10/11

In progress:

* Unix - need to figure out how to link to Unix OpenVR using cmake.
* macOS

## Supported devices

* Valve Index
* HTC Vive XR Elite
    - Works when turning on [Vive Streaming Hub](https://www.vive.com/us/solution/vive-streaming/).
    - Need to be on same Wi-Fi as the PC running `simulate` or connect to it using a USB-C cable.
* Meta Quest 2 - should be working, but have not tested recently.

In progress:
* Meta Quest 3 - technically works (through [Steam Link](https://www.meta.com/experiences/5841245619310585/)), but the projections are wrong, will need fixing.
* Apple Vision Pro. Might be able to stream through [ALVR](https://github.com/alvr-org/ALVR).


## Integration

To expand or integrate the VR code, you will need a copy of the files located in this folder. You can find all places where it is integrated into `simulate` by looking for `mjBUILDSIMULATEVR` identifier.

## Current priorities

* Fixing Meta Quest 3 projections.
* Switching between different origin points in the scene, similar to switching between cameras.
* Adding UI tabs to the VR interface.
* Unix/macOS support.
* Adding controllers.

## Future plans

* Move from OpenVR to [OpenXR](https://github.com/KhronosGroup/OpenXR-SDK).
* Run MuJoCo on the XR headsets themselves.
