# Simulate XR

Simulate XR is an extension of `simulate` that renders the scene for VR/XR/AR headsets.


## Installation

To build simulate with XR, switch the `SIMULATE_BUILD_XR` option in [simulate CMakeLists](/simulate/CMakeLists.txt#L56) to ON, then configure, build and install normally. I prefer to keep it in a separate location from usual build and install, because launching with built-in XR takes a bit longer. The XR capability is also fully integrated into `libsimulate`, so should be usable from the loaded library or python bindings (untested).


## Running

1. Connect the headset to the computer and to the SteamVR. For non-native integration with SteamVR see [Supported devices](#supported-devices).
2. Run simulate as you normally do.


## Used libraries

The code currently uses [OpenXr](https://www.khronos.org/OpenXR/), and generally works through [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) Runtime.


## Supported systems

* Windows 10/11

In progress:

* OS X
    * For screen sharing with Quest 3, Immersed app works.
    * One could try developing for XR using [Meta Xr Simulator](https://github.com/Oculus-VR/homebrew-repo/blob/main/meta-xr-simulator.md).
    * OpenXR runtime Monado is in progress of [porting to MacOS](https://gitlab.freedesktop.org/monado/monado/-/issues/318), however no specific deadline exists.
    * Windows VM + SteamVR is an [option](https://www.youtube.com/watch?v=Wzk3nBWMKL8), however the performance will suffer.
    * Probably is not happening anytime soon, however will work on building on Mac to deploy to Quest3 and Vive XR.

* Unix
    * Use [Monado](https://github.com/shinyquagsire23/monado)
    * https://github.com/shinyquagsire23/openxr-simple-example-macos


## Supported devices

* Valve Index
* HTC Vive XR Elite
    - Works when turning on [Vive Streaming Hub](https://www.vive.com/us/solution/vive-streaming/).
    - Need to be on same Wi-Fi as the PC running `simulate` or connect to it using a USB-C cable.
* Meta Quest 2 - should be working, but have not tested recently.
* Meta Quest 3 - works through [Steam Link](https://www.meta.com/experiences/5841245619310585/).

Future plans:
* Apple Vision Pro. Might be able to stream through [ALVR](https://github.com/alvr-org/ALVR), although it is unknown if that will be available for free, and currently only available for small group of people.


## Integration into your own code

To expand or integrate the XR code, you will need a copy of the files located in this folder. You can find all places where it is integrated into `simulate` by looking for `mjBUILDSIMULATEXR` identifier.


## Known issues

* Connection over WiFi might not work on corporate network. Talk to your IT.
* Making the window very narrow (~1:4) makes the visuals flip. Possibly to do with how buffer blitting works.


## Current priorities

* Adding controllers. - check the dev/controllers-resync branch
* Unix/OS X support. Low priority, the systems do not easily support OpenXR.
* Specify spawn location in the model file field.
* Compile MuJoCo on the XR headsets themselves.
* Expose the XR rendering parameters, like the projected region for the user.
