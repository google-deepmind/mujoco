# Simulate XR

Simulate XR is an extension of `simulate` that renders the scene for VR/XR/AR headsets.


## Installation

To build simulate with XR, switch the option in [simulate CMakeLists](/simulate/CMakeLists.txt#L56) to ON, then configure, build and install normally. I prefer to keep it in a separate location from usual build and install, because launching with built-in XR takes a bit longer. The XR capability is also fully integrated into `libsimulate`, so should be usable from the loaded library or python bindings (untested).


## Running

1. Connect the headset to the computer and to the SteamVR. For non-native integration with SteamVR see [Supported devices](#supported-devices).
2. Run simulate as you normally do.


## Used libraries

The code currently uses [OpenXr](https://www.khronos.org/OpenXR/), and generally works through [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) Runtime.


## Supported systems

* Windows 10/11

In progress:

* OS X
* Unix


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

* Unix/OS X support.
* Specify spawn location in the model file field.
* Adding controllers.
* Controller maps to keys.
* Compile MuJoCo on the XR headsets themselves.

