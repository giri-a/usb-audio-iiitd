| Supported Targets | ESP32-S3 |
| ----------------- | ---------|

# _USB Audio dongle (for IIIT-Dharwad workshop)_

This project implements a 2-ch USB audio device for playback and recording functions. 
I2S amplifier (for playback) and I2S Mic (for recording) are interfaced with the GPIO pins
configured as BCLK, WS, DIN, DOUT.

It was started based on https://github.com/espressif/esp-skainet/tree/master/examples/usb_mic_recorder
(I found the pointer in a discussion on the thread https://github.com/espressif/esp-idf/issues/12774).

While working on some issues, I came across https://github.com/espressif/esp-box/tree/master/examples/usb_headset 
and incorporated some codes from that.

I started with Knowles SPH0645 I2S MEMS mics, which has its own specific requirements. One important
one is that it seems to have large offset, which needed a highpass filter to correct.
Those mics broke and now I use INMP441 mics from Invensense. These do not require the offset cancellation, but
vestiges of the previous code linger on. 

INMP441 seems to have a noise issue at 16kHz, which disappers at 24kHz or higher.

The data format is fixed at 16bits and the sampling frequency is configurable (16kHz, 32kHz and 24kHz).
Mute and volume control functions are implemented.

Most of the development was done on Mac. Recently when I started testing on Windows, I came across a few issues:
- Windows driver does not support a master channel (only L and R) for volume control. 
- Controlling volume requires one to go through a few hops (COntrol Panel->Hardware->Sound ...)
- I have not found a single relatively simple application so far that does playback and recording in one application. So operations like looping back are not allowed because it requires two programs to have access to the driver. 
- For whatever reason, Windows driver does not like 24kHz sampling frequency, so it does not even offer it.
- Windows native 'voice recorder' application does not work with 32kHz. So I am stuck with 16kHz only.

## Building 
- This was built using VSCode using ESP-IDF 5.4.0 extension. 
- It has dependency on two managed components : espressif__tinyusb (0.15.0~9) and espressif__led_strip. Previous versions of espressif__tinyusb has a bug related to opening the device connections multiple times.

## Folder contents

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── idf_components.yml
│   ├── Kconfig.projbuild
│   ├── main.c
│   ├── src
│   │    ├── i2s_functions.c
│   │    ├── blink_led.c
│   │    ├── uad_callbacks.c
│   │    |── usb_descriptors.c
|   |    └── utilities.c
│   └── include
│        ├── tusb_config.h
│        ├── usb_descriptors.h
│        ├── i2s_functions.h
│        ├── blink.h
|        |── data_buffers.h
|        |── utilities.h
│        └── gain_table.h
├
└── README.md                  This is the file you are currently reading
```
## settings.json

I work on Mac as well as Windows10 machine and use vscode to develop this code.
Right settings.json is required to seamlessly move between the platforms.
See https://code.visualstudio.com/docs/getstarted/settings#_settings-precedence

The settings for ESP-IDF containing ESP_IDF and ESP_IDF_TOOLS paths and python
environment should be local to the platform specific installations (therefore
not part of this repo).

Only certain things like the idfTarget, idfPort etc. are project specific and
that should be stored in .vscode/settings.json and is part of the repo. 

Note that everytime ESP-IDF is configured, it updates .vscode/seetings.json with the platform-specific settings.

.vscode/c_cpp_properties.json is manually edited to include two configurations with names ESP-IDF(Mac) and ESP-IDF(Win). Configuring through command panel, will create one with platform specific paths; manual edit was required to combine the two under different names. Based on the platform you are working on, choose the appropriate configuration by clicking on the configuration (at the very right end) on the status bar of vscode.

## CMake
Sometimes the settings get all messed up and/or cmake makes it so. It cannot find IDF_PATH variable. For this reason and also to be able to run idf.py from command line it is advisable to add the tools path to system PATH variable. The instructions are at this link.
https://docs.espressif.com/projects/esp-idf/en/v3.3/get-started-cmake/add-idf_path-to-profile.html

However, it seems that I ran into this problem since I clicked on a prompt in VSCODE to run cmake at every startup. Once I disabled it in settings, the build seemed to happen just fine. 