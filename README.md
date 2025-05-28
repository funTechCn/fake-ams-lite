# fake-ams-lite

DIY Bambu AMS Lite for A1

A low-cost DIY automatic filament changing system, compatible with Bambu A1.
You can find a demo video on Bilibili by searching:
**"不到100块钱手搓3D打印机多色换料系统"** (Multicolor filament changing system for 3D printer made under 100 RMB).
Circuit board and mechanical designs will be uploaded to **JLC Open Source** and **MakerWorld**.

The code is based on [https://github.com/applenana/AP-AMS](https://github.com/applenana/AP-AMS), with logical modifications to support a low-cost control board.

Due to variations in Raspberry Pi Pico modules bought online, the pin layout might differ. Please adjust pin assignments accordingly, such as for the RGB LED.

Currently, two IO pins originally used for filament detection are repurposed to control an extra motor. Adjust them based on your setup:

```cpp
#define IO_PIN_3 19
#define IO_PIN_4 18

Motor mc(7,8); // Main motor object
Motor mc_ext(IO_PIN_3, IO_PIN_4); // External motor object (for external main motor driver)
```

Note: The filament presence detection feature of the feeder has not been tested.

Once connected to the control board via USB, you can debug through the serial terminal:

```
|  Pause AMS command execution — will cause disconnect with printer and AMS
/  Resume AMS command execution
s- Release all actuators
>XXX Retract XXX mm of filament (for currently selected line)
<XXX Feed XXX mm of filament (for currently selected line)
]XXX Increment filament counter by XXX mm in retract direction (untested)
[-XXX Increment filament counter by XXX mm in feed direction (untested)
s1 s2 s3 s4 Switch to filament line 1, 2, 3, or 4
i Print debug information
```

**Note:** After executing manual commands, the system will enter `|` (pause) state. You must manually resume with `/`.

This code is for educational use only and **must not be used for commercial purposes**.
