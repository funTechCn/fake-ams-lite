# fake-ams-lite 
diy bambu ams lite for A1

低成本的diy自动换色系统，适用于拓竹A1。
演示视频可以在B站搜索“不到100块钱手搓3D打印机多色换料系统”
电路板和机械结构会上传到嘉立创开源和makerworld


代码来源于 https://github.com/applenana/AP-AMS ，对逻辑进行了调整，以适配低成本主板

因为网购的树莓派pico 模块规格各异，如果io不一致，请注意调节io，比如rgb led的io

当前把送料机存料检测的两个IO用作了扩展电机控制，需要根据实际情况调整
```
#define IO_PIN_3 19
#define IO_PIN_4 18

Motor mc(7,8); //构建电机对象
Motor mc_ext(IO_PIN_3, IO_PIN_4); //构建扩展电机对象(用于外接主电机驱动)
```
送料机存料检测功能没有进行测试


通过USB线连接控制板后，可以通过串口调试：
```
| 暂停执行ams指令，会导致打印机和ams断连
/ 恢复执行ams指令
s- 释放所有执行机
>XXX 回抽XXXmm，针对当前选中的料
<XXX 送出XXXmm，针对当前选中的料
]XXX 直接给料线计数回抽方向增加XXXmm,未测试
[-XXX 直接给料线计数进料方向增加XXXmm,未测试
s1 s2 s3 s4 切换到料线1 2 3 4
i 打印调试信息
```

注意，执行手工命令后，会进入|暂停状态，需要手工/恢复执行

本代码为学习用途，不得用于商业用途。



-------------------------------------------------------------------------------------



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
