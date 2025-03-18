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