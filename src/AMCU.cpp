#include "AMCU.h"

#include "Motor.h"
// #include "Servo.h"
#include <limits> // 用于获取最小值

#include "as5600.h"
#define max_filament_num 16
arduino::MbedI2C AS5600_Wire(AMCU_AS5600_SDA, AMCU_AS5600_SCL);

//送料机存料检测IO
#define IO_PIN_1 21
#define IO_PIN_2 20
#define IO_PIN_3 19
#define IO_PIN_4 18
#define IO_PIN_KEY 24

#define LED_PIN_1 22
#define LED_PIN_2 26
#define LED_PIN_3 27
#define LED_PIN_4 28


#define LED_BLINK_INTERVAL 500 // LED 的闪烁间隔，单位为毫秒


#define SERVO_COUNT 4

// 舵机引脚定义
#define SERVO_PIN_1 13
#define SERVO_PIN_2 12
#define SERVO_PIN_3 11
#define SERVO_PIN_4 10

//送料轮直径
#define WHEEL_D 11

//舵机延迟时间
#define SERVO_DELAY 6000

//回抽距离
#define PULL_BACK_METER 200


// 全局变量，用于记录上次切换时间
uint32_t last_toggle_time = 0;

// 检查并切换 LED 状态
void handle_led_toggle(uint pin, uint32_t *last_toggle_time, uint32_t interval_ms) {
    // 获取当前绝对时间
    absolute_time_t absolute_time = get_absolute_time();
    uint32_t current_time = to_ms_since_boot(absolute_time);
    if (current_time - *last_toggle_time >= interval_ms) {
        gpio_put(pin, !gpio_get(pin)); // 切换 LED 电平状态
        *last_toggle_time = current_time; // 更新上次切换时间
    }
}

void init_io_pins() {
    pinMode(IO_PIN_1, INPUT);
    pinMode(IO_PIN_2, INPUT);
    // pinMode(IO_PIN_3, INPUT);
    // pinMode(IO_PIN_4, INPUT);
    gpio_pull_up(IO_PIN_1);
    gpio_pull_up(IO_PIN_2);
    // gpio_pull_up(IO_PIN_3);
    // gpio_pull_up(IO_PIN_4);

}

void init_led_pins() {
    // 初始化所有 LED 引脚为输出模式
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);
    pinMode(LED_PIN_4, OUTPUT);

    // 确保 LED 处于关闭状态
    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, LOW);
    digitalWrite(LED_PIN_3, LOW);
    digitalWrite(LED_PIN_4, LOW);
}


// double distance_count = 0;
bool if_as5600_init = false;
// bool AMCU_bus_need_to_waiting_slave[max_filament_num];
char last_num = -1; // 上一次的 current_num
unsigned long last_num_change_time = 0; // 记录上次 current_num 变化的时间
unsigned long last_pullback_time = 0; // 记录上次回抽开始的时间
bool execute_motion = true; // 控制是否执行动作
bool last_execute_motion=false;
int last_cmd=BambuBus_package_ERROR;
int last_filament_num = 255;
float last_meters[max_filament_num];
float last_pullback_meters[max_filament_num];
float last_action[max_filament_num];
_filament_motion_state_set act;
bool ams_enable=true;
int now_debug_num=0;
float target=0;

// Servo servos[SERVO_COUNT];
Motor mc(7,8); //构建电机对象
Motor mc_ext(IO_PIN_3, IO_PIN_4); //构建扩展电机对象(用于外接主电机驱动)

AS5600 as5600(&AS5600_Wire); //  use default Wire


void AS5600_init()
{
    AS5600_Wire.begin();

    as5600.begin(NC);                       // direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE); //  default, just be explicit.
    if_as5600_init = as5600.isConnected();
}
#define AS5600_PI 3.1415926535897932384626433832795

int32_t now_distance=0;
bool enable_as5600=true;
float AS5600_get_distance_E()
{
    if(!enable_as5600){
        return 0;
    }
    static int32_t last_distance = 0;
    int32_t cir_E = 0;
    now_distance = as5600.rawAngle();
    float distance_E;
    if ((now_distance > 3072) && (last_distance <= 1024))
    {
        cir_E = -4096;
    }
    else if ((now_distance <= 1024) && (last_distance > 3072))
    {
        cir_E = 4096;
    }

    distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * WHEEL_D / 4096; // D=19.3mm
    last_distance = now_distance;
    return distance_E;
}
// #define FILTER_ALPHA 0.2f // 滤波系数 (0 < FILTER_ALPHA <= 1.0)
// int32_t now_distance=0;
// float AS5600_get_distance_E() {
//     static int32_t last_distance = 0;
//     static float filtered_distance = 0.0f; // 上次滤波后的距离
//     int32_t cir_E = 0;
//     now_distance = as5600.rawAngle();
//     float distance_E;

//     // 处理角度跳跃
//     if ((now_distance > 3072) && (last_distance <= 1024)) {
//         cir_E = -4096;
//     } else if ((now_distance <= 1024) && (last_distance > 3072)) {
//         cir_E = 4096;
//     }

//     // 当前测量值计算
//     distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * WHEEL_D / 4096;
//     last_distance = now_distance;

//     // 使用指数加权滤波更新
//     filtered_distance = FILTER_ALPHA * distance_E + (1 - FILTER_ALPHA) * filtered_distance;

//     return -filtered_distance;
// }
#include "CRC.h"

/**
 * 初始化执行机
 */
void init_selector(){
    // 初始化所有 LED 引脚为输出模式
    pinMode(SERVO_PIN_1, OUTPUT);
    pinMode(SERVO_PIN_2, OUTPUT);
    pinMode(SERVO_PIN_3, OUTPUT);
    pinMode(SERVO_PIN_4, OUTPUT);

    // digitalWrite(SERVO_PIN_1, LOW);
    // digitalWrite(SERVO_PIN_2, LOW);
    // digitalWrite(SERVO_PIN_3, LOW);
    // digitalWrite(SERVO_PIN_4, LOW);
}

/**
 * 控制一路执行机
 */
void setSelectPos(char num,boolean state){
        // 控制舵机选择料槽
    pin_size_t pin;
    switch(num){
        case 0:
            pin=SERVO_PIN_1;
            break;
        case 1:
            pin=SERVO_PIN_2;
            break;
        case 2:
            pin=SERVO_PIN_3;
            break;
        case 3:
            pin=SERVO_PIN_4;        
            break;
        default:  
            return;
    }
    if(state){
        digitalWrite(pin, HIGH);
    }else{
        digitalWrite(pin, LOW);
    }
}


/**
 * 选择唯一一路执行机
 */
void selectOnePos(char num){
    for (int i = 0; i < SERVO_COUNT; i++) {
        if(i==num){
            setSelectPos(i,true);
        }else{
            setSelectPos(i,false);
        }
    }
    set_color(get_filament_color_R(num), get_filament_color_G(num), get_filament_color_B(num),get_filament_color_A(num));
    // rgb_set_breath(-1,500);
}

/**
 * 释放所有执行机
 */
void releaseAllPos(){
    for (int i = 0; i < SERVO_COUNT; i++) {
        setSelectPos(i,false);
    }
}
/*
void init_servos() {
    // 初始化舵机引脚
    servos[0].initIO(SERVO_PIN_1);
    servos[1].initIO(SERVO_PIN_2);
    servos[2].initIO(SERVO_PIN_3);
    servos[3].initIO(SERVO_PIN_4);
}


void switchServo(char num){
        // 控制舵机选择料槽
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i == num) {      

            // servos[i].setServoPosition(30); // 选中舵机转动30度
        } else {
            // servos[i].setServoPosition(0); // 其他舵机回到初始位置
        }
    }
}

void releaseAllServo(){
    for (int i = 0; i < SERVO_COUNT; i++) {
            servos[i].setServoPosition(0); // 其他舵机回到初始位置
    }
}
*/

/**
 * state 0=off,1=on,2=blink
 */
void set_LED_state(char led, char state) {
    pin_size_t led_pin;
    switch(led){
        case 0:
            led_pin=LED_PIN_1;
            break;
        case 1:
            led_pin=LED_PIN_2;
            break;
        case 2:
            led_pin=LED_PIN_3;
            break;
        case 3:
            led_pin=LED_PIN_4;        
            break;
        default:  
        return;
    }
    switch (state)
    {
    case 0:
        digitalWrite(led_pin, LOW);   // LED 灭        
        break;
    case 1:
        digitalWrite(led_pin, HIGH);   // LED 亮
        break;    
    case 2:
        handle_led_toggle(led_pin, &last_toggle_time, LED_BLINK_INTERVAL);
        break;
    default:
        break;
    }

}


void AMCU_bus_deal_set_motion_res(uint8_t *data, int data_length)
{
    // char ADR = data[1];
    uint8_t res = data[6];
    switch (res)
    {
    case 0x00:
        break;
    }
}

#include <Adafruit_NeoPixel.h>
#define PIN 23         // WS2812 数据引脚
#define NUMPIXELS 1   // WS2812 LED 数量
Adafruit_NeoPixel RGB_2812(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
u_int8_t brightness = 127;     // 亮度绝对值
void set_color(u_int8_t r, u_int8_t g, u_int8_t b,u_int8_t a)
{
    brightness = a;
    // RGB_2812.setBrightness(a);
    RGB_2812.setPixelColor(0, RGB_2812.Color(r, g, b)); 
    RGB_2812.show();
}

// #include <NeoPixelConnect.h>
// // #include "hardware/pio.h"
// // #define RGBLED_PIN 23         // WS2812 数据引脚
// #define RGBLED_PIN 15         // WS2812 数据引脚

// #define NUMPIXELS 1   // WS2812 LED 数量
// NeoPixelConnect RGB_2812(RGBLED_PIN, NUMPIXELS,pio1,1);

// void set_color(u_int8_t r, u_int8_t g, u_int8_t b)
// {

//     static u_int8_t last_r,last_g,last_b;
//     // static unsigned long last_update_time = 0; // 上次刷新时间
//     // unsigned long current_time = millis();    // 当前时间
//     if (r!=last_r || g!=last_g || b!=last_b 
//     // || (current_time - last_update_time > 100)
//     ) // 100ms 刷新间隔
//     {
//         // 临时禁用中断，确保数据一致性
//         // bool current_irq_level = irq_is_enabled(BambuBus_uart_IRQ);
//         // irq_set_enabled(BambuBus_uart_IRQ, false);

//         // uint32_t primask = __get_PRIMASK(); // 保存当前中断状态
//         // __disable_irq();                    // 禁用中断
//         // 设置 WS2812 颜色
//         RGB_2812.neoPixelSetValue(0, r, g, b, true);
//         // 恢复中断
//         // __set_PRIMASK(primask); // 恢复之前的中断状态
//         // irq_set_enabled(BambuBus_uart_IRQ, current_irq_level);
//         // char debug_message[64]; // 确保足够大的缓冲区存储拼接后的字符串
//         // sprintf(debug_message, "\r\nR=%d ,G=%d, B=%d \n",r,g,b);
//         // DEBUG(debug_message);       
//         // last_update_time = current_time;     // 更新上次刷新时间
//     }
// }

// NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> RGB_2812(1, 23); // note: modern WS2812 with letter like WS2812b
// NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> RGB_2812(1, 23); // note: modern WS2812 with letter like WS2812b
// void set_color(u_int8_t r, u_int8_t g, u_int8_t b)
// {
//     RGB_2812.SetPixelColor(0, RgbColor(r,g,b));
//     RGB_2812.Show(false);
// }

float brightness_scale = 1.0f;       // 当前亮度比例
float increment = 0.05f;       // 每次变化的步长（控制呼吸速度）
float breath_scale = 1.0f; // 速度比例
float breath_scale_flag = 0.0f; // 速度开始标记

uint16_t wait_count=2000;  //呼吸变化等待次数
int16_t breath_time=-1;  //呼吸变化次数
bool increment_dir=false;

void rgb_set_breath(int16_t time,uint16_t count){
    breath_time=time;
    wait_count=count;
    increment=0.9/wait_count;
}
/**
 * 加速闪烁speedup_count个1/2周期
 */
void rgb_breath_speedup(uint8_t speedup_count){
    breath_scale=20.0f;
    breath_scale_flag=(increment_dir?breath_scale:-breath_scale);
}

void proc_rgb_breath() {

        if(wait_count==0 || breath_time==0){//不呼吸
            // rgb_loop_counter=0;
            brightness_scale=1;
            RGB_2812.setBrightness(brightness);
            RGB_2812.show();
            return;
        }

        if(breath_time>0){
            breath_time-=1;
            // DEBUG("+");            
        }
        // 调整亮度
        if(increment_dir){
            brightness_scale += (increment*breath_scale);
        }else{
            brightness_scale -= (increment*breath_scale);
        }
        if((breath_scale_flag>0 && !increment_dir )||(breath_scale_flag<0 && increment_dir )) {
                breath_scale_flag=1.0f;
                breath_scale=1.0f;
        }

        if (brightness_scale >= 1.0f) {
            brightness_scale = 1.0f;
            increment_dir=false; 
        } else if (brightness_scale <= 0.1f) {
            brightness_scale = 0.1f;
            increment_dir=true;
        }
        if(wait_count%10!=0){//每10次真正调整一次
            return;
        }
        // 应用亮度变化到当前颜色
        RGB_2812.setBrightness(brightness*brightness_scale);
        RGB_2812.show();
}

void AMCU_init()
{
    // RGB_2812.begin();
    // init_rgb();
    //--初始化下位机串口
    //-- AMCU_bus_init();
    init_io_pins();    
    // init_servos();//使用舵机
    init_selector();//使用二元舵机
    char debug_msg[64]; 
    // gpio_set_function(RGBLED_PIN, GPIO_FUNC_SIO); // 明确设定为标准 GPIO 功能
    AS5600_init();
    if (if_as5600_init == false)
    {
        set_color(255,0,0,127);
        // RGB_2812.SetPixelColor(0, RgbColor(255, 0, 0));
        // RGB_2812.show();
        while(1);
    }
    BambuBus_init();
    mc.stop(); // 停止电机
    mc_ext.stop(); // 停止电机，外部
    releaseAllPos();//释放所有料槽
    delay(500);
    selectOnePos(get_now_filament_num());//选中当前料槽
    for (int i = 0; i < max_filament_num; i++) {
        last_meters[i] = std::numeric_limits<float>::lowest();
    }
    sprintf(debug_msg, "\r\n===inited,num:%d,mag:%+5d,mm:%+5.3f,l_i_p:%1d%1d%1d%1d-%1d%1d%1d%1d-%1d%1d%1d%1d===\r\n",
            get_now_filament_num(),
            now_distance,
            get_filament_meters(get_now_filament_num()),
            digitalRead(LED_PIN_1) ? 1 : 0,
            digitalRead(LED_PIN_2) ? 1 : 0,
            digitalRead(LED_PIN_3) ? 1 : 0,
            digitalRead(LED_PIN_4) ? 1 : 0,

            digitalRead(IO_PIN_1) ? 1 : 0,
            digitalRead(IO_PIN_2) ? 1 : 0,
            digitalRead(IO_PIN_3) ? 1 : 0,
            digitalRead(IO_PIN_4) ? 1 : 0,

            digitalRead(SERVO_PIN_1) ? 1 : 0,
            digitalRead(SERVO_PIN_2) ? 1 : 0,
            digitalRead(SERVO_PIN_3) ? 1 : 0,
            digitalRead(SERVO_PIN_4) ? 1 : 0
            );
    DEBUG(debug_msg);   
}

void debug_info(bool force)
{
    bool force_show = force;
    int t_cmd = get_cmd_type();
    int num = get_now_filament_num();
    if (!ams_enable)
    {
        num = now_debug_num;
    }

    float meter = get_filament_meters(num);
    if (meter - last_meters[num] > 1 || meter - last_meters[num] < -1 // 变化1mm
    || last_meters[num] == std::numeric_limits<float>::lowest())
    {
        rgb_breath_speedup(2);
        last_meters[num] = meter;
        force_show = true;
    }

    if (!force_show && t_cmd == last_cmd && num == last_filament_num && last_execute_motion == execute_motion)
    {
        // DEBUG(",");
        return;
    }

    if (!force_show && (true
                        // t_cmd == BambuBus_package_filament_motion_short
                        // || t_cmd == BambuBus_package_filament_motion_long
                        || t_cmd == BambuBus_package_online_detect || t_cmd == BambuBus_package_REQx6 || t_cmd == BambuBus_package_NFC_detect
                        // || t_cmd == BambuBus_package_set_filament
                        || t_cmd == BambuBus_long_package_MC_online
                        // || t_cmd == BambuBus_longe_package_filament
                        || t_cmd == BambuBus_long_package_version ||
                         t_cmd == BambuBus_package_heartbeat || 
                         t_cmd == BambuBus_package_ETC))
    {
        // DEBUG(".");
        return;
    }
    ////
    last_cmd = t_cmd;
    last_filament_num = num;
    last_execute_motion = execute_motion;

    char debug_message[128]; // 确保足够大的缓冲区存储拼接后的字符串
    sprintf(debug_message, "\r\nmo:%d,en:%d,pos:%d,opos:%3d,cmd:%2d,mag:%+5d,mm:%+5.3f,o_mm:%+5.3f,l_i_p:%1d%1d%1d%1d-%1d%1d%1d%1d-%1d%1d%1d%1d",
            get_filament_motion(num),
            execute_motion ? 1 : 0,
            num,
            get_now_op_num(),
            get_cmd_type(),
            now_distance,
            meter,
            last_pullback_meters[num],
            digitalRead(LED_PIN_1) ? 1 : 0,
            digitalRead(LED_PIN_2) ? 1 : 0,
            digitalRead(LED_PIN_3) ? 1 : 0,
            digitalRead(LED_PIN_4) ? 1 : 0,

            digitalRead(IO_PIN_1) ? 1 : 0,
            digitalRead(IO_PIN_2) ? 1 : 0,
            digitalRead(IO_PIN_3) ? 1 : 0,
            digitalRead(IO_PIN_4) ? 1 : 0,

            digitalRead(SERVO_PIN_1) ? 1 : 0,
            digitalRead(SERVO_PIN_2) ? 1 : 0,
            digitalRead(SERVO_PIN_3) ? 1 : 0,
            digitalRead(SERVO_PIN_4) ? 1 : 0);
    DEBUG(debug_message);
}

#ifdef __cplusplus
extern "C"
{
#include "pico/bootrom.h"
}
#endif

void process_usb_command(const std::string &cmd) {
    if (cmd.rfind("|", 0) == 0) { // 检查是否以 "|||" 开头
        act=cancel;
        ams_enable=false;
        DEBUG("Stop\n");
    } else if (cmd.rfind("/", 0) == 0) { // 
        act=waiting;
        ams_enable=true;
        enable_as5600=true;  
        DEBUG("Run\n");
    } else if (cmd.rfind("s-", 0) == 0) { // 
        // target = std::atoi(cmd.substr(1).c_str()); 
        act=release_all;
        ams_enable=false;
        DEBUG("S-\n");
    } else if (cmd.rfind(">", 0) == 0) { // 检查是否以 ">" 开头
        last_pullback_meters[now_debug_num]=get_filament_meters(now_debug_num);
        target = std::atoi(cmd.substr(1).c_str()); 
        act=act_pull_mm;
        // ams_enable=false;
        DEBUG("Pull\n");
    } else if (cmd.rfind("<", 0) == 0) { // 检查是否以 "<" 开头
        last_pullback_meters[now_debug_num]=get_filament_meters(now_debug_num);
        target = std::atoi(cmd.substr(1).c_str()); 
        act=act_send_mm;
        // ams_enable=false;
        DEBUG("Send\n");
    } else if (cmd.rfind("]", 0) == 0) { // 检查是否以 "]" 开头
        last_pullback_meters[now_debug_num]=get_filament_meters(now_debug_num);
        target = std::atoi(cmd.substr(1).c_str()); 
        add_filament_meters(now_debug_num, target);
        DEBUG("f_pull\n");
    } else if (cmd.rfind("[", 0) == 0) { // 检查是否以 "[" 开头
        last_pullback_meters[now_debug_num]=get_filament_meters(now_debug_num);
        target = std::atoi(cmd.substr(1).c_str()); 
        add_filament_meters(now_debug_num, target);
        DEBUG("f_send\n");
    } else if (cmd == "s1") {
        act=select_pos;
        target=0;
        now_debug_num=0;
        ams_enable=false;
        DEBUG("s1\n");
    } else if (cmd == "s2") {
        act=select_pos;
        target=1;
        ams_enable=false;
        now_debug_num=1;
        DEBUG("s2\n");
    } else if (cmd == "s3") {
        act=select_pos;
        target=2;
        now_debug_num=2;
        ams_enable=false;
        DEBUG("s3\n");
    } else if (cmd == "s4") {
        act=select_pos;
        target=3;
        now_debug_num=3;
        ams_enable=false;
        DEBUG("s4\n");
    } else if (cmd == "i") {
        // uint8_t b[4]; //AMS-buffer
        // b[0]=0;
        // b[1]=1;
        // b[2]=2;
        // b[3]=3;
        // send_uart(b,4);
        // DEBUG("Info command\n");
    } else if (cmd == "r"){
        set_color(255,0,0,127);
    } else if (cmd == "g"){
        set_color(0,255,0,127);
    } else if (cmd == "b"){
        set_color(0,0,255,127);
    } else if (cmd == "reboot"){
        DEBUG("reboot..\n");
        reset_usb_boot(0, 0); // This reboots the Pico into BOOTSEL mode     
    } else if (cmd == "magoff"){
        DEBUG("disable as5600.\n");
        enable_as5600=false;   
    } else {
        DEBUG("err_cmd\n");
    }
    debug_info(true);
}

void run_debug() {
        if (USB_debug_has_cmd()) { // 检查是否有新命令
            const char* cmd = USB_debug_get_cmd(); // 获取命令
            if (cmd != NULL) {
                // 调用命令处理函数
                process_usb_command(cmd);
            }
        }
}

void AMCU_run()
{
    bool if_count_meters = true;
    float distance_E = AS5600_get_distance_E();
    static int now_filament_num = 255;
    int x = get_now_filament_num();
    if (now_filament_num != x)
    {
        now_filament_num = x;
        if_count_meters = false;
        Bambubus_set_need_to_save();
        //reset_filament_meters(now_filament_num);
    }

    // for (int i = 0; i < max_filament_num; i++)
    // {
    //     if ((AMCU_bus_need_to_waiting_slave[i] == true) && (i != now_filament_num))
    //     {
    //         if_count_meters = false;
    //         break;
    //     }
    // }
    if(!ams_enable){
        now_filament_num=now_debug_num;
    }

    switch (get_filament_motion(now_filament_num))
    {
    case need_pull_back:
        if(last_action[now_filament_num]!=need_pull_back){
            last_action[now_filament_num]=need_pull_back;
            last_pullback_meters[now_filament_num]=get_filament_meters(now_filament_num);
        }
        break;
    case need_send_out://送出时，不计入里程
        last_action[now_filament_num]=need_send_out;
        if_count_meters = false;
        break;
    case waiting:
        last_action[now_filament_num]=waiting;
        break;
    }

    if (if_count_meters || ams_enable==false){
        add_filament_meters(now_filament_num, distance_E);
    }
    // debug_send_run();

    int stu=-1;
    if(ams_enable){
        stu=BambuBus_run();
    }
    if(stu==-1)
    {
        // RGB_2812.SetPixelColor(0, RgbColor(127, 0, 127));
    }else{
        // RGB_2812.SetPixelColor(0, RgbColor(get_filament_color_R(now_filament_num), get_filament_color_G(now_filament_num), get_filament_color_B(now_filament_num)));
        // RGB_2812.Show(false);
        set_color(get_filament_color_R(now_filament_num), get_filament_color_G(now_filament_num), get_filament_color_B(now_filament_num),get_filament_color_A(now_filament_num));
        rgb_set_breath(-1,2000);
    }

    //--下位机运行
    // AMCU_bus_run();
    run_debug();
    AMCU_motion();
    debug_info(false);

    // 检查IO引脚状态
    if (digitalRead(IO_PIN_1) == LOW && now_filament_num == 0) {
        set_filament_online(0, offline);
        DEBUG("Down 1 \n");      
        //set_LED_state(0,0);
    }else{
        set_filament_online(0, online);    
        //set_LED_state(0,1);
    }
    if (digitalRead(IO_PIN_1) == LOW && now_filament_num == 1) {
        set_filament_online(1, offline);
        DEBUG("Down 2 \n");      
        //set_LED_state(1,0);
    }else{
        set_filament_online(1, online);     
        //set_LED_state(1,1);
    }
    if (digitalRead(IO_PIN_1) == LOW && now_filament_num == 2) {
        set_filament_online(2, offline);
        DEBUG("Down 3 \n");      
        //set_LED_state(2,0);
    }else{
        set_filament_online(2, online);     
        //set_LED_state(2,1);
    }
    if (digitalRead(IO_PIN_1) == LOW && now_filament_num == 3) {
        set_filament_online(3, offline);
        DEBUG("Down 4 \n");      
        //set_LED_state(3,0);
    }else{
        set_filament_online(3, online);     
        //set_LED_state(3,1);
    }

    if (digitalRead(IO_PIN_KEY) == HIGH){
        //reset_filament_meters(now_filament_num);
    }

    // if (digitalRead(IO_PIN_KEY) == HIGH){
    //     RGB_2812.SetPixelColor(0, RgbColor(127, 0, 127)); 
    //     RGB_2812.Show();
    // }else{
    //     RGB_2812.SetPixelColor(0, RgbColor(0, 127, 0)); 
    //     RGB_2812.Show();
    // }
    proc_rgb_breath();
}

void AMCU_motion()
{
    static int pull_end_cnt=0; //回抽结束后，循环次数
    char current_num = get_now_filament_num();    
     if(!ams_enable){//手工模式
        current_num=now_debug_num;
    }
    if(ams_enable){//ams模式
        selectOnePos(current_num);
    }
    // 检查 current_num 是否变化
    if (current_num != last_num) {
        last_num = current_num;
        last_num_change_time = millis(); // 更新时间戳
        execute_motion = false;      // 重置动作执行标志位
    }
    // 检查等待时间
    if (!execute_motion && millis() - last_num_change_time >= SERVO_DELAY) {
        execute_motion = true; // 允许执行动作
    }

    // if(millis() - last_pullback_time >=30000){//回抽超过30秒，异常
    //      set_filament_motion(current_num,waiting);
    // }

    float meter=get_filament_meters(current_num);    
    if(ams_enable){
        act=get_filament_motion(current_num);
    }
    switch (act)
    {
    case need_pull_back:
        // FilamentState = "busy";
        // DEBUG("AMCU_motion selectOnePos\r\n");
        set_LED_state(0,1);            
        if(meter-last_pullback_meters[current_num]> PULL_BACK_METER  ){
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            if(pull_end_cnt>0){        
                pull_end_cnt--;    
                DEBUG("]");    
            }
            if(pull_end_cnt==0){
                pull_end_cnt=-1;
                DEBUG("\n]]...\n");      
            }
        }else if (execute_motion && is_filament_online(current_num)) {
            // DEBUG("AMCU_motion backforward\r\n");
            pull_end_cnt=20;
            mc.backforward(); // 退料控制DC电机反转
            mc_ext.backforward(); // 退料控制DC电机反转
            DEBUG(">");
            // DEBUG("back....");
        }else{
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            DEBUG(")");
        }
        // debug_info();
        // RGB_2812.SetPixelColor(0,RgbColor(0, 127, 0));
        // set_color(0,127,0);
        break;
        // motion = AMCU_bus_motion_need_pull_back;
        // break;
    case need_send_out:
        // FilamentState = "busy";
        set_LED_state(1,1);   
        if (execute_motion && is_filament_online(current_num)) {
            // DEBUG("AMCU_motion forward\r\n");
            // DEBUG("send....");
            mc.forward(); // 送料控制DC电机正转
            mc_ext.forward(); // 送料控制DC电机正转
            DEBUG("<");
        }else{
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            DEBUG("(");
        }
        //last_pullback_meters[current_num]=meter;         
        // debug_info();
        // RGB_2812.SetPixelColor(0,RgbColor(0, 0, 127));
        // set_color(0,0,127);
        // motion = AMCU_bus_motion_need_send_out;
        break;
    case waiting:
        // DEBUG("waiting....");
        mc.stop(); // 停止电机
        mc_ext.stop(); // 停止电机
        // last_pullback_meters[current_num]=meter;         
        // DEBUG("AMCU_motion stop\r\n");
        set_LED_state(0,0);
        set_LED_state(1,0);
        // debug_info();
        // releaseAllPos();
            // FilamentState = "exist";
        break;
    case act_send_mm:
        // FilamentState = "busy";
        set_LED_state(1,1);   
        if(last_pullback_meters[current_num]-meter>target){
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            act=waiting;
            DEBUG("[-");
            // ams_enable=true;
        }else if (execute_motion && is_filament_online(current_num)) {
            // DEBUG("AMCU_motion forward\r\n");
            // DEBUG("send....");
            mc.forward(); // 送料控制DC电机反转
            mc_ext.forward(); // 送料控制DC电机反转
            DEBUG("<-");
        }else{
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            DEBUG("(-");
        }
        break;
    case select_pos:
        selectOnePos(target);
        break;
    case act_pull_mm:
        set_LED_state(0,1);            
        if(meter-last_pullback_meters[current_num]>target){
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            act=waiting;
            DEBUG("-]");
            // ams_enable=true;
        }else if (execute_motion && is_filament_online(current_num)) {
            // DEBUG("AMCU_motion backforward\r\n");
            mc.backforward(); // 退料控制DC电机反转
            mc_ext.backforward(); // 退料控制DC电机反转
            DEBUG("->");
            // DEBUG("back....");
        }else{
            mc.stop(); // 停止电机
            mc_ext.stop(); // 停止电机
            set_LED_state(0,0);
            set_LED_state(1,0);
            DEBUG("-)");
        }
        // debug_info();
        // RGB_2812.SetPixelColor(0,RgbColor(0, 127, 0));
        // set_color(0,127,0);
        break;
        // motion = AMCU_bus_motion_need_pull_back;
        // break;
    case cancel:
        mc.stop(); // 停止电机
        mc_ext.stop(); // 停止电机
        set_LED_state(0,0);
        set_LED_state(1,0);
        // ams_enable=true;
        act=waiting;
        break;        
    case release_all:
        mc.stop(); // 停止电机
        mc_ext.stop(); // 停止电机
        set_LED_state(0,0);
        set_LED_state(1,0);
        releaseAllPos();
        // ams_enable=true;
        act=waiting;
        break;        
    default:
        break;
        // if (last_motion == AMCU_bus_motion_need_pull_back)
        //     motion = AMCU_bus_motion_end_pull_back;
        // else if (last_motion == AMCU_bus_motion_need_send_out)
        //     motion = AMCU_bus_motion_end_send_out;
        // else
        //     motion = last_motion;
        // break;
    }
    // last_motion = motion;

}
