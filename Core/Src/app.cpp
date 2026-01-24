/*
 * main.cpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#include "HardwareConfig.hpp"
#include "usbd_customhid.h"
#include "usb_device.h"
#include "ZeroTracker.hpp"

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct
{
  union {
    struct {
      uint32_t button1  : 1;
      uint32_t button2  : 1;
      uint32_t button3  : 1;
      uint32_t button4  : 1;
      uint32_t button5  : 1;
      uint32_t button6  : 1;
      uint32_t button7  : 1;
      uint32_t button8  : 1;
      uint32_t button9  : 1;
      uint32_t button10 : 1;
      uint32_t button11 : 1;
      uint32_t button12 : 1;
      uint32_t button13 : 1;
      uint32_t button14 : 1;
      uint32_t button15 : 1;
      uint32_t button16 : 1;
      uint32_t button17 : 1;
      uint32_t button18 : 1;
      uint32_t button19 : 1;
      uint32_t button20 : 1;
      uint32_t button21 : 1;
      uint32_t button22 : 1;
      uint32_t button23 : 1;
      uint32_t button24 : 1;
      uint32_t padding  : 8;  // Padding to align to 32 bits
    } bits;
    uint32_t all_buttons;      // Access all buttons as a single uint32_t
  } buttons;

  int16_t x;      // Axis X (-32768 to 32767)
  int16_t y;      // Axis Y (-32768 to 32767)
  int16_t z;      // Axis Z (-32768 to 32767)
  int16_t rx;     // Axis Rx (-32768 to 32767)
  int16_t ry;     // Axis Ry (-32768 to 32767)
  int16_t rz;     // Axis Rz (-32768 to 32767)

} __attribute__((packed)) USB_HID_JoystickReport_t;

typedef struct __attribute__((packed)) {
    // --- 传感器量程与灵敏度 ---
    int32_t maxRange;          // 对应代码中的 maxRange (默认 2400000)
    float   sensitivityScale;  // 预留：可在 scale 基础上进一步微调灵敏度

    // --- 物理计算系数 (用于坐标变换公式) ---
    // fx ≈ cos_factor * (dv2 - dv1)
    // fy ≈ sin_factor * (0.5*(dv1 + dv2) - dv0)
    float   xy_factor;         // 对应代码中的 0.866f (cos 30°)

    // --- 校准逻辑参数 ---
    uint32_t calibLongPressMs; // 触发校准所需的长按时间 (默认 5000)

    // --- 摇杆性能微调 ---
    uint16_t deadzone;         // 软件死区，防止静止时抖动
    uint8_t  invertX;          // 是否反转X轴 (0: 正常, 1: 反转)
    uint8_t  invertY;          // 是否反转Y轴 (0: 正常, 1: 反转)
    uint8_t  invertZ;          // 是否反转Z轴 (0: 正常, 1: 反转)

    // --- 采样与更新频率控制 ---
    uint16_t reportIntervalMs; // 发送 HID 报告的时间间隔 (HAL_Delay 的替代值)

    // 预留槽位方便后续扩展而不破坏结构体大小
    uint8_t  reserved[8];
} JoystickConfig_t;

uint32_t DWT_GetUs(void) {
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000);
}

#define DEBOUNCE_MS 10
#define NUM_BUTTONS 22
typedef struct {
    uint8_t last_raw_state;   // 上一次扫描到的原始电平
    uint8_t stable_state;     // 经过消抖后的稳定状态
    uint32_t last_change_ms;  // 最后一次电平发生变化的时间点
} Button_Typedef;
Button_Typedef buttons[NUM_BUTTONS] = {0};
uint8_t button_stable_state[NUM_BUTTONS] = {0};

extern "C" void RealMain(){
	while(!(sg0.configGood&&sg1.configGood&&sg2.configGood));
	PE5=0;

	USB_HID_JoystickReport_t report = {0};
	ZeroTracker zt0(sg0.ADCdata),zt1(sg1.ADCdata),zt2(sg2.ADCdata);
	int32_t v0,v1,v2;
	int32_t dc0,dc1,dc2;
	int32_t dv0,dv1,dv2;
	int32_t maxRange=2400000;
	const float scale = 32767.f / maxRange;




	// 定义引脚数组方便循环处理
	uint8_t raw_pins[NUM_BUTTONS];

    // 时间窗口（ms）
//    const uint32_t sample_ms = 1000;
//    const uint32_t cpu_freq  = 72000000;  // STM32F103 72 MHz
//    uint32_t window_start    = DWT->CYCCNT;
//    uint32_t last_irq_cycles = 0;
//    uint32_t last_irq_count  = 0;
//    float irq_ratio=0;
    uint32_t lastL=0;
	while(true){
		uint32_t loopTime=DWT_GetUs()-lastL;
		lastL = DWT_GetUs();
		v0=sg0.ADCdata;
		v1=sg1.ADCdata;
		v2=sg2.ADCdata;
		dc0=zt0.update(v0);
		dc1=zt1.update(v1);
		dc2=zt2.update(v2);
		dv0=v0-dc0;
		dv1=v1-dc1;
		dv2=v2-dc2;
		// X component: fx = v2*cos(330°) + v1*cos(210°) + v0*cos(90°)
		//              fx = v2*0.866 + v1*(-0.866) + v0*0
		//              fx ≈ 0.866*(v2 - v1)
		int16_t fx = (int16_t)__SSAT((int32_t)((dv2 - dv1) * scale * 0.866f), 16);

		// Y component: fy = v0*sin(90°) + v1*sin(210°) + v2*sin(330°)
		//              fy = v0*1 + v1*(-0.5) + v2*(-0.5)
		//              fy = v0 - 0.5*(v1 + v2)
		int16_t fy = (int16_t)__SSAT((int32_t)((0.5f*(dv1 + dv2) - dv0) * scale), 16);

		// Z component: average compression (all sensors pushed down)
		int16_t fz = (int16_t)__SSAT((int32_t)((dv0 + dv1 + dv2) * scale / 3.f), 16);

		// Set axes
		report.x = fx;
		report.y = fy;
		report.z = fz;
		report.rx = dv0*scale;
		report.ry = dv1*scale;
		report.rz = dv2*scale;
		report.rz = loopTime;
		raw_pins[0] = !PA0;
		raw_pins[1] = !PF2;
		raw_pins[2] = !PA1;
		raw_pins[3] = !PE6;
		raw_pins[4] = !PF1;
		raw_pins[5] = !PF9;
		raw_pins[6] = !PC2;
		raw_pins[7] = !PF6;
		raw_pins[8] = !PC13;
		raw_pins[9] = !PF3;
		raw_pins[10] = !PF4;
		raw_pins[11] = !PF0;
		raw_pins[12] = !PF8;
		raw_pins[13] = !PE0;
		raw_pins[14] = !PE1;
		raw_pins[15] = !PC3;
		raw_pins[16] = !PC1;
		raw_pins[17] = !PF5;
		raw_pins[18] = !PF10;
		raw_pins[19] = !PF7;
		raw_pins[20] = !PC0;
		raw_pins[21] = 0;

		uint32_t now = HAL_GetTick(); // 使用毫秒级时间戳

		for (int i = 0; i < NUM_BUTTONS; i++) {
		    uint8_t current_raw = raw_pins[i];

		    // 如果当前读到的引脚状态和上一次不一样
		    if (current_raw != buttons[i].last_raw_state) {
		        buttons[i].last_change_ms = now; // 重置计时器
		    }

		    // 如果当前电平已经稳定维持了超过 DEBOUNCE_MS
		    if ((now - buttons[i].last_change_ms) >= DEBOUNCE_MS) {
		        // 更新稳定状态
		        buttons[i].stable_state = current_raw;
		        button_stable_state[i] = buttons[i].stable_state;
		    }

		    // 更新“上一次”的原始电平记录
		    buttons[i].last_raw_state = current_raw;
		}

		report.buttons.bits.button1  = button_stable_state[0];
		report.buttons.bits.button2  = button_stable_state[1];
		report.buttons.bits.button3  = button_stable_state[2];
		report.buttons.bits.button4  = button_stable_state[3];
		report.buttons.bits.button5  = button_stable_state[4];
		report.buttons.bits.button6  = button_stable_state[5];
		report.buttons.bits.button7  = button_stable_state[6];
		report.buttons.bits.button8  = button_stable_state[7];
		report.buttons.bits.button9  = button_stable_state[8];
		report.buttons.bits.button10 = button_stable_state[9];
		report.buttons.bits.button11 = button_stable_state[10];
		report.buttons.bits.button12 = button_stable_state[11];
		report.buttons.bits.button13 = button_stable_state[12];
		report.buttons.bits.button14 = button_stable_state[13];
		report.buttons.bits.button15 = button_stable_state[14];
		report.buttons.bits.button16 = button_stable_state[15];
		report.buttons.bits.button17 = button_stable_state[16];
		report.buttons.bits.button18 = button_stable_state[17];
		report.buttons.bits.button19 = button_stable_state[18];
		report.buttons.bits.button20 = button_stable_state[19];
		report.buttons.bits.button21 = button_stable_state[20];
		report.buttons.bits.button22 = button_stable_state[21];

		static uint32_t pressStartTime = 0;
		// 读取 PC0 引脚状态
		if (PC0 == 0) {
		    // 如果是刚按下（计时器还没启动），则记录当前系统时间
		    if (pressStartTime == 0) {
		        pressStartTime = HAL_GetTick();
		    }

		    // 判断按下持续时间是否超过 5000 毫秒
		    if (HAL_GetTick() - pressStartTime > 5000) {
		        zt0.calibrate(sg0.ADCdata);
		        zt1.calibrate(sg1.ADCdata);
		        zt2.calibrate(sg2.ADCdata);
		    }
		} else {
		    // 只要按键松开（电平变回 1），立即清零计时器
		    pressStartTime = 0;
		}
		// Send the report
		//if(DWT_GetUs()-lastL>=500){
			USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
		//}
		//HAL_Delay(5);

		// ------------------- CPU 占用率统计 -------------------
//		uint32_t now = DWT->CYCCNT;
//		uint32_t elapsed_cycles = now - window_start;
//
//		if (elapsed_cycles >= sample_ms * (cpu_freq / 1000)) {
//			uint32_t delta_cycles = irq_cycles - last_irq_cycles;
//			uint32_t delta_count  = irq_count - last_irq_count;
//
//			irq_ratio = (float)delta_cycles / elapsed_cycles;
//
//			last_irq_cycles = irq_cycles;
//			last_irq_count  = irq_count;
//			window_start    = now;
//		}

	}
}

