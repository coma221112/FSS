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

extern "C" void RealMain(){
	while(!(sg0.configGood&&sg1.configGood&&sg2.configGood));
	PE5=0;

	USB_HID_JoystickReport_t report = {0};
	ZeroTracker zt0(sg0.ADCdata),zt1(sg1.ADCdata),zt2(sg2.ADCdata);
	int32_t v0,v1,v2;
	int32_t dc0,dc1,dc2;
	int32_t dv0,dv1,dv2;
	int32_t maxRange=800000;
	const float scale = 32767.f / maxRange;

    // 时间窗口（ms）
    const uint32_t sample_ms = 1000;
    const uint32_t cpu_freq  = 72000000;  // STM32F103 72 MHz
    uint32_t window_start    = DWT->CYCCNT;
    uint32_t last_irq_cycles = 0;
    uint32_t last_irq_count  = 0;
    float irq_ratio=0;

	while(true){
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

		report.buttons.bits.button1 = !PE0;
		report.buttons.bits.button2 = !PE1;
		report.buttons.bits.button3 = !PC13;
		report.buttons.bits.button4 = !PF0;
		report.buttons.bits.button5 = !PF1;
		report.buttons.bits.button6 = !PF2;
		report.buttons.bits.button7 = !PF3;
		report.buttons.bits.button8 = !PF4;
		report.buttons.bits.button9 = !PF5;
		report.buttons.bits.button10 = !PF6;
		report.buttons.bits.button11 = !PF7;
		report.buttons.bits.button12 = !PF8;
		report.buttons.bits.button13 = !PF9;
		report.buttons.bits.button14 = !PF10;
		report.buttons.bits.button15 = !PC0;
		report.buttons.bits.button16 = !PC1;
		report.buttons.bits.button17 = !PC2;
		report.buttons.bits.button18 = !PC3;
		report.buttons.bits.button19 = !PA0;
		report.buttons.bits.button20 = !PA1;
		report.buttons.bits.button21 = !PA2;
		report.buttons.bits.button22 = !PE6;

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
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
		//HAL_Delay(5);

		// ------------------- CPU 占用率统计 -------------------
		uint32_t now = DWT->CYCCNT;
		uint32_t elapsed_cycles = now - window_start;

		if (elapsed_cycles >= sample_ms * (cpu_freq / 1000)) {
			uint32_t delta_cycles = irq_cycles - last_irq_cycles;
			uint32_t delta_count  = irq_count - last_irq_count;

			irq_ratio = (float)delta_cycles / elapsed_cycles;

			last_irq_cycles = irq_cycles;
			last_irq_count  = irq_count;
			window_start    = now;
		}
	}
}

