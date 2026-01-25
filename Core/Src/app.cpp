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
#include "JoystickProtocol.hpp"
#include "cmath"

extern USBD_HandleTypeDef hUsbDeviceFS;

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
uint8_t raw_pins[NUM_BUTTONS];

inline int32_t ApplyDeadzone(int32_t Value, uint16_t Threshold) {
    if (Value > (int32_t)Threshold) return Value - Threshold;
    if (Value < -(int32_t)Threshold) return Value + Threshold;
    return 0;
}

inline float Clamp(float raw_val){
	return fmaxf(-1.0f, fminf(1.0f, raw_val));
}

uint8_t HID_SendReport_Safe(USBD_HandleTypeDef *pdev,
                                        uint8_t *report,
                                        uint16_t len,
                                        uint8_t priority)  // 0=normal, 1=high priority
{
    USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)pdev->pClassData;

    if (priority) {
        // High priority - wait up to 50ms
        uint32_t start = HAL_GetTick();
        while (hhid->state == CUSTOM_HID_BUSY) {
            if ((HAL_GetTick() - start) > 50) {
                return USBD_BUSY;
            }
        }
        return USBD_CUSTOM_HID_SendReport(pdev, report, len);
    } else {
        // Normal priority - drop if busy
        if (hhid->state == CUSTOM_HID_BUSY) {
            return USBD_BUSY;
        }
        return USBD_CUSTOM_HID_SendReport(pdev, report, len);
    }
}

extern "C" void RealMain(){
	while(!(sg0.configGood&&sg1.configGood&&sg2.configGood));
	PE5=0;

	ZeroTracker zt0(sg0.ADCdata),zt1(sg1.ADCdata),zt2(sg2.ADCdata);
	int32_t v0,v1,v2;
	int32_t dc0,dc1,dc2;
	int32_t dv0,dv1,dv2;
    uint32_t lastL=0;

	while(true){
		uint32_t loopTime = DWT_GetUs()-lastL;
		lastL = DWT_GetUs();

		float maxRange = joystickConfig.maxRange;
		//float scale = 32767.f / maxRange;

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
		float fx = Clamp((dv2 - dv1) / maxRange * 0.866f);
		// Y component: fy = v0*sin(90°) + v1*sin(210°) + v2*sin(330°)
		//              fy = v0*1 + v1*(-0.5) + v2*(-0.5)
		//              fy = v0 - 0.5*(v1 + v2)
		float fy = Clamp((0.5f*(dv1 + dv2) - dv0) / maxRange);
		// Z component: average compression (all sensors pushed down)
		float fz = Clamp(((dv0 + dv1 + dv2) / maxRange / 3.f));

		// 1. 计算合力的大小（模长）
		float magnitude = sqrtf(fx * fx + fy * fy);

		// 2. 检查是否在死区内
		if (magnitude < joystickConfig.deadzone) {
		    fx = 0;
		    fy = 0;
		} else {
		    // 3. 线性重映射（可选）：让输出从死区边缘平滑起始
		    float factor = (magnitude - joystickConfig.deadzone) / (1 - joystickConfig.deadzone);
		    // 防止溢出
		    if (factor > 1.0f) factor = 1.0f;

		    // 重新缩放矢量
		    fx = (fx / magnitude) * factor;
		    fy = (fy / magnitude) * factor;
		}

		// Set axes
		auto& report = joystickReport;
		report.x = fx * 32767;
		report.y = fy * 32767;
		report.z = fz * 32767;
		report.rx = dv0 / maxRange * 32767;
		report.ry = dv1 / maxRange * 32767;
		report.rz = dv2 / maxRange * 32767;
		report.lt = loopTime;

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

		//report.btn1_8=button_stable_state[0];
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
		    if (HAL_GetTick() - pressStartTime > joystickConfig.calibLongPressMs) {
		        zt0.calibrate(sg0.ADCdata);
		        zt1.calibrate(sg1.ADCdata);
		        zt2.calibrate(sg2.ADCdata);
		    }
		} else {
		    // 只要按键松开（电平变回 1），立即清零计时器
		    pressStartTime = 0;
		}
		// Send the report
		static uint32_t lastJoystickSend = 0;
		static uint32_t lastConfigSend = 0;


		if (configNeedsSending) {
		    if (HID_SendReport_Safe(&hUsbDeviceFS, (uint8_t*)&joystickConfig, sizeof(joystickConfig), 1) == USBD_OK) {
		    	configNeedsSending = false;
		    }
		}
		else{
			HID_SendReport_Safe(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report), 0);
		}
	}
}

