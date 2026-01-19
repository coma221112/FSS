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
      uint32_t padding  : 12;  // Padding to align to 32 bits
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
	PB5=0;

	USB_HID_JoystickReport_t report = {0};
	ZeroTracker zt0(sg0.ADCdata),zt1(sg1.ADCdata),zt2(sg2.ADCdata);
	int32_t v0,v1,v2;
	int32_t dc0,dc1,dc2;
	int32_t dv0,dv1,dv2;
	int32_t maxRange=800000;
	const int32_t divisor = maxRange / 32767;
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
		// X component: weighted sum
		int16_t fx = (int16_t)__SSAT((-dv1 + dv2) / divisor, 16);

		// Y component: weighted sum
		int16_t fy = (int16_t)__SSAT((2*dv0 - dv1 - dv2) / divisor, 16);

		// Z component: average of all three
		int16_t fz = (int16_t)__SSAT((dv0 + dv1 + dv2) / (divisor * 3), 16);

		// Set axes
		report.x = fx;
		report.y = fy;
		report.z = fz;

		// Send the report
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
		HAL_Delay(5);
	}
}

