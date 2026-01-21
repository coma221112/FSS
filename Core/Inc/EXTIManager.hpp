/*
 * EXTI.hpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_EXTI_HPP_
#define INC_EXTI_HPP_

#include "stm32f1xx_hal.h"

class EXTI_Observer {
public:
    virtual void onExternalInterrupt() = 0;
    virtual ~EXTI_Observer() = default;
};

class EXTI_Manager {
private:

    static inline EXTI_Observer* volatile registry[16] = { nullptr };

public:
    static void registerPin(uint16_t pin, EXTI_Observer* observer) {
        if (pin == 0) return;
        int i = __builtin_ctz(pin);

        if (i < 16) {
            if (registry[i] != nullptr && registry[i] != observer) {
                while(1);
            }
            registry[i] = observer;
        }
    }

    static void dispatch(uint16_t pin) {
        if (pin == 0) return;
        int i = __builtin_ctz(pin);

        if (i < 16 && registry[i]) {
            registry[i]->onExternalInterrupt();
        }
    }
};

volatile uint32_t irq_cycles = 0;      // 中断执行时间（CPU ticks）
volatile uint32_t irq_count = 0;      // 中断触发次数

#ifdef __cplusplus
extern "C" {
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t start = DWT->CYCCNT;     // 记录进入中断的 CPU cycle
	EXTI_Manager::dispatch(GPIO_Pin);
	uint32_t end = DWT->CYCCNT;
	irq_cycles += (end - start);
	irq_count++;
}

#ifdef __cplusplus
}
#endif


#endif /* INC_EXTI_HPP_ */
