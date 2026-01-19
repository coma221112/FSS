/*
 * GPIOHelper.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_GPIOHELPER_HPP_
#define INC_GPIOHELPER_HPP_

#include "stm32f1xx_hal.h"

enum class GPIOMode {
    Input,          // 普通输入（浮空）
    InputPullUp,    // 上拉输入
    InputPullDown,  // 下拉输入
    OutputPP,       // 推挽输出 (Push-Pull)
    OutputOD        // 开漏输出 (Open-Drain)
};

class GPIOPin {
public:
    GPIOPin(GPIO_TypeDef* port, uint16_t pin)
        : port_(port), pin_(pin) {}

    inline void write(bool v) const {
        HAL_GPIO_WritePin(
            port_, pin_,
            v ? GPIO_PIN_SET : GPIO_PIN_RESET
        );
    }

    inline bool read() const {
        return HAL_GPIO_ReadPin(port_, pin_) == GPIO_PIN_SET;
    }

    inline void toggle() const {
        HAL_GPIO_TogglePin(port_, pin_);
    }

    inline GPIOPin& operator=(bool v) {
        write(v);
        return *this;
    }

    inline operator bool() const {
        return read();
    }

    inline char sub() const {
            if (port_ == GPIOA) return 'A';
            if (port_ == GPIOB) return 'B';
            if (port_ == GPIOC) return 'C';
            if (port_ == GPIOD) return 'D';
            if (port_ == GPIOE) return 'E';
            if (port_ == GPIOF) return 'F';
            if (port_ == GPIOG) return 'G';
            return '?';
    }

    inline uint8_t pin() const {
    	return __builtin_ctz(pin_);
    }

    inline void setMode(GPIOMode mode, uint32_t speed = GPIO_SPEED_FREQ_LOW) const
    {
        GPIO_InitTypeDef init = {};
        init.Pin = pin_;

        switch (mode)
        {
        case GPIOMode::Input:
            init.Mode = GPIO_MODE_INPUT;
            init.Pull = GPIO_NOPULL;
            break;

        case GPIOMode::InputPullUp:
            init.Mode = GPIO_MODE_INPUT;
            init.Pull = GPIO_PULLUP;
            break;

        case GPIOMode::InputPullDown:
            init.Mode = GPIO_MODE_INPUT;
            init.Pull = GPIO_PULLDOWN;
            break;

        case GPIOMode::OutputPP:
            init.Mode = GPIO_MODE_OUTPUT_PP;
            init.Pull = GPIO_NOPULL;
            init.Speed = speed;
            break;

        case GPIOMode::OutputOD:
            init.Mode = GPIO_MODE_OUTPUT_OD;
            init.Pull = GPIO_NOPULL;
            init.Speed = speed;
            break;
        }

        HAL_GPIO_Init(port_, &init);
    }


    GPIO_TypeDef* port_;
    uint16_t pin_;
};

#define DEFINE_PIN_GROUP(port_char, port_ptr) \
    static GPIOPin P##port_char##0  {port_ptr, GPIO_PIN_0};  \
    static GPIOPin P##port_char##1  {port_ptr, GPIO_PIN_1};  \
    static GPIOPin P##port_char##2  {port_ptr, GPIO_PIN_2};  \
    static GPIOPin P##port_char##3  {port_ptr, GPIO_PIN_3};  \
    static GPIOPin P##port_char##4  {port_ptr, GPIO_PIN_4};  \
    static GPIOPin P##port_char##5  {port_ptr, GPIO_PIN_5};  \
    static GPIOPin P##port_char##6  {port_ptr, GPIO_PIN_6};  \
    static GPIOPin P##port_char##7  {port_ptr, GPIO_PIN_7};  \
    static GPIOPin P##port_char##8  {port_ptr, GPIO_PIN_8};  \
    static GPIOPin P##port_char##9  {port_ptr, GPIO_PIN_9};  \
    static GPIOPin P##port_char##10 {port_ptr, GPIO_PIN_10}; \
    static GPIOPin P##port_char##11 {port_ptr, GPIO_PIN_11}; \
    static GPIOPin P##port_char##12 {port_ptr, GPIO_PIN_12}; \
    static GPIOPin P##port_char##13 {port_ptr, GPIO_PIN_13}; \
    static GPIOPin P##port_char##14 {port_ptr, GPIO_PIN_14}; \
    static GPIOPin P##port_char##15 {port_ptr, GPIO_PIN_15};

// 展开所有端口
DEFINE_PIN_GROUP(A, GPIOA)
DEFINE_PIN_GROUP(B, GPIOB)
DEFINE_PIN_GROUP(C, GPIOC)
DEFINE_PIN_GROUP(D, GPIOD)
DEFINE_PIN_GROUP(E, GPIOE)
DEFINE_PIN_GROUP(F, GPIOF)
DEFINE_PIN_GROUP(G, GPIOG)

// 清理宏，避免污染后续代码
#undef DEFINE_PIN_GROUP

#endif /* INC_GPIOHELPER_HPP_ */
