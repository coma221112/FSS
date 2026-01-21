/*
 * HardwareConfig.hpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_HARDWARECONFIG_HPP_
#define INC_HARDWARECONFIG_HPP_

#include "CS1237.hpp"

volatile CS1237 sg0(PB4,PB5);
volatile CS1237 sg1(PB6,PB7);
volatile CS1237 sg2(PB8,PB9);



#endif /* INC_HARDWARECONFIG_HPP_ */
