/*
 * main.cpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#include "HardwareConfig.hpp"

extern "C" void RealMain(){
	while(!(sg0.configGood&&sg1.configGood&&sg2.configGood));
	PB5=0;
}

