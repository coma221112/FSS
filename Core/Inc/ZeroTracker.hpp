/*
 * ZeroTracker.hpp
 *
 *  Created on: Jan 19, 2026
 *      Author: Drac
 */

#ifndef INC_ZEROTRACKER_HPP_
#define INC_ZEROTRACKER_HPP_

#include <stdint.h>

#include <stdint.h>

class ZeroTracker {
public:
    ZeroTracker(int32_t initialZero = 0)
        : zero(initialZero), deadzone(40000), fastzone(10000), slowzone(40000)
        , slowRate(0.05), fastRate(0.1) {}

    void setDeadzone(int32_t threshold) { deadzone = threshold; }
    void setFastzone(int32_t threshold) { fastzone = threshold; }
    void setSlowzone(int32_t threshold) { slowzone = threshold; }
    void setSlowRate(int8_t rate) { slowRate = rate; }
    void setFastRate(int8_t rate) { fastRate = rate; }

    int32_t update(int32_t rawValue) {
        int32_t dev = rawValue - zero;
        int32_t absDev = (dev < 0) ? -dev : dev;
        float step = 0;
        if (absDev < deadzone) {
			if (absDev <= fastzone) {
				step = fastRate;
			} else if (absDev <= slowzone) {
				step = slowRate;
			}

			if (dev > 0) {
				zero += step;
			} else {
				zero -= step;
			}
		}
        return zero;
        // Beyond slowzone: no tracking
    }

    int32_t getCorrected(int32_t rawValue) const { return rawValue - zero; }
    int32_t getZero() const { return zero; }
    void calibrate(int32_t rawValue) { zero = rawValue; }

private:
    float zero;
    int32_t deadzone;   // No movement zone
    int32_t fastzone;   // Fast tracking zone
    int32_t slowzone;   // Slow tracking zone
    float slowRate;
    float fastRate;
};



#endif /* INC_ZEROTRACKER_HPP_ */
