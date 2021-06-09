#pragma once

#include <cstdint>
#include <cassert>

class Lap {
private:
	float startTime;
	float endTime;

	static float toTimeStamp(float seconds) {
		uint16_t hours = (uint16_t)(seconds / 3600);
		uint16_t minutes = (uint16_t)(seconds / 60 - (hours * 60));
		float fs = seconds - (uint16_t)(seconds / 100) * 100;
		float timestamp = (((hours * 100) + minutes) * 100) + fs;

		return timestamp;
	}

	static float toSeconds(float time) {
		uint16_t hm = (uint16_t)(time / 100);
		uint16_t hours = (uint16_t)(time / 10000);
		uint16_t minutes = (hm - (hours * 100) + (hours * 60));
		float seconds = time - (hm * 100) + (minutes * 60);

		return seconds;
	}
public:
	void setStartTime(const float t) {
		assert(t >= 0.0f);
		startTime = t;
	}

	float getStartTime() const { return startTime; }

	void setEndTime(const float t) { 
		assert(t >= 0.0f);
		endTime = t;
	}

	float getEndTime() const { return endTime; }

	float getLapDuration() const {
		if (endTime < startTime)
			return 0.0f;

		return (endTime - startTime);
	}
};