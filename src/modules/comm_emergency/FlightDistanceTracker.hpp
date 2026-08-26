#pragma once

#include <cmath>
#include <stdint.h>

class FlightDistanceTracker
{
public:
	static constexpr uint64_t MAX_SAMPLE_GAP_US = 1'000'000;

	void update(uint64_t sample_time_us, bool armed, bool landed_confirmed, bool horizontal_velocity_valid,
		    float velocity_north_m_s, float velocity_east_m_s)
	{
		if (!armed || landed_confirmed) {
			resetForNextFlight();
			return;
		}

		if (!_ready_for_flight || sample_time_us == 0 || !horizontal_velocity_valid
		    || !std::isfinite(velocity_north_m_s) || !std::isfinite(velocity_east_m_s)) {
			invalidate();
			return;
		}

		const float speed_m_s = std::sqrt(velocity_north_m_s * velocity_north_m_s
					      + velocity_east_m_s * velocity_east_m_s);

		if (!std::isfinite(speed_m_s)) {
			invalidate();
			return;
		}

		if (_last_sample_time_us == 0) {
			_last_sample_time_us = sample_time_us;
			_last_speed_m_s = speed_m_s;
			_distance_valid = true;
			return;
		}

		if (sample_time_us <= _last_sample_time_us) {
			return;
		}

		const uint64_t sample_gap_us = sample_time_us - _last_sample_time_us;

		if (sample_gap_us > MAX_SAMPLE_GAP_US) {
			invalidate();
			return;
		}

		const float sample_gap_s = static_cast<float>(sample_gap_us) * 1e-6f;
		const float distance_increment_m = 0.5f * (_last_speed_m_s + speed_m_s) * sample_gap_s;

		if (!std::isfinite(distance_increment_m) || distance_increment_m < 0.f
		    || !std::isfinite(_distance_m + distance_increment_m)) {
			invalidate();
			return;
		}

		_distance_m += distance_increment_m;
		_last_sample_time_us = sample_time_us;
		_last_speed_m_s = speed_m_s;
	}

	bool valid(uint64_t now_us) const
	{
		return _distance_valid && _last_sample_time_us != 0 && now_us >= _last_sample_time_us
		       && now_us - _last_sample_time_us <= MAX_SAMPLE_GAP_US;
	}

	float distanceM() const { return _distance_m; }

private:
	void resetForNextFlight()
	{
		_distance_m = 0.f;
		_last_speed_m_s = 0.f;
		_last_sample_time_us = 0;
		_distance_valid = false;
		_ready_for_flight = true;
	}

	void invalidate()
	{
		_distance_valid = false;
		_ready_for_flight = false;
		_last_sample_time_us = 0;
		_last_speed_m_s = 0.f;
	}

	float _distance_m{0.f};
	float _last_speed_m_s{0.f};
	uint64_t _last_sample_time_us{0};
	bool _distance_valid{false};
	bool _ready_for_flight{false};
};
