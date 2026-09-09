#pragma once

#include <stdint.h>

class GnssEmergencyStateMachine
{
public:
	static constexpr uint64_t LAND_STATUS_TIMEOUT_US = 3'000'000;

	enum class State : uint8_t { Idle, Landing, Released };
	enum class Action : uint8_t { None, Land };

	struct Input {
		bool armed{false};
		bool landed{true};
		bool gnss_failure{false};
		bool manual_control_available{false};
		bool manual_takeover{false};
	};

	static bool landedOrStatusUnavailable(uint64_t now_us, uint64_t timestamp_us, bool landed)
	{
		return landed || timestamp_us == 0 || timestamp_us > now_us
		       || now_us - timestamp_us >= LAND_STATUS_TIMEOUT_US;
	}

	Action update(const Input &input)
	{
		if (!input.armed || input.landed) {
			reset();
			return Action::None;
		}

		switch (_state) {
		case State::Idle:
			if (input.gnss_failure) {
				_state = State::Landing;
				return Action::Land;
			}

			break;

		case State::Landing:
			if (input.manual_takeover) {
				_state = State::Released;
			}

			break;

		case State::Released:
			if (!input.gnss_failure) {
				reset();

			} else if (!input.manual_control_available) {
				_state = State::Landing;
				return Action::Land;
			}

			break;
		}

		return Action::None;
	}

	void reset()
	{
		_state = State::Idle;
	}

	State state() const { return _state; }

private:
	State _state{State::Idle};
};
