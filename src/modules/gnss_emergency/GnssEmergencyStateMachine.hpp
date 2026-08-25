#pragma once

#include <stdint.h>

class GnssEmergencyStateMachine
{
public:
	static constexpr uint64_t LAND_STATUS_TIMEOUT_US = 3'000'000;

	enum class State : uint8_t { Idle, Landing };
	enum class Action : uint8_t { None, Descend, ResumeMission };

	struct Input {
		bool armed{false};
		bool landed{true};
		bool mission_active{false};
		bool interference{false};
		bool navigation_recovered{false};
	};

	static bool landedOrStatusUnavailable(uint64_t now_us, uint64_t timestamp_us, bool landed)
	{
		return landed || timestamp_us == 0 || timestamp_us > now_us
		       || now_us - timestamp_us >= LAND_STATUS_TIMEOUT_US;
	}

	Action update(uint64_t now_us, const Input &input, uint64_t recovery_time_us)
	{
		if (!input.armed || input.landed) {
			reset();
			return Action::None;
		}

		if (_state == State::Idle) {
			if (input.mission_active && input.interference) {
				_state = State::Landing;
				return Action::Descend;
			}

			return Action::None;
		}

		if (input.interference || !input.navigation_recovered) {
			_recovery_started = 0;
			return Action::None;
		}

		if (_recovery_started == 0) {
			_recovery_started = now_us;
			return Action::None;
		}

		if (now_us - _recovery_started >= recovery_time_us) {
			reset();
			return Action::ResumeMission;
		}

		return Action::None;
	}

	void reset()
	{
		_state = State::Idle;
		_recovery_started = 0;
	}

	State state() const { return _state; }
	uint64_t recoveryStarted() const { return _recovery_started; }

private:
	State _state{State::Idle};
	uint64_t _recovery_started{0};
};
