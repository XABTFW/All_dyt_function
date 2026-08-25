#pragma once

#include <stdint.h>

class CommEmergencyStateMachine
{
public:
	static constexpr uint64_t LAND_STATUS_TIMEOUT_US = 3'000'000;

	enum class State : uint8_t { Idle, Holding, Committed };
	enum class Action : uint8_t { None, Hold, ResumeMission, Return, Land };

	struct Input {
		bool armed{false};
		bool landed{true};
		bool mission_intended{false};
		bool link_lost{false};
		bool battery_below_threshold{false};
		bool rtl_feasible{false};
	};

	static bool landedOrStatusUnavailable(uint64_t now_us, uint64_t timestamp_us, bool landed)
	{
		return landed || timestamp_us == 0 || timestamp_us > now_us
		       || now_us - timestamp_us >= LAND_STATUS_TIMEOUT_US;
	}

	Action update(uint64_t now_us, const Input &input, uint64_t wait_us,
		      Action timeout_action = Action::Return)
	{
		if (!input.armed || input.landed) {
			reset();
			return Action::None;
		}

		switch (_state) {
		case State::Idle:
			if (input.link_lost) {
				_state = State::Holding;
				_loss_started = now_us;
				_resume_mission = input.mission_intended;
				return Action::Hold;
			}

			break;

		case State::Holding:
			if (input.battery_below_threshold) {
				_state = State::Committed;
				return input.rtl_feasible ? Action::Return : Action::Land;
			}

			if (!input.link_lost) {
				const bool resume_mission = _resume_mission;
				reset();
				return resume_mission ? Action::ResumeMission : Action::None;
			}

			if (now_us - _loss_started >= wait_us) {
				_state = State::Committed;
				return timeout_action == Action::Land ? Action::Land : Action::Return;
			}

			break;

		case State::Committed:
			break;
		}

		return Action::None;
	}

	void reset()
	{
		_state = State::Idle;
		_loss_started = 0;
		_resume_mission = false;
	}

	State state() const { return _state; }
	uint64_t lossStarted() const { return _loss_started; }

private:
	State _state{State::Idle};
	uint64_t _loss_started{0};
	bool _resume_mission{false};
};
