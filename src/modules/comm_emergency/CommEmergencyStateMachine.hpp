#pragma once

#include <stdint.h>

class CommEmergencyStateMachine
{
public:
	static constexpr uint64_t LAND_STATUS_TIMEOUT_US = 3'000'000;

	enum class State : uint8_t { Idle, Holding, Committed, Resuming };
	enum class Action : uint8_t { None, Hold, ResumeMission, ResumeOffboard, Return, Land };

	struct Input {
		bool armed{false};
		bool landed{true};
		bool mission_intended{false};
		bool offboard_intended{false};
		bool link_lost{false};
		bool battery_below_threshold{false};
		bool rtl_feasible{false};
		bool return_active{false};
		bool resume_distance_allowed{false};
		bool midcourse_requested{false};
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
			if (!input.link_lost && input.return_active && input.midcourse_requested) {
				_state = State::Resuming;
				_resume_from_return = true;
				_resume_midcourse_requested = true;
				return Action::ResumeOffboard;
			}

			if (input.link_lost) {
				_state = State::Holding;
				_loss_started = now_us;
				_resume_mission = input.mission_intended;
				_resume_offboard = input.offboard_intended;
				return Action::Hold;
			}

			break;

		case State::Holding:
			if (input.battery_below_threshold) {
				_state = State::Committed;
				_return_recovery_allowed = false;
				return input.rtl_feasible ? Action::Return : Action::Land;
			}

			if (!input.link_lost) {
				const Action resume_action = input.midcourse_requested ? Action::ResumeOffboard : resumeAction();

				if (resume_action == Action::None) {
					reset();

				} else {
					_state = State::Resuming;
					_resume_from_return = false;
					_resume_midcourse_requested = input.midcourse_requested;
				}

				return resume_action;
			}

			if (now_us - _loss_started >= wait_us) {
				_state = State::Committed;
				_return_recovery_allowed = timeout_action != Action::Land;
				return timeout_action == Action::Land ? Action::Land : Action::Return;
			}

			break;

		case State::Committed:
			if (!input.link_lost && input.return_active && input.midcourse_requested) {
				_state = State::Resuming;
				_resume_from_return = true;
				_resume_midcourse_requested = true;
				return Action::ResumeOffboard;
			}

			if (_return_recovery_allowed && !input.link_lost && input.return_active
			    && input.resume_distance_allowed) {
				const Action resume_action = resumeAction();

				if (resume_action != Action::None) {
					_state = State::Resuming;
					_resume_from_return = true;
					_resume_midcourse_requested = false;
					return resume_action;
				}
			}

			break;

		case State::Resuming:
			if (input.link_lost) {
				if (_resume_from_return) {
					_state = State::Committed;

				} else {
					_state = State::Holding;
					_loss_started = now_us;
				}

				_resume_from_return = false;
				_resume_midcourse_requested = false;

			} else if (_resume_from_return &&
				   (!input.return_active || (!_resume_midcourse_requested && !input.resume_distance_allowed))) {
				_state = State::Committed;
				_resume_from_return = false;
				_resume_midcourse_requested = false;
			}

			break;
		}

		return Action::None;
	}

	void reset()
	{
		_state = State::Idle;
		_loss_started = 0;
		_resume_mission = false;
		_resume_offboard = false;
		_return_recovery_allowed = false;
		_resume_from_return = false;
		_resume_midcourse_requested = false;
	}

	void resumeCompleted() { reset(); }

	State state() const { return _state; }
	uint64_t lossStarted() const { return _loss_started; }
	bool resumeFromReturn() const { return _state == State::Resuming && _resume_from_return; }

private:
	Action resumeAction() const
	{
		if (_resume_mission) {
			return Action::ResumeMission;
		}

		return _resume_offboard ? Action::ResumeOffboard : Action::None;
	}

	State _state{State::Idle};
	uint64_t _loss_started{0};
	bool _resume_mission{false};
	bool _resume_offboard{false};
	bool _return_recovery_allowed{false};
	bool _resume_from_return{false};
	bool _resume_midcourse_requested{false};
};
