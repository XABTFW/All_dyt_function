#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/rangefinder/PX4Rangefinder.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sdm50_status.h>

using namespace time_literals;

class SDM50 : public ModuleBase<SDM50>, public px4::ScheduledWorkItem
{
public:
	SDM50(const char *port, uint8_t rotation);
	~SDM50() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;

private:
	enum class ParseState : uint8_t {
		WaitHeader,
		DistanceLow,
		DistanceHigh,
		Checksum
	};

	void Run() override;
	bool open_serial();
	void close_serial();
	bool send_start_command();
	void parse_byte(uint8_t byte);
	bool send_command(const uint8_t *command, size_t length);
	void print_info();
	int print_range();

	static constexpr uint8_t FRAME_HEADER{0x5C};
	static constexpr uint16_t NO_TARGET_MM{50000};
	static constexpr uint16_t MIN_DISTANCE_MM{50};
	static constexpr uint16_t MAX_DISTANCE_MM{50000};
	static constexpr unsigned RAW_DUMP_SIZE{32};

	PX4Rangefinder _px4_rangefinder;
	uint32_t _device_id{0};
	char _port[20]{};
	int _fd{-1};
	ParseState _parse_state{ParseState::WaitHeader};
	uint8_t _distance_low{0};
	uint8_t _distance_high{0};
	px4::atomic<uint16_t> _last_distance_mm{NO_TARGET_MM};
	px4::atomic<hrt_abstime> _last_sample{0};
	px4::atomic<uint32_t> _rx_byte_count{0};
	px4::atomic<uint32_t> _header_5a_count{0};
	px4::atomic<uint32_t> _header_5c_count{0};
	px4::atomic<uint32_t> _header_55_count{0};
	px4::atomic<uint8_t> _rx_tail[RAW_DUMP_SIZE];
	hrt_abstime _last_open_attempt{0};
	hrt_abstime _last_start_command{0};
	hrt_abstime _last_velocity_sample{0};
	float _last_velocity_distance_m{NAN};
	float _closing_speed_m_s{NAN};
	uORB::Publication<sdm50_status_s> _sdm50_status_pub{ORB_ID(sdm50_status)};
	perf_counter_t _sample_perf{perf_alloc(PC_COUNT, MODULE_NAME ": samples")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": communication errors")};
};
