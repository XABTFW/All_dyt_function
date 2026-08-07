#include "SDM50.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>

SDM50::SDM50(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation)
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	for (auto &byte : _rx_tail) {
		byte.store(0);
	}

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_SDM50;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_LASER);
	_px4_rangefinder.set_min_distance(0.05f);
	_px4_rangefinder.set_max_distance(50.f);
	_px4_rangefinder.set_fov(math::radians(1.7f));
}
SDM50::~SDM50()
{
	ScheduleClear();
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int SDM50::init()
{
	ScheduleOnInterval(2_ms);
	return PX4_OK;
}

bool SDM50::open_serial()
{
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed (%i)", _port, errno);
		return false;
	}

	termios config{};

	if (tcgetattr(_fd, &config) != 0) {
		PX4_ERR("tcgetattr failed (%i)", errno);
		close_serial();
		return false;
	}

	cfmakeraw(&config);
	config.c_cflag |= CLOCAL | CREAD;
	config.c_cflag &= ~CSIZE;
	config.c_cflag |= CS8;
	config.c_cflag &= ~(PARENB | CSTOPB);
#ifdef CRTSCTS
	config.c_cflag &= ~CRTSCTS;
#endif
	config.c_cc[VMIN] = 0;
	config.c_cc[VTIME] = 0;

	if (cfsetispeed(&config, B460800) != 0 || cfsetospeed(&config, B460800) != 0 ||
	    tcsetattr(_fd, TCSANOW, &config) != 0) {
		PX4_ERR("configure 460800 8N1 failed (%i)", errno);
		close_serial();
		return false;
	}

	tcflush(_fd, TCIOFLUSH);
	_parse_state = ParseState::WaitHeader;
	_distance_low = 0;
	_distance_high = 0;
	if (!send_start_command()) {
		close_serial();
		return false;
	}

	return true;
}

void SDM50::close_serial()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

bool SDM50::send_start_command()
{
	static constexpr uint8_t start_command[]{0x5A, 0x0A, 0x02, 0x02, 0x00, 0xF1};
	_last_start_command = hrt_absolute_time();
	return send_command(start_command, sizeof(start_command));
}

bool SDM50::send_command(const uint8_t *command, size_t length)
{
	const ssize_t written = ::write(_fd, command, length);

	if (written != static_cast<ssize_t>(length)) {
		PX4_ERR("write failed (%i)", errno);
		perf_count(_comms_errors);
		return false;
	}

	return true;
}

void SDM50::Run()
{
	if (should_exit()) {
		if (_fd >= 0) {
			static constexpr uint8_t stop_command[]{0x5A, 0x0A, 0x02, 0x00, 0x00, 0xF3};
			send_command(stop_command, sizeof(stop_command));
		}

		ScheduleClear();
		close_serial();
		exit_and_cleanup();
		return;
	}

	if (_fd < 0) {
		const hrt_abstime now = hrt_absolute_time();

		if (_last_open_attempt == 0 || now - _last_open_attempt >= 1_s) {
			_last_open_attempt = now;
			open_serial();
		}

		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime last_sample = _last_sample.load();
	const bool measurement_stale = last_sample == 0 || now < last_sample || now - last_sample >= 1_s;

	if (measurement_stale && (_last_start_command == 0 || now - _last_start_command >= 1_s) && !send_start_command()) {
		close_serial();
		return;
	}

	uint8_t buffer[128]{};
	const ssize_t bytes_read = ::read(_fd, buffer, sizeof(buffer));

	if (bytes_read > 0) {
		uint32_t rx_byte_count = _rx_byte_count.load();

		for (ssize_t i = 0; i < bytes_read; ++i) {
			_rx_tail[rx_byte_count % RAW_DUMP_SIZE].store(buffer[i]);
			_rx_byte_count.store(++rx_byte_count);

			if (buffer[i] == 0x5A) {
				_header_5a_count.fetch_add(1);

			} else if (buffer[i] == FRAME_HEADER) {
				_header_5c_count.fetch_add(1);

			} else if (buffer[i] == 0x55) {
				_header_55_count.fetch_add(1);
			}

			parse_byte(buffer[i]);
		}

	} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		perf_count(_comms_errors);
		close_serial();
	}
}

void SDM50::parse_byte(uint8_t byte)
{
	switch (_parse_state) {
	case ParseState::WaitHeader:
		if (byte == FRAME_HEADER) {
			_parse_state = ParseState::DistanceLow;
		}
		break;

	case ParseState::DistanceLow:
		_distance_low = byte;
		_parse_state = ParseState::DistanceHigh;
		break;

	case ParseState::DistanceHigh:
		_distance_high = byte;
		_parse_state = ParseState::Checksum;
		break;

	case ParseState::Checksum: {
		const uint8_t expected_checksum = static_cast<uint8_t>(~(_distance_low + _distance_high));

		if (byte == expected_checksum) {
			const uint16_t distance_mm = static_cast<uint16_t>(_distance_low | (_distance_high << 8));
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			const bool valid = distance_mm >= MIN_DISTANCE_MM && distance_mm < MAX_DISTANCE_MM;

			_last_distance_mm.store(distance_mm);
			_last_sample.store(timestamp_sample);
			_px4_rangefinder.update(timestamp_sample, static_cast<float>(distance_mm) * 0.001f, valid ? 100 : 0);
			perf_count(_sample_perf);
			_parse_state = ParseState::WaitHeader;

		} else {
			perf_count(_comms_errors);
			_parse_state = byte == FRAME_HEADER ? ParseState::DistanceLow : ParseState::WaitHeader;
		}
		break;
	}
	}
}

int SDM50::print_range()
{
	const hrt_abstime last_sample = _last_sample.load();

	if (last_sample == 0 || hrt_elapsed_time(&last_sample) > 500_ms) {
		PX4_WARN("no recent SDM50 measurement");
		return PX4_ERROR;
	}

	const uint16_t distance_mm = _last_distance_mm.load();

	if (distance_mm == NO_TARGET_MM || distance_mm < MIN_DISTANCE_MM || distance_mm >= MAX_DISTANCE_MM) {
		PX4_WARN("SDM50: no target");
		return PX4_ERROR;
	}

	PX4_INFO("SDM50 distance: %.3f m (%u mm)", static_cast<double>(distance_mm) * 0.001,
		 static_cast<unsigned>(distance_mm));
	return PX4_OK;
}

int SDM50::print_status()
{
	print_info();
	return PX4_OK;
}

void SDM50::print_info()
{
	PX4_INFO("port: %s, fd: %d, baud: 460800", _port, _fd);
	const uint32_t rx_byte_count = _rx_byte_count.load();
	PX4_INFO("rx bytes: %u, headers 5A/5C/55: %u/%u/%u",
		 static_cast<unsigned>(rx_byte_count),
		 static_cast<unsigned>(_header_5a_count.load()),
		 static_cast<unsigned>(_header_5c_count.load()),
		 static_cast<unsigned>(_header_55_count.load()));

	if (rx_byte_count > 0) {
		const unsigned available = rx_byte_count < RAW_DUMP_SIZE ? rx_byte_count : RAW_DUMP_SIZE;
		const unsigned start = (rx_byte_count - available) % RAW_DUMP_SIZE;
		char raw_line[RAW_DUMP_SIZE * 3]{};
		unsigned offset = 0;

		for (unsigned i = 0; i < available; ++i) {
			const uint8_t byte = _rx_tail[(start + i) % RAW_DUMP_SIZE].load();
			const int written = snprintf(raw_line + offset, sizeof(raw_line) - offset,
						     i + 1 < available ? "%02X " : "%02X", static_cast<unsigned>(byte));

			if (written <= 0) {
				break;
			}

			offset += static_cast<unsigned>(written);
		}

		PX4_INFO("last rx: %s", raw_line);
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	print_range();
}
