#include "SDM50.hpp"

#include <cstdlib>
#include <cstring>

#include <px4_platform_common/getopt.h>

int SDM50::task_spawn(int argc, char *argv[])
{
	const char *device_path = "/dev/ttyS6";
	uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch = 0;

	while ((ch = px4_getopt(argc, argv, "d:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		case 'R': {
			const int parsed_rotation = atoi(myoptarg);

			if (parsed_rotation < 0 || parsed_rotation > 100) {
				return print_usage("invalid rotation");
			}

			rotation = static_cast<uint8_t>(parsed_rotation);
			break;
		}

		default:
			return print_usage("invalid option");
		}
	}

	SDM50 *instance = new SDM50(device_path, rotation);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}
int SDM50::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return print_usage("module not running");
	}

	if (!strcmp(argv[0], "range")) {
		return get_instance()->print_range();
	}

	return print_usage("unknown command");
}

int SDM50::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Siman SDM50 50 m laser rangefinder driver (460800 baud, 8N1).");
	PRINT_MODULE_USAGE_NAME("sdm50", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS6", nullptr, "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('R', distance_sensor_s::ROTATION_FORWARD_FACING, 0, 100, "Sensor rotation", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("range", "Print the latest measured distance");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int sdm50_main(int argc, char *argv[])
{
	return SDM50::main(argc, argv);
}
