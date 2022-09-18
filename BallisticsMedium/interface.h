#ifndef INTERFACE_H
#define INTERFACE_H

#include <filesystem>

namespace a3_interface
{
	using vec3 = std::tuple<double, double, double>;

	std::shared_ptr<std::tuple<double, double, double>> find_solution(std::tuple<double, double> target_range,
	                                                                  vec3 target_velocity,
	                                                                  vec3 target_acceleration, double init_speed,
	                                                                  double air_friction);

	void init();

	void cleanup();
}


#endif // INTERFACE_H
