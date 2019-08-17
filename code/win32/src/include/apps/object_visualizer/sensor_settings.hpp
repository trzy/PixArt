#pragma once
#ifndef INCLUDED_SENSOR_SETTINGS_HPP
#define INCLUDED_SENSOR_SETTINGS_HPP

#include "pixart/settings.hpp"

class i_serial_device;

pixart::settings read_sensor_settings(i_serial_device *port, bool print_settings);
void write_sensor_settings(i_serial_device *port, const pixart::settings &settings);

#endif  // INCLUDED_SENSOR_SETTINGS_HPP
