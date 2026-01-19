#pragma once

// Firmware version information
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0

// Build date/time (automatically set at compile time)
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__

// Version string
#define FW_VERSION_STRING "2.1.0"

// Project information
#define FW_PROJECT_NAME "Robot Sensor Hub"
#define FW_PROTOCOL_VERSION "1.0"

// Build a version info string at compile time
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define FW_VERSION_FULL FW_VERSION_STRING " (" FW_BUILD_DATE " " FW_BUILD_TIME ")"
