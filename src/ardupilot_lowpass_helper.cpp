// SPDX-License-Identifier: GPL-3.0-or-later

// The EKF-enabled plugin gets these math functions from DAL. Non-DAL smoke
// executables link this archive instead, without adding a second definition
// to the plugin's final link.
#define HAKONIWA_AP_STANDALONE_MATH_ONLY 1
#include "ardupilot_runtime_shim.cpp"
