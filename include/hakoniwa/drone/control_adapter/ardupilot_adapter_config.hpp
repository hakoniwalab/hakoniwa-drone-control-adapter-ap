// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

// The adapter uses AC_PID's scalar target/error/derivative filters. Dynamic
// notch filters belong to the complete ArduPilot vehicle runtime and depend on
// its AP_Filters singleton, so they are intentionally outside this backend.
#undef AP_FILTER_NUM_FILTERS
#define AP_FILTER_NUM_FILTERS 0
#undef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED 0
