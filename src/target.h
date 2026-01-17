#pragma once

// Target selection for sensor hub hardware configuration
// This file includes the appropriate target configuration based on build flags

// Include the target configuration file based on build flag
#if defined(USE_TARGET_full_config)
    #include "targets/full_config.h"
#else
    // Default target
    #include "targets/default.h"
#endif

// Validate configuration
#if !defined(TARGET_NAME)
    #error "TARGET_NAME not defined in target configuration"
#endif

#if !defined(I2C_SDA_PIN) || !defined(I2C_SCL_PIN)
    #error "I2C pins not defined in target configuration"
#endif

#if NUM_FANS < 1 || NUM_FANS > 2
    #error "NUM_FANS must be 1 or 2"
#endif
