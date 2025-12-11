#ifndef PID_CONSTANTS_HPP
#define PID_CONSTANTS_HPP

#include <string>

namespace pid_constants {
    // PID config node parameter names
    const std::string P_GAIN_NAME = "p_gain";
    const std::string I_GAIN_NAME = "i_gain";
    const std::string D_GAIN_NAME = "d_gain";
    const std::string I_MAX_NAME = "i_max";
    const std::string I_MIN_NAME = "i_min";
    const std::string EFFORT_CLAMP_NAME = "effort_clamp";
    const std::string DEADBAND_THRESHOLD_NAME = "deadband_threshold";

    // Define PID default configuration
    // Gains
    const double P_GAIN = 0.5;
    const double I_GAIN = 0.3;
    const double D_GAIN = 0.005;
    // Limit integral windup effects
    const double I_MAX = 0.1;
    const double I_MIN = -0.1; 
    const bool ANTI_WINDUP = false;  // Constrain integral contribution to control output (false) or constrain integral error (true) 
    // Constrain effort from PID output
    const double MAX_ABS_EFFORT = 0.3;
    // Ignore error when its absolute value is lower than defined threshold 
    const double DEADBAND_THRESHOLD = 0.03;
}

#endif