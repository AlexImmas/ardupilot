// Ardusub Adaptive Feedback Linearization Controller - 4 DOF
// GPS required
// Alexandre Immas
// June 2021

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED

bool Sub::aflc4_init()
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok()) {
        return false;
    }

    // initialize controller
    nc_control.init_controller();

    // Initalize target to current position
    nc.control.set_target(); 

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

void Sub::aflc4_run()
{

    // ....
}



#endif  // POSHOLD_ENABLED == ENABLED