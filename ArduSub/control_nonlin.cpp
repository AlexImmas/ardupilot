// Ardusub Adaptive Feedback Linearization Controller - 4 DOF
// GPS required
// June 2021

#include "Sub.h"

// aflc4_init - initialize nonlinear controller
bool Sub::nonlin_init()
{
    // fail to initialise AFLC mode if no GPS lock 
    if (!position_ok()) {
        return false;
    }

    // Init reference model
    nonlin_control.init_nonlin_control();  // ADD INITIALIZATION OF ETA_R, etc.. HERE

    return true;
}

// aflc4_run - run nonlinear controller
void Sub::nonlin_run()
{

    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        // initialise velocity controller
        pos_control.init_z_controller();
        pos_control.init_xy_controller();
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // update target point from path planner: TBD. Brute force targer point in the mean time
    // ....

    // Update state
    nonlin_control.update_state();

    // run nonlinear control
    nonlin_control.update_nonlin_control();

    // Send to motors
    nonlin_control.update_output();

}
