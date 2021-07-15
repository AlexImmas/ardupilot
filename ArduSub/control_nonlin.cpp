// Ardusub Adaptive Feedback Linearization Controller - 4 DOF
// GPS required
// June 2021

#include "Sub.h"

// aflc4_init - initialize nonlinear controller
bool Sub::nonlin_init()
{
  
    // // fail to initialise AFLC mode if no GPS lock 
    // if (!position_ok()) {
    //     return false;
    // }
    printf("nonlin commands: %u\n", mission.num_commands());
    if (!position_ok() || mission.num_commands() < 2) {
        return false;
    }

    // Init reference model
    nonlin_control.init_nonlin_control();  // ADD INITIALIZATION OF ETA_R, etc.. HERE

    // start/resume the mission (based on MIS_RESTART parameter)
    mission.start_or_resume();

    return true;
}

// aflc4_run - run nonlinear controller
void Sub::nonlin_run()
{

    mission.update();

    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        // initialise velocity controller
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        // pos_control.init_z_controller();
        // pos_control.init_xy_controller();
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

    // log data
    nonlin_control.logdata();

    // Send to motors
    nonlin_control.update_output();

}

// nonlin_set_destination - sets nonlinear mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Sub::nonlin_set_destination(const Vector3f& destination)
{
printf("running nonlin_set_destination_dest\n");
#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    //const Location dest_loc(destination, Location::AltFrame::ABOVE_ORIGIN);
    const Location dest_loc(destination);
    if (!fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // no need to check return status because terrain data is not used
    nonlin_control.update_target(destination);

    // log target
    //Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

bool Sub::nonlin_set_destination(const Location& dest_loc)
{
printf("running nonlin_set_destination_loc\n");

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!nonlin_control.update_target_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // log target
    // Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;


}