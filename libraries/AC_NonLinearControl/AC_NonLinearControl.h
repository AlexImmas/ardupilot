#pragma once

// @file AC_NonLinearControl.h
// Ardusub nonlinear control library

#include <AP_Common/AP_Common.h>               
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>              // common vehicle parameters
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>                // motors library
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_InertialNav/AP_InertialNav.h>      // Inertial Navigation library
#include <AC_AFLC/AC_AFLC_4D.h>


// Controller parameters (used in constructor)
#define POSCONTROL_DT_400HZ                     0.0025f // time difference in seconds for 400hz update rate


class AC_NonLinearControl{
public:

    // Constructor
    AC_NonLinearControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
        const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

    // Empty destructor to suppress compiler warning
    //virtual ~AC_NonLinearControl() {}

    // Get state
    void get_state();

    // Set target
    void set_target();


protected:

    // Parameters
    float       _dt;                    // time difference (in seconds) between calls from the main program

    // Variables
    Vector4f    _target;            // target location in cm from home
    Vector4f    _eta;
    Vector4f    _deta;
    Vector4f    _nu;


    // references to inertial nav and ahrs libraries
    AP_AHRS_View &                  _ahrs;
    const AP_InertialNav &          _inav;
    AP_MotorsMulticopter &          _motors_multi; //const AP_Motors &               _motors;
    const AP_Vehicle::MultiCopter & _aparm;




}