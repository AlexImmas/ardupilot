#include "AC_NonLinearControl.h"

AC_NonLinearControl::AC_NonLinearControl(AP_AHRS_View & ahrs, const AP_InertialNav & inav,
    const AP_MotorsMulticopter & motors, const AP_Vehicle::MultiCopter & aparm) :
        _ahrs(ahrs),
        _inav(inav),
        _motors(motors),
        _aparm(aparm),

        // attitude control
        _dt(POSCONTROL_DT_400HZ),
        // controller parameters
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

