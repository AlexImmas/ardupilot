#pragma once

// @file AC_NonLinearControl.h
// Ardusub nonlinear control library


//#include <AP_Common/AP_Common.h>      
//#define ALLOW_DOUBLE_MATH_FUNCTIONS
//#include <AP_HAL/HAL.h>
#include <AC_AFLC/AC_AFLC.h>
#include <AP_Logger/AP_Logger.h>         
#include <AP_Param/AP_Param.h>
//#include <AP_Vehicle/AP_Vehicle.h>              // common vehicle parameters
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_Motors.h>                // motors library
//#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_InertialNav/AP_InertialNav.h>      // Inertial Navigation library
#include <AP_Common/Location.h>
#include <stdio.h>
// #pragma push_macro("_GLIBCXX_USE_C99_STDIO")
// #undef _GLIBCXX_USE_C99_STDIO
// //#include <eigen-3.3.9/Eigen/Core>
//#include <eigen-3.3.9/Eigen/Dense>
// #pragma pop_macro("_GLIBCXX_USE_C99_STDIO")
//using namespace Eigen;


// Controller parameters (used in constructor)
#define N0                    4                // Model dof
#define BETA1                 0.1f              // Reference model bandwidth in surge
#define BETA2                 0.1f             // Reference model bandwidth in sway
#define BETA3                 0.08f            // Reference model bandwidth in heave
#define BETA4                 0.4f              // Reference model bandwidth in yaw
#define LAMBDA1               1.4f //1.0f            // Pole placement gain in surge
#define LAMBDA2               1.4f //1.0f          // Pole placement gain in sway
#define LAMBDA3               1.4f            // Pole placement gain in heave
#define LAMBDA4               10.0f              // Pole placement gain in yaw
#define UMAX                  10000.0f // //500.0f //50.0f            // Maximum control authority    
#define UMIN                  -10000.0f // //-500.0f //-50.0f           // Minimum control authority
#define C1                    3.0f             // Adaptive law parameter in surge
#define C2                    3.0f             // Adaptive law parameter in sway
#define C3                    1.0f             // Adaptive law parameter in heave
#define C4                    0.5f             // Adaptive law parameter in yaw
#define D1                    1.0f             // Adaptive law parameter in surge
#define D2                    1.0f             // Adaptive law parameter in sway
#define D3                    1.0f             // Adaptive law parameter in heave
#define D4                    1.0f             // Adaptive law parameter in yaw
#define M                     10               // number of adaptive parameters
#define GAMMA                 100000.0f           // adaptive law gain
#define WP_RADIUS             20.0f      // default waypoint radius in cm


class AC_NonLinearControl{
public:

    // Constructor
    AC_NonLinearControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                            AP_Motors & motors, float dt, float u1);

    // Empty destructor to suppress compiler warning
    //virtual ~AC_NonLinearControl() {}

    // Initialize control
    void init_nonlin_control();

    // Run control
    void update_nonlin_control();

    // Update control output
    void update_output();

    // Update target
    bool update_target(const Vector3f& destination);
    bool update_target_loc(const Location& destination);

    /// reached_destination - true when we have come within RADIUS cm of the waypoint
    bool reached_wp_destination();

    // Update state
    void update_state();

    // Update rotation amtrix
    void update_rot_matrix(float & roll, float & pitch, float & yaw);

    // Convert a 321-intrinsic euler angle derivative to an angular velocity vector
    void euler_rate_to_ang_vel(const Eigen::Vector3f& euler_rad, const Eigen::Vector3f& euler_rate_rads, Eigen::Vector3f& ang_vel_rads);

    // Convert an angular velocity vector to a 321-intrinsic euler angle derivative
    bool ang_vel_to_euler_rate(const Eigen::Vector3f& euler_rad, const Eigen::Vector3f& ang_vel_rads, Eigen::Vector3f& euler_rate_rads);

    // convert a vector from body to earth frame
    Vector3f body_to_earth(const Vector3f &v) const ;

    // convert a vector from earth to body frame
    Vector3f earth_to_body(const Vector3f &v) const ;

    void logdata();

    static const struct AP_Param::GroupInfo var_info[];


protected:

    // Parameters
    float       _dt;                    // time difference (in seconds) between calls from the main program
    int         _n;                      // Model dof
    float       _u1;                    // Max control input (for scaling)

    // Variables
    Eigen::Vector4f _target;// target location in cm from home
    Eigen::Vector4f _eta;           // _eta(1:3)  : AUV's position in NEU frame in cm relative to home (pos where UUV is armed)
                              // _eta(4:N)  : AUV's euler angles (rad) 
    Eigen::Vector4f _nu;             // _nu(1:3)   : AUV's body fixed linear velocity
                              // _nu(4:N)   : AUV's body fixed angular velocity
    Eigen::Vector4f _deta;           // _deta(1:3) : AUV's velocity in NEU frame (cm/s)
                              // _deta(4:N) : AUV's euler rate vector
    Eigen::Vector4f _tau;   // control input (N) 

    // Transformation matrices
    Matrix3f _rot_mat;         // Rotation matrix J: _deta(1:3) = _rot_mat * _nu(1:3)
    bool _reached_destination;

    // references to control libraries
    AC_AFLC _AFLC;

    // references to inertial nav and ahrs libraries
    AP_AHRS_View &                  _ahrs;
    const AP_InertialNav &          _inav;
    AP_Motors &               _motors; //AP_MotorsMulticopter &          _motors_multi; 




};