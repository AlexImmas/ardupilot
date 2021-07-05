// Gneeric Adaptive Feedback Linearization class - 4 dof
#pragma once

#include <AP_HAL/HAL.h>
#include <AP_Logger/AP_Logger.h> 

#include <eigen-3.3.9/Eigen/Dense>
//using namespace Eigen;


class AC_AFLC_4D {
public:

    // Constructore for AFLC
    AC_AFLC_4D(int n, float t, 
    float b1, float b2, float b3, float b4, 
    float l1, float l2, float l3, float l4, 
    float u1, float u2, 
    float c11, float c12, float c13, float c14, 
    float g);

    // Build reference model matrices
    void init_reference_model(Eigen::Vector4f eta_r0, Eigen::Vector4f deta_r0);

    // Update reference model states
    void update_reference_model(Eigen::Vector4f target);

    // Update transformation matrices
    void update_transformation_matrices(Eigen::Vector4f eta, Eigen::Vector4f nu);
    //Eigen::Matrix4f update_transformation_matrices(Eigen::Vector4f eta, Eigen::Vector4f nu);

    // Update anti windup filter
    void update_anti_windup(Eigen::Vector4f eta);

    // Compute commanded acceleration 
    void update_commanded_acceleration(Eigen::Vector4f eta, Eigen::Vector4f deta, Eigen::Vector4f nu);

    // Parameters update law
    void update_parameters_law(Eigen::Vector4f eta, Eigen::Vector4f deta, Eigen::Vector4f nu);

    // Compute control input
    void update_control_input();
    void update_control_input_na(Eigen::Vector4f nu);

     // Get states
    Eigen::Vector4f get_ref_position() const;
    Eigen::Vector4f get_ref_velocity() const;
    Eigen::Vector4f get_ref_acceleration() const;

    // Get control inpuots
    Eigen::Vector4f get_control_input() const;

    // Log data
    void logdata();

protected:

     // Control Parameters
    int _N;
    float _dt;        // timesteps in second
    float _lambda1, _lambda2, _lambda3, _lambda4;
    Eigen::Vector4f _lambda; // Control gains
    float _uU;     // upper bound on control input
    float _uL;      // lower bound on control input

    // Adaptive parameters
    float _c11, _c12, _c13, _c14;
    Eigen::Vector4f _c1;
    float _gain;
    Eigen::MatrixXf _gamma;

    // Transformation matrices
    Eigen::Matrix4f _J;      // Transformation matrix
    Eigen::Matrix4f _dJ;     // Derivative of trasnformation matrix

    // Control Variables
    Eigen::Vector4f _q;      // Integrator term
    Eigen::Vector4f _u;      // control input
    Eigen::Vector4f _Iawp;   // Anti windup filter
    Eigen::Vector4f _a_eta;  // commanded acceleration in body-fixed frame
    Eigen::Vector4f _a_nu;   // commanded acceleration in earth-fixed frame


    // Adaptive variables
    Eigen::VectorXf _theta;  // Parameters estimate
    Eigen::MatrixXf _Phi;
    Eigen::Vector4f _s; 
    Eigen::VectorXf _dtheta;

    // Reference model Parameters
    float _beta1, _beta2, _beta3, _beta4;
    Eigen::Vector4f _beta;  // bandwidth

    // Reference model Matrices
    Eigen::MatrixXf _A;           
    Eigen::MatrixXf _B; 

    // Reference model States       
    Eigen::Vector4f _eta_r;           // reference position
    Eigen::Vector4f _deta_r;          // reference velocity
    Eigen::Vector4f _d2eta_r;         // reference acceleration
};


