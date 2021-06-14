// Gneeric Adaptive Feedback Linearization class - 4 dof
#pragma once

#include <Eigen/Dense>
using namespace Eigen;


class AC_AFLC_4D {
public:

    // Constructore for AFLC
    AC_AFLC_4D(int n, float t, Vector4f b, Vector4f l, float u1, float u2, 
        Vector4f c, MatrixXf g, Vector4f pos_init, Vector4f vel_init);

    // Build reference model matrices
    void build_reference_model();

    // Update reference model states
    void update_reference_model(Vector4f target);

    // Update transformation matrices
    void update_transformation_matrices(Vector4f eta, Vector4f nu);

    // Update anti windup filter
    void update_anti_windup(Vector4f eta);

    // Compute commanded acceleration 
    void update_commanded_acceleration(Vector4f eta, Vector4f deta, Vector4f nu);

    // Parameters update law
    void update_parameters_law(Vector4f eta, Vector4f deta, Vector4f nu);

    // Compute control input
    void update_control_input();

     // Get states
    Vector4f get_ref_position() const;
    Vector4f get_ref_velocity() const;
    Vector4f get_ref_acceleration() const;

    // Get control inpuots
    Vector4f get_control_input() const;

protected:

    // Control Parameters
    int _N;
    float _dt;        // timesteps in second
    Vector4f _lambda; // Control gains
    float _uU;     // upper bound on control input
    float _uL;      // lower bound on control input

    // Adaptive parameters
    Vector4f _c1;
    MatrixXf _gamma;

    // Transformation matrices
    Matrix4f _J;      // Transformation matrix
    Matrix4f _dJ;     // Derivative of trasnformation matrix

    // Control Variables
    Vector4f _q;      // Integrator term
    Vector4f _u;      // control input
    Vector4f _Iawp;   // Anti windup filter
    Vector4f _a_eta;  // commanded acceleration in body-fixed frame
    Vector4f _a_nu;   // commanded acceleration in earth-fixed frame


    // Adaptive variables
    VectorXf _theta;  // Parameters estimate
    MatrixXf _Phi;
    Vector4f _s; 
    VectorXf _dtheta;

    // Reference model Parameters
    Vector4f _beta;  // bandwidth

    // Reference model Matrices
    MatrixXf _A;           
    MatrixXf _B; 

    // Reference model States       
    Vector4f _eta_r;           // reference position
    Vector4f _deta_r;          // reference velocity
    Vector4f _d2eta_r;         // reference acceleration





};


