// Generic Reference Model Class - 4 dof
#pragma once

#include <Eigen/Dense>
using namespace Eigen;

class AC_RefModel {
public:

    // friend class
    friend class AC_AFLC_4D;

    // Constructore for RefModel
    AC_RefModel(int n, Vector4f b, float t, Vector4f e, Vector4f de);

    // Build reference model matrices
    void build_reference_model();

    // Update reference model states
    void update_reference_model(Vector4f target);

    // Get states
    Vector4f get_position() const;
    Vector4f get_velocity() const;
    Vector4f get_acceleration() const;

    // Debug
    MatrixXf get_matrixA() const;
    MatrixXf get_matrixB() const;

private:

    // State dimension
    int _N;

    // Parameters
    Vector4f _beta;  // bandwidth
    float _dt;          // timesteps in second

    // Reference model Matrices
    MatrixXf _A;           
    MatrixXf _B; 

    // Reference model States       
    Vector4f _eta_r;           // reference position
    Vector4f _deta_r;          // reference velocity
    Vector4f _d2eta_r;         // reference acceleration


};
