#include <iostream>
using namespace std;
#include <AP_Math/AP_Math.h>
#include "AC_AFLC_4D.h"


int main()
{
    // Positions
    N = 4
    VectorN<float,N> eta_init(3.0f,2.0f,1.0f,M_PI/6);
    VectorN<float,N> nu_init(0.5f,-0.5f,0.2f,0.3f);
    VectorN<float,N> target(10.0f,10.0f,10.0f,10.0f);

    // Matrix4f J  =  Matrix4f::Identity(4,4);
    // J(0,0) = cos(eta_init(3));
    // J(0,1) = -sin(eta_init(3));
    // J(1,0) = sin(eta_init(3));
    // J(1,1) = cos(eta_init(3));
    // Vector4f deta_init = J*nu_init;

    // // Parameters
    // int N = 4;
    // float dt = 0.1;
    // Vector4f beta(1,2,3,4);
    // Vector4f lambda(0.4,0.4,0.7,4);
    // float u1 = 50;
    // float u2 = -50;
    // Vector4f c(3,3,1,1);
    // MatrixXf g = 10 * MatrixXf::Identity(10,10);
 
    // //// Initialization 
    // AC_AFLC_4D AC_AFLC_4D(N, dt, beta, lambda, u1, u2, c, g, eta_init, deta_init);
    // // Building reference model matrices
    // AC_AFLC_4D.build_reference_model();

    // //// In time loop
    // for(int time = 0; time<2; time++)
    // {
    //   // Update transformation matrices
    //   AC_AFLC_4D.update_transformation_matrices(eta_init, nu_init);
    //   // Update reference position
    //   AC_AFLC_4D.update_reference_model(target);
    //   //  Anti windup
    //   AC_AFLC_4D.update_anti_windup(eta_init);
    //   // Update commanded acceleration
    //   AC_AFLC_4D.update_commanded_acceleration(eta_init, deta_init, nu_init);
    //   // Update parameters law
    //   AC_AFLC_4D.update_parameters_law(eta_init, deta_init, nu_init);
    //   // Compute control input
    //   AC_AFLC_4D.update_control_input();
    //   // Get control input
    //   cout << "---------------------" << endl;
    //   cout << "Control input" << endl;
    //   cout << AC_AFLC_4D.get_control_input() << endl;;
    // }


    return 0;
}
 