// Generic Reference Model Class
#include <iostream>
#include <math.h>    
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>
using namespace std;
using namespace Eigen;

#include "AC_AFLC_4D.h"
#include "AC_RefModel.h"

AC_AFLC_4D::AC_AFLC_4D(int n, float t, Vector4f b, Vector4f l, float u1, float u2, 
    Vector4f c, MatrixXf g, Vector4f pos_init, Vector4f vel_init)
    : _N(n),
      _dt(t),
      _lambda(l),
      _uU(u1),
      _uL(u2),
      _c1(c),
      _gamma(g),
      _beta(b),
      _eta_r(pos_init),
      _deta_r(vel_init),
      _d2eta_r(Vector4f::Zero(4))
    {
        // Initialize transformation matrices 
        _J  =  Matrix4f::Identity(4,4);  
        _dJ =  Matrix4f::Zero(4,4);  

        // Initialize control variables         
        _q    = Vector4f::Zero(4);
        _u    = Vector4f::Zero(4);

        // Initialize adaptive variables
        _theta = VectorXf::Zero(10);
        _Phi = MatrixXf::Zero(4,10); 

        // Initialize reference model matrices
        MatrixXf _A(12,12);           
        MatrixXf _B(12,4); 
    }


// Build reference model matrices
void AC_AFLC_4D::build_reference_model()
{

    //// Compute matrices for continuous-time model
    // Matrix Ac
    MatrixXf Ac(3*_N,3*_N);      
    
    // Upper block of rows
    Ac.block(0,0,_N,_N) = MatrixXf::Zero(_N,_N); 
    Ac.block(0,_N,_N,_N) = MatrixXf::Identity(_N,_N);
    Ac.block(0,2*_N,_N,_N) = MatrixXf::Zero(_N,_N);

    // Medium block of rows
    Ac.block(_N,0,_N,_N) = MatrixXf::Zero(_N,_N);
    Ac.block(_N,_N,_N,_N) = MatrixXf::Zero(_N,_N);
    Ac.block(_N,2*_N,_N,_N) = MatrixXf::Identity(_N,_N);

    // Lower block of rows
    Ac.block(2*_N,0,_N,_N).diagonal() = - (_beta.cwiseProduct(_beta)).cwiseProduct(_beta);
    Ac.block(2*_N,_N,_N,_N).diagonal() = - 3 * _beta.cwiseProduct(_beta);
    Ac.block(2*_N,2*_N,_N,_N).diagonal() = -3 * _beta;

    // Matrix Bc
    MatrixXf Bc(3*_N,_N); 
    Bc.block(0,0,_N,_N) = MatrixXf::Zero(_N,_N);
    Bc.block(_N,0,_N,_N) = MatrixXf::Zero(_N,_N);
    Bc.block(2*_N,0,_N,_N).diagonal() = (_beta.cwiseProduct(_beta)).cwiseProduct(_beta);

    //// Compute matrices for discrete-time model
    // Matrix A
    MatrixXf Acdt(3*_N,3*_N);  
    Acdt = _dt * Ac;
    _A = Acdt.exp();

    // MAtrix B
    _B = Ac.inverse()*(_A-MatrixXf::Identity(3*_N,3*_N))*Bc;

}

void AC_AFLC_4D::update_reference_model(Vector4f target)
{
    // Delare state-space vector
    VectorXf y(3*_N);
    y.head(_N) = _eta_r;
    y.segment(_N,_N) = _deta_r;
    y.tail(_N) = _d2eta_r;

    // Update state-space vector
    y = _A * y + _B * target;

    // Assign update to parameters objects
    _eta_r = y.head(_N);
    _deta_r = y.segment(_N,_N);
    _d2eta_r = y.tail(_N);
}

// Update transformation matrices
void AC_AFLC_4D::update_transformation_matrices(Vector4f eta, Vector4f nu)
{
    _J(0,0) = cos(eta(3));
    _J(0,1) = -sin(eta(3));
    _J(1,0) = sin(eta(3));
    _J(1,1) = cos(eta(3));

    _dJ(0,0) = -sin(eta(3))*nu(3);
    _dJ(0,1) = -cos(eta(3))*nu(3);
    _dJ(1,0) = cos(eta(3))*nu(3);
    _dJ(1,1) = -sin(eta(3))*nu(3);
}

// Anti windup
void AC_AFLC_4D::update_anti_windup(Vector4f eta)
{
    _Iawp = Vector4f::Ones(4);

    for(int i=0; i<4; i++){
        // Saturated and tending to be more saturated -> Stop integration
        if (_u(i) >= _uU && eta(i)-_eta_r(i)<=0){ 
            _Iawp(i) = 0;
        } else if(_u(i) <= _uL && eta(i)-_eta_r(i)>=0) {
            _Iawp(i) = 0;
        } 
    }
}

// Compute commanded acceleration 
void AC_AFLC_4D::update_commanded_acceleration(Vector4f eta, Vector4f deta, Vector4f nu)
{
    // I-term
    _q += _dt * _Iawp.cwiseProduct(eta-_eta_r);

    // Pole placement algorithm
    _a_eta = _d2eta_r - 3 * _lambda.cwiseProduct(deta-_deta_r) 
                - 3* (_lambda.cwiseProduct(_lambda)).cwiseProduct(eta-_eta_r)
                - ((_lambda.cwiseProduct(_lambda)).cwiseProduct(_lambda)).cwiseProduct(_q);

    // CHange of reference frame
    _a_nu = _J.inverse()*(_a_eta-_dJ*nu);

}

// Parameters update law
void AC_AFLC_4D::update_parameters_law(Vector4f eta, Vector4f deta, Vector4f nu)
{
    _Phi(0,0) = _a_nu(0)-nu(3)*nu(1);
    _Phi(0,1) = _a_nu(0);
    _Phi(0,2) = -nu(1)*nu(3);
    _Phi(0,6) = abs(nu(0))*nu(0);

    _Phi(1,0) = _a_nu(1)+nu(3)*nu(0);
    _Phi(1,1) = nu(0)*nu(3);
    _Phi(1,2) = _a_nu(1);
    _Phi(1,7) = abs(nu(1))*nu(1);

    _Phi(2,0) = _a_nu(2);
    _Phi(2,3) = _a_nu(2);
    _Phi(2,8) = abs(nu(2))*nu(2);

    _Phi(3,1) = -nu(0)*nu(1);
    _Phi(3,2) = nu(1)*nu(0);
    _Phi(3,4) = _a_nu(3);
    _Phi(3,5) = _a_nu(3);
    _Phi(3,9) = abs(nu(3))*nu(3);

    // change of variables
    _s = (deta - _deta_r) + _c1.cwiseProduct(eta-_eta_r);

    // Parameters update law
    _dtheta = - _gamma.inverse() * _Phi.transpose() * _J.inverse() * _s;
}

// Compute control input
void AC_AFLC_4D::update_control_input()
{
    // Update parameters
    _theta += _dtheta * _dt;

    // Compute control input
    _u = _Phi * _theta;

    // Thrust limitation
    for(int i = 0; i<4; i++)
    {
        if(_u(i) >= _uU){
            _u(i) = _uU;
        } else if (_u(i) <= _uL){
            _u(i) = _uL;
        }
    }
}

Vector4f AC_AFLC_4D::get_ref_position() const
{
    return _eta_r;
}

Vector4f AC_AFLC_4D::get_ref_velocity() const
{
    return _deta_r;
}

Vector4f AC_AFLC_4D::get_ref_acceleration() const
{
    return _d2eta_r;
}

Vector4f AC_AFLC_4D::get_control_input() const
{
    return _u;
}









