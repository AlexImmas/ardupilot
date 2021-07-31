// Generic Reference Model Class
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unsupported/Eigen/MatrixFunctions>
using namespace std;
using namespace Eigen;

#include "AC_RefModel.h"


// Constructure for Reference Model class
AC_RefModel::AC_RefModel(int n, Vector4f b, float t, Vector4f e, Vector4f de)
    : _N(n),
      _beta(b),
      _dt(t),
      _eta_r(e),
      _deta_r(de),
      _d2eta_r(Vector4f::Zero(4))
      {
        MatrixXf _A(12,12);           
        MatrixXf _B(12,4); 
      }

// Build reference model matrices
void AC_RefModel::build_reference_model()
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

void AC_RefModel::update_reference_model(Vector4f target)
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

Vector4f AC_RefModel::get_position() const
{
    return _eta_r;
}

Vector4f AC_RefModel::get_velocity() const
{
    return _deta_r;
}

Vector4f AC_RefModel::get_acceleration() const
{
    return _d2eta_r;
}

 // Debug
MatrixXf AC_RefModel::get_matrixA() const
{
    return _A;
}

MatrixXf AC_RefModel::get_matrixB() const
{
    return _B;
}

