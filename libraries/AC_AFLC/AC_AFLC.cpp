// Generic Reference Model Class
////////// WATCH OUT //////////
// EXP MATRIX HAS BEEN COMMENTED OUT /////


//#define ALLOW_DOUBLE_MATH_FUNCTIONS
#include "AC_AFLC.h"
#include <math.h>   

#include <iostream> // COMMENT OUT FOR PIXHAWK COMPILATION

//#pragma push_macro("_GLIBCXX_USE_C99_STDIO")
//#undef _GLIBCXX_USE_C99_STDIO 
//#include <eigen-3.3.9/Eigen/Core>
// #include <eigen-3.3.9/Eigen/Dense>
// #include <eigen-3.3.9/Eigen/LU>
// #include <eigen-3.3.9/unsupported/Eigen/MatrixFunctions>
//#pragma pop_macro("_GLIBCXX_USE_C99_STDIO")

//using namespace Eigen;

// table of user settable parameters
const AP_Param::GroupInfo AC_AFLC::var_info[] = {

   // AP_GROUPINFO("P",    0, AC_NonLinearControl, _dt, 0),

    AP_GROUPEND
};


AC_AFLC::AC_AFLC(int n, float t, 
    float b1, float b2, float b3, float b4, 
    float l1, float l2, float l3, float l4, 
    float u1, float u2, 
    float c11, float c12, float c13, float c14, 
    float d11, float d12, float d13, float d14,
    float g)
     : _n(n),
       _dt(t),
       _beta1(b1), _beta2(b2), _beta3(b3), _beta4(b4),
      _lambda1(l1), _lambda2(l2), _lambda3(l3), _lambda4(l4), 
      _uU(u1),
      _uL(u2),
      _c11(c11), _c12(c12), _c13(c13), _c14(c14), 
      _d11(d11), _d12(d12), _d13(d13), _d14(d14), 
      _gain(g)
    {
        // Assign reference model bandwitch
        _beta << _beta1, _beta2, _beta3, _beta4;
        
        // Assign reference model bandwitch
        _lambda << _lambda1, _lambda2, _lambda3, _lambda4;
      
        // Assign adpater law paramenter
        _c1 << _c11, _c12, _c13, _c14;
        _d1 << _d11, _d12, _d13, _d14;

        // Assign adpative law gain
        _gamma = _gain * Eigen::MatrixXf::Identity(10,10);
        _gamma(4,4) = 0.005 * _gamma(4,4); // 0.01 
        _gamma(5,5) = 0.005 * _gamma(5,5); // 0.01
        _gamma(9,9) = 0.005 * _gamma(9,9); // 0.01

        // Initialize transformation matrices 
        _J  =  Eigen::Matrix4f::Identity(4,4);  
        _dJ =  Eigen::Matrix4f::Zero(4,4);  

        // Initialize control variables         
        _q    = Eigen::Vector4f::Zero(4);
        _u    = Eigen::Vector4f::Zero(4);

        // Initialize adaptive variables
        _theta = Eigen::VectorXf::Zero(10);
        _theta << 11.5, 5.5, 12.7, 14.57, 0.16, 0.12, 18.18, 21.66, 36.99, 1.55;
        //_theta = Eigen::Vector10f(11.5, 5.5, 12.7, 14.57, 0.16, 0.12, 18.18, 21.66, 36.99, 1.55);
        _Phi = Eigen::MatrixXf::Zero(4,10); 

        // Initialize reference model matrices
        _A = Eigen::MatrixXf::Zero(12,12);
        _Br = Eigen::MatrixXf::Zero(12,4);
        //MatrixXf _A(12,12);           
        //MatrixXf _B(12,4); 

        AP_Param::setup_object_defaults(this, var_info);
    }


// Build reference model matrices
void AC_AFLC::init_reference_model(Eigen::Vector4f eta_r0, Eigen::Vector4f deta_r0)
{

    //// Compute matrices for continuous-time model
    // Matrix Ac
    Eigen::MatrixXf Ac(3*_n,3*_n);      
    
    // Upper block of rows
    Ac.block(0,0,_n,_n) = Eigen::MatrixXf::Zero(_n,_n); 
    Ac.block(0,_n,_n,_n) = Eigen::MatrixXf::Identity(_n,_n);
    Ac.block(0,2*_n,_n,_n) = Eigen::MatrixXf::Zero(_n,_n);

    // Medium block of rows
    Ac.block(_n,0,_n,_n) = Eigen::MatrixXf::Zero(_n,_n);
    Ac.block(_n,_n,_n,_n) = Eigen::MatrixXf::Zero(_n,_n);
    Ac.block(_n,2*_n,_n,_n) = Eigen::MatrixXf::Identity(_n,_n);

    // Lower block of rows
    Ac.block(2*_n,0,_n,_n).diagonal() = - (_beta.cwiseProduct(_beta)).cwiseProduct(_beta);
    Ac.block(2*_n,_n,_n,_n).diagonal() = - 3 * _beta.cwiseProduct(_beta);
    Ac.block(2*_n,2*_n,_n,_n).diagonal() = -3 * _beta;

    // Matrix Bc
    Eigen::MatrixXf Bc(3*_n,_n); 
    Bc.block(0,0,_n,_n) = Eigen::MatrixXf::Zero(_n,_n);
    Bc.block(_n,0,_n,_n) = Eigen::MatrixXf::Zero(_n,_n);
    Bc.block(2*_n,0,_n,_n).diagonal() = (_beta.cwiseProduct(_beta)).cwiseProduct(_beta);

    //// Compute matrices for discrete-time model
    // Matrix A
    Eigen::MatrixXf Acdt(3*_n,3*_n);  
    Acdt = _dt * Ac;
    _A = Acdt.exp();

    // MAtrix B
    _Br = Ac.inverse()*(_A-Eigen::MatrixXf::Identity(3*_n,3*_n))*Bc;

    //// Initiazlie reference model states
    _eta_r = eta_r0;
    _deta_r = deta_r0;
    _d2eta_r = Eigen::Vector4f::Zero(4);

//     std::cout << "Ac:\n " << Ac << "\n";
//     std::cout << "Ac size: " << Ac.rows() << "x" << Ac.cols() << "\n";
//     std::cout << "Bc:\n " << Bc << "\n";
//     std::cout << "Ad:\n " << _A << "\n";
//     std::cout << "Bd:\n " << _Br << "\n";
    // printf("REF Model INIT\n");
    // printf("ref x:   %f cm\n", _eta_r(0));
    // printf("ref y:   %f cm\n", _eta_r(1));
    // printf("ref z:   %f cm\n", _eta_r(2));
    // printf("ref psi: %f degrees\n", _eta_r(3)*180/M_PI);
    // printf("ref dx:   %f cm/s\n", _deta_r(0));
    // printf("ref dy:   %f cm/s\n", _deta_r(1));
    // printf("ref dz:   %f cm/s\n", _deta_r(2));
    // printf("ref dpsi: %f degrees/s\n", _deta_r(3)*180/M_PI);

}

void AC_AFLC::update_reference_model(Eigen::Vector4f target)
{
    // Delare state-space vector
    Eigen::VectorXf y(3*_n);
    y.head(_n) = _eta_r;
    y.segment(_n,_n) = _deta_r;
    y.tail(_n) = _d2eta_r;

    // Update state-space vector
    y = _A * y + _Br * target;

    // Assign update to parameters objects
    _eta_r = y.head(_n);
    _deta_r = y.segment(_n,_n);
    _d2eta_r = y.tail(_n);

    // std::cout << "ref_model target:\n " << target << "\n";
}

// Update transformation matrices
void AC_AFLC::update_transformation_matrices(Eigen::Vector4f eta, Eigen::Vector4f nu)
//Eigen::Matrix4f AC_AFLC_4D::update_transformation_matrices(Eigen::Vector4f eta, Eigen::Vector4f nu)
{
    _J(0,0) = cos(eta(3));
    _J(0,1) = -sin(eta(3));
    _J(1,0) = sin(eta(3));
    _J(1,1) = cos(eta(3));

    _dJ(0,0) = -sin(eta(3))*nu(3);
    _dJ(0,1) = -cos(eta(3))*nu(3);
    _dJ(1,0) = cos(eta(3))*nu(3);
    _dJ(1,1) = -sin(eta(3))*nu(3);

    //return _J;
}

// Anti windup
void AC_AFLC::update_anti_windup(Eigen::Vector4f eta)
{
    _Iawp = Eigen::Vector4f::Ones(4);

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
void AC_AFLC::update_commanded_acceleration(Eigen::Vector4f eta, Eigen::Vector4f deta, Eigen::Vector4f nu)
{
    // I-term
    _q += _dt * _Iawp.cwiseProduct(eta-_eta_r);

    // Pole placement algorithm: 
    //_a_eta = _d2eta_r - 2 * _lambda.cwiseProduct(deta-_deta_r) - (_lambda.cwiseProduct(_lambda)).cwiseProduct(eta-_eta_r);

    _a_eta = _d2eta_r - 3 * _lambda.cwiseProduct(deta-_deta_r) 
                - 3* (_lambda.cwiseProduct(_lambda)).cwiseProduct(eta-_eta_r)
                - ((_lambda.cwiseProduct(_lambda)).cwiseProduct(_lambda)).cwiseProduct(_q);

    // CHange of reference frame
    _a_nu = _J.inverse()*(_a_eta-_dJ*nu);

}

// Parameters update law
void AC_AFLC::update_parameters_law(Eigen::Vector4f eta, Eigen::Vector4f deta, Eigen::Vector4f nu)
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

    // std::cout << "a_nu:\n " << _a_nu << "\n";
    // std::cout << "nu:\n " << nu << "\n";

    // change of variables
    _s = _d1.cwiseProduct(deta - _deta_r) + _c1.cwiseProduct(eta-_eta_r);

    // Parameters update law
    _dtheta = - _gamma.inverse() * _Phi.transpose() * _J.inverse() * _s;
}

// Compute control input
void AC_AFLC::update_control_input()
{
    // Update parameters
    _theta += _dtheta * _dt;

    // Compute control input
    _u = _Phi * _theta;

    // std::cout << "theta:\n " << _theta << "\n";
    // std::cout << "dtheta:\n " << _dtheta << "\n";
    // std::cout << "Phi:\n " << _Phi << "\n";


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

// Compute control input
void AC_AFLC::update_control_input_na(Eigen::Vector4f nu)
{

    // Model parameters
    float m    = 11.5;
    float I4   = 0.16;
    float ma11 = 5.5;
    float ma22 = 12.7;
    float ma33 = 14.57;
    float Ia4  = 0.12;
    float dx   = 18.18;
    float dy   = 21.66;
    float dz   = 36.99;
    float d4   = 1.55;

    // Model
    Eigen::Matrix4f M    =  Eigen::Matrix4f::Zero(4,4);  
    M(0,0) = m;
    M(1,1) = m;
    M(2,2) = m;
    M(3,3) = I4;

    Eigen::Matrix4f Ma   =  Eigen::Matrix4f::Zero(4,4);  
    M(0,0) = ma11;
    M(1,1) = ma22;
    M(2,2) = ma33;
    M(3,3) = Ia4;

    Eigen::Matrix4f Ccor =  Eigen::Matrix4f::Zero(4,4);  
    Ccor(0,1) = -m*nu(3);
    Ccor(1,0) = m*nu(3);

    Eigen::Matrix4f Ca   =  Eigen::Matrix4f::Zero(4,4);  
    Ca(0,3) = -ma22 * nu(1);
    Ca(1,3) = ma11 * nu(0);
    Ca(3,0) = ma22 * nu(1);
    Ca(3,1) = -ma11*nu(0);

    Eigen::Matrix4f Dv   =  Eigen::Matrix4f::Zero(4,4);  
    Dv(0,0) = dx *abs(nu(0));
    Dv(1,1) = dy *abs(nu(1));
    Dv(2,2) = dz *abs(nu(2));
    Dv(3,3) = d4 *abs(nu(3));

    // Compute thrust output
    _u = (M+Ma) * _a_nu + (Ccor + Ca) * nu + Dv * nu;

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

Eigen::Vector4f AC_AFLC::get_ref_position() const
{
    return _eta_r;
}

Eigen::Vector4f AC_AFLC::get_ref_velocity() const
{
    return _deta_r;
}

Eigen::Vector4f AC_AFLC::get_ref_acceleration() const
{
    return _d2eta_r;
}

Eigen::Vector4f AC_AFLC::get_control_input() const
{
    return _u;
}

void AC_AFLC::logdata(){

    AP::logger().Write("NLRP", "TimeUS,etarx,etary,etarz,etarpsi", "Qffff",
                                        AP_HAL::micros64(),
                                        (double)_eta_r(0),
                                        (double)_eta_r(1),
                                        (double)_eta_r(2),
                                        (double)_eta_r(3));

    AP::logger().Write("NLRV", "TimeUS,detarx,detary,detarz,detarpsi", "Qffff",
                                        AP_HAL::micros64(),
                                        (double)_deta_r(0),
                                        (double)_deta_r(1),
                                        (double)_deta_r(2),
                                        (double)_deta_r(3));

    AP::logger().Write("NLPA", "TimeUS,m,m11,m22,m33,Jx,m44,d1,d2,d3,d4", "Qffffffffff",
                                        AP_HAL::micros64(),
                                        (double)_theta(0),
                                        (double)_theta(1),
                                        (double)_theta(2),
                                        (double)_theta(3),
                                        (double)_theta(4),
                                        (double)_theta(5),
                                        (double)_theta(6),
                                        (double)_theta(7),
                                        (double)_theta(8),
                                        (double)_theta(9));
    AP::logger().Write("NLDT", "TimeUS,dm,dm11,dm22,dm33,dJx,dm44,dd1,dd2,dd3,dd4", "Qffffffffff",
                                        AP_HAL::micros64(),
                                        (double)_dtheta(0),
                                        (double)_dtheta(1),
                                        (double)_dtheta(2),
                                        (double)_dtheta(3),
                                        (double)_dtheta(4),
                                        (double)_dtheta(5),
                                        (double)_dtheta(6),
                                        (double)_dtheta(7),
                                        (double)_dtheta(8),
                                        (double)_dtheta(9));
    AP::logger().Write("AFLC", "TimeUS,qx,qy,qz,q4,Iawpx,Iawpy,Iawpz,Iawp4,anu1,anu2,anu3,anu4", "Qffffffffffff",
                                    AP_HAL::micros64(),
                                    (double)_q(0),
                                    (double)_q(1),
                                    (double)_q(2),
                                    (double)_q(3),
                                    (double)_Iawp(0),
                                    (double)_Iawp(1),
                                    (double)_Iawp(2),
                                    (double)_Iawp(3),
                                    (double)_a_nu(0),
                                    (double)_a_nu(1),
                                    (double)_a_nu(2),
                                    (double)_a_nu(3));
}



