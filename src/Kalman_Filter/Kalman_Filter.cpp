
/*
    Kalman_Filter Class

    Copyright 2018 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Kalman_Filter/Kalman_Filter.h"


using namespace TooN;
using namespace std;



/*===============CONSTRUCTORS===================*/
//COMPLETE CONSTRUCTOR
Kalman_Filter::Kalman_Filter(Vector<> initial_state, Matrix<> initial_covariance, const int outDim, KF_FCN f_fcn, KF_JAC_FCN FF_fcn, KF_FCN h_fcn, KF_JAC_FCN HH_fcn):
    x_hat_k_k(initial_state),
    P_k_k(initial_covariance),
    y_hat_k_k1(Zeros(outDim)),

    //Internal Vars
    x_hat_k1_k1(initial_state),
    P_k1_k1(initial_covariance),
    x_hat_k_k1(initial_state),
    F_k1(Zeros(initial_state.size(),initial_state.size())),
    Identity_x(Identity(initial_state.size())),
    P_k_k1(initial_covariance),
    y_tilde_k(Zeros(outDim)),
    H_k(Zeros(outDim,initial_state.size())),
    S_k(Zeros(outDim,outDim)),
    K_k(initial_state.size() ,outDim )

{ 

    set_state(initial_state);
    set_f_fcn(f_fcn);
    set_FF_fcn(FF_fcn);
    set_h_fcn(h_fcn);
    set_HH_fcn(HH_fcn);
    set_covariance( initial_covariance );

}

/*
Kalman_Filter::Kalman_Filter(const Kalman_Filter& kf):
    x_hat_k_k(kf.x_hat_k_k),
    P_k_k(kf.P_k_k),
    y_hat_k_k1(kf.y_hat_k_k1)
{
    f_fcn = kf.f_fcn;
    FF_fcn = kf.FF_fcn;
    h_fcn = kf.h_fcn;
    HH_fcn = kf.HH_fcn;
}*/
/*==============================================*/

/*===============DESTRUCTOR===================*/
//(default)
//Kalman_Filter::~Kalman_Filter(){ //(destructor)
//}
/*==============================================*/

/*=============GETTER===========================*/

    KF_FCN Kalman_Filter::get_f_fcn(){ return f_fcn_ptr; }

    KF_JAC_FCN Kalman_Filter::get_FF_fcn(){ return FF_fcn_ptr; }

    KF_FCN Kalman_Filter::get_h_fcn(){ return h_fcn_ptr; }

    KF_JAC_FCN Kalman_Filter::get_HH_fcn(){ return HH_fcn_ptr; }

    Vector<> Kalman_Filter::get_state(){ return x_hat_k_k; }

    Vector<> Kalman_Filter::get_output(){ return y_hat_k_k1; }

    Matrix <> Kalman_Filter::get_covariance(){ return P_k_k; }

/*==============================================*/

/*=============SETTER===========================*/

    void Kalman_Filter::set_f_fcn(KF_FCN fcn){
        f_fcn_ptr = fcn;
    }

    void Kalman_Filter::set_FF_fcn(KF_JAC_FCN fcn){
        FF_fcn_ptr = fcn;
    }

    void Kalman_Filter::set_h_fcn(KF_FCN fcn){
        h_fcn_ptr = fcn;
    }

    void Kalman_Filter::set_HH_fcn(KF_JAC_FCN fcn){
        HH_fcn_ptr = fcn;
    }

    void Kalman_Filter::set_state(Vector<> state){
        x_hat_k_k = state;
    }

    void Kalman_Filter::set_covariance( Matrix<> P ){
        P_k_k = P;
    }

/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/

    Vector<> Kalman_Filter::apply( Vector<> y_k, Vector<> u_k1, Matrix<> W_k1, Matrix<> V_k ){

        x_hat_k1_k1 = x_hat_k_k;
        P_k1_k1 = P_k_k;

        /*PREDICT*/
        //predict state estimate
        x_hat_k_k1 = f_fcn(x_hat_k1_k1, u_k1);

        //Predicted covariance estimate
        F_k1 = FF_fcn(x_hat_k1_k1, u_k1);
        P_k_k1 = F_k1 * P_k1_k1 * (F_k1.T()) + W_k1;

        /*UPDATE*/
        //Innovation or measurement residual
        y_hat_k_k1 = h_fcn(x_hat_k_k1, u_k1);
        y_tilde_k = y_k - y_hat_k_k1;

        //Innovation (or residual) covariance
        H_k = HH_fcn(x_hat_k_k1, u_k1);
        S_k = H_k * P_k_k1 * (H_k.T()) + V_k;

        //Near-optimal Kalman gain
        SVD<> S_k_SVD(S_k);
        K_k = P_k_k1 * (H_k.T()) * S_k_SVD.get_pinv();

        //Update state estimate
        x_hat_k_k = x_hat_k_k1 + K_k * y_tilde_k;

        //Update covariance estimate
        P_k_k = ( Identity_x - K_k*H_k )*P_k_k1;

        return y_hat_k_k1;


    }
/*==============================================*/

/*=============VARIE===========================*/

	void Kalman_Filter::display(){
        cout << endl << "===========================" << endl;
        cout << "Kalman Filter DISP:" << endl;
        cout << "state = " << endl;
        cout << x_hat_k_k << endl;
        cout << "output = " << endl;
        cout << y_hat_k_k1 << endl;
        cout << "covariance = " << endl;
        cout << P_k_k << endl;
        cout << "===========================" << endl;
    }

    void Kalman_Filter::reset(){

        x_hat_k_k = Zeros;
        P_k_k = Zeros;
        y_hat_k_k1 = Zeros;

    }

    Vector<> Kalman_Filter::f_fcn(const Vector<>& x_k_1, const Vector<>& u_k){
        return f_fcn_ptr( x_k_1, u_k );
    }
    
    Matrix<> Kalman_Filter::FF_fcn(const Vector<>& x_k_1, const Vector<>& u_k){
        return FF_fcn_ptr( x_k_1, u_k );
    }

    Vector<> Kalman_Filter::h_fcn(const Vector<>& x_k_1, const Vector<>& u_k){
        return h_fcn_ptr( x_k_1, u_k );
    }

    Matrix<> Kalman_Filter::HH_fcn(const Vector<>& x_k_1, const Vector<>& u_k){
        return HH_fcn_ptr( x_k_1, u_k );
    }

/*==============================================*/

/*=============STATIC FUNS===========================*/	
/*===================================================*/
