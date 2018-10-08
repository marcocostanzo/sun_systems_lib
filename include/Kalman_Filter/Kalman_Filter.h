
/*
    Kalman_Filter Class

    Copyright 2018 Università della Campania Luigi Vanvitelli

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

#ifndef KALMAN_FILTER_LIB
#define KALMAN_FILTER_LIB


#include <TooN/TooN.h>
#include "TooN/SVD.h"
#include <Helper.h>

typedef TooN::Vector<> (*KF_FCN)(const TooN::Vector<>&, const TooN::Vector<>&); //fcns handles type
typedef TooN::Matrix<> (*KF_JAC_FCN)(const TooN::Vector<>&, const TooN::Vector<>&); //fcns handles type

class Kalman_Filter{
private:

	Kalman_Filter(); //NO DEFAULT CONSTRUCTOR

    KF_FCN f_fcn;
    KF_JAC_FCN FF_fcn;
    KF_FCN h_fcn;
    KF_JAC_FCN HH_fcn;

    TooN::Vector<> x_hat_k_k;
    TooN::Matrix<> P_k_k;
    TooN::Vector<> y_hat_k_k1;

    //Internal Vars
    TooN::Vector<> x_hat_k1_k1;
    TooN::Matrix<> P_k1_k1;
    TooN::Vector<> x_hat_k_k1;
    TooN::Matrix<> F_k1;
    TooN::Matrix<> Identity_x;
    TooN::Matrix<> P_k_k1;
    TooN::Vector<> y_tilde_k;
    TooN::Matrix<> H_k;
    TooN::Matrix<> S_k;
    TooN::Matrix<> K_k;

public:
/*===============CONSTRUCTORS===================*/
	Kalman_Filter(TooN::Vector<> initial_state, TooN::Matrix<> initial_covariance, const int outDim, KF_FCN f_fcn, KF_JAC_FCN FF_fcn, KF_FCN h_fcn, KF_JAC_FCN HH_fcn); //COMPLETE CONSTRUCTOR

	//Kalman_Filter(const Kalman_Filter& kf);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~Kalman_Filter(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
	KF_FCN get_f_fcn();
    KF_JAC_FCN get_FF_fcn();
    KF_FCN get_h_fcn();
    KF_JAC_FCN get_HH_fcn();
    TooN::Vector<> get_state();
    TooN::Vector<> get_output();
    TooN::Matrix<> get_covariance();
/*==============================================*/

/*=============SETTER===========================*/
    void set_f_fcn(KF_FCN fcn);
    void set_FF_fcn(KF_JAC_FCN fcn);
    void set_h_fcn(KF_FCN fcn);
    void set_HH_fcn(KF_JAC_FCN fcn);
    void set_state(TooN::Vector<> state);
    void set_covariance( TooN::Matrix<> P );
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
	TooN::Vector<> apply( TooN::Vector<> y_measure, TooN::Vector<> u_measure, TooN::Matrix<> W_k1, TooN::Matrix<> V_k1 );
/*==============================================*/

/*=============VARIE===========================*/
	void display();
    void reset();
/*==============================================*/

};


/*=============STATIC FUNS===========================*/
/*==============================================*/

#endif
