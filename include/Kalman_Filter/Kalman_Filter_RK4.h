
/*
    Kalman_Filter_RK4 is a Kalman_Filter that use RK4 discretization Class

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

#ifndef KALMAN_FILTER_RK4_LIB
#define KALMAN_FILTER_RK4_LIB


#include <Kalman_Filter/Kalman_Filter.h>
#include <Discretization/RK4.h>

class Kalman_Filter_RK4 : public Kalman_Filter{

private:

	Kalman_Filter_RK4(); //NO DEFAULT CONSTRUCTOR

protected:

    RK4 rk4;

public:
/*===============CONSTRUCTORS===================*/
	Kalman_Filter_RK4( TooN::Vector<> initial_state, TooN::Matrix<> initial_covariance, const int inDim, const int outDim, RK4_FCN f_cnt_fcn, RK4_JAC_FCN FF_cnt_fcn, KF_FCN h_fcn, KF_JAC_FCN HH_fcn, double Ts); //COMPLETE CONSTRUCTOR

	//Kalman_Filter_RK4(const Kalman_Filter_RK4& kf);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~Kalman_Filter_RK4(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
    virtual void setPrecInput( TooN::Vector<> prec_input);
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
	virtual TooN::Vector<> apply( TooN::Vector<> y_measure, TooN::Vector<> u_measure, TooN::Matrix<> W_k1, TooN::Matrix<> V_k1 );
/*==============================================*/

/*=============VARIE===========================*/
	virtual void display();
    virtual void reset();
    virtual TooN::Vector<> f_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k);
    virtual TooN::Matrix<> FF_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k);
/*==============================================*/

};


/*=============STATIC FUNS===========================*/
/*==============================================*/

#endif
