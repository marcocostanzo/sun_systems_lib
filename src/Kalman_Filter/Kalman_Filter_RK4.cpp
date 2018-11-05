
/*
    Kalman_Filter_RK4 is a Kalman_Filter that use RK4 discretization Class

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



#include <Kalman_Filter/Kalman_Filter_RK4.h>

using namespace TooN;
using namespace std;



/*===============CONSTRUCTORS===================*/
	Kalman_Filter_RK4::Kalman_Filter_RK4(const Vector<>& initial_state, const Matrix<>& initial_covariance, const int inDim, const int outDim, const RK4_FCN& f_cnt_fcn, const RK4_JAC_FCN& FF_cnt_fcn, const KF_FCN& h_fcn, const KF_JAC_FCN& HH_fcn, double Ts): //COMPLETE CONSTRUCTOR
        Kalman_Filter(initial_state, initial_covariance, outDim, NULL, NULL, h_fcn, HH_fcn),
        rk4( initial_state.size(), inDim, f_cnt_fcn, FF_cnt_fcn, Ts )
        {
        }



	//Kalman_Filter_RK4::(const Kalman_Filter& kf);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~Kalman_Filter_RK4(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
    void Kalman_Filter_RK4::setPrecInput( const Vector<>& prec_input){
        rk4.setPrecInput(prec_input);
    }
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
	Vector<> Kalman_Filter_RK4::apply( const Vector<>& y_measure, const Vector<>& u_measure, const Matrix<>& W_k1, const Matrix<>& V_k1 ){

        rk4.estimateFutureInputs( u_measure );
        return Kalman_Filter::apply( y_measure, u_measure,  W_k1, V_k1 );
        
    }
/*==============================================*/

/*=============VARIE===========================*/

	void Kalman_Filter_RK4::display(){
        cout << endl << "===========================" << endl;
        cout << "Kalman Filter RK4 DISP:" << endl;
        cout << "state = " << endl;
        cout << x_hat_k_k << endl;
        cout << "output = " << endl;
        cout << y_hat_k_k1 << endl;
        cout << "covariance = " << endl;
        cout << P_k_k << endl;
        cout << "===========================" << endl;
    }

    void Kalman_Filter_RK4::reset(){
        Kalman_Filter::reset();
        rk4.reset();
    }

    Vector<> Kalman_Filter_RK4::f_fcn(const Vector<>& x_k_1, const Vector<>& u_k){
        return rk4.apply_rk4( x_k_1, u_k );
    }

    Matrix<> Kalman_Filter_RK4::FF_fcn(const Vector<>& x_k_1, const Vector<>& u_k){
        return rk4.apply_rk4_jac( x_k_1, u_k );
    }

/*==============================================*/


/*=============STATIC FUNS===========================*/
/*==============================================*/

