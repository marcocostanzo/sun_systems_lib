
/*
    RK4 Class Runge Kutta 4

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

#ifndef RK4_LIB
#define RK4_LIB

#include <TooN/TooN.h>
#include <Helper.h>

typedef TooN::Vector<> (*RK4_FCN)(const TooN::Vector<>&, const TooN::Vector<>&); //fcns handles type
typedef TooN::Matrix<> (*RK4_JAC_FCN)(const TooN::Vector<>&, const TooN::Vector<>&); //fcns handles type

class RK4{
private:

	RK4(); //NO DEFAULT CONSTRUCTOR

    const double _Ts;
    const int _size_state;
    const int _size_input;

    RK4_FCN _ff;
    RK4_JAC_FCN _FF;

    bool _b_use_zoh = true;

    /*Internal Vars (state)*/
    TooN::Vector<> u_n_1; //input at step n-1

    /*Output Vars*/


    /*Vars (preallocated)*/
    double Ts_2;
    double Ts_6;
    TooN::Matrix<> I;
    TooN::Vector<> k1, k2, k3, k4, x_n;
    TooN::Matrix<> jac_k1, jac_k2, jac_k3, jac_k4, jac_n;
    TooN::Vector<> internal_u_n, M, u_n12, u_n1;


    /*Private fun*/

public:
/*===============CONSTRUCTORS===================*/

    RK4( const int dim_state, TooN::Vector<> initial_prec_input, RK4_FCN ff, RK4_JAC_FCN FF, double Ts );

    RK4( const int dim_state, const int dim_input , RK4_FCN ff, RK4_JAC_FCN FF, double Ts );

	//RK4(const RK4& rk4);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~RK4(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
    TooN::Vector<> getPrecInput();
/*==============================================*/

/*=============SETTER===========================*/
    void setPrecInput( TooN::Vector<> prec_input);
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    TooN::Vector<> apply_rk4( TooN::Vector<> x_n_1, TooN::Vector<> u_n );
    TooN::Matrix<> apply_rk4_jac( TooN::Vector<> x_n_1, TooN::Vector<> __un );
    void estimateFutureInputs(TooN::Vector<> u_n);
/*==============================================*/

/*=============VARIE===========================*/
	void display();
    void reset();
/*==============================================*/

};


/*=============STATIC FUNS===========================*/

/*==============================================*/

#endif
