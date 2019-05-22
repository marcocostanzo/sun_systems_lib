
/*
    RK4 Class Runge Kutta 4

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

#ifndef RK4_LIB
#define RK4_LIB

#include <TooN/TooN.h>
#include "boost/function.hpp"

#define ERRORCOLOR      "\033[1m\033[31m"      /* Bold Red */
#define SUCCESSCOLOR    "\033[1m\033[32m"      /* Bold Green */
#define WARNCOLOR       "\033[1m\033[33m"      /* Bold Yellow */
#define CRESET          "\033[0m"

//typedef TooN::Vector<> (*RK4_FCN)(const TooN::Vector<>&, const TooN::Vector<>&); //fcns handles type
//typedef TooN::Matrix<> (*RK4_JAC_FCN)(const TooN::Vector<>&, const TooN::Vector<>&); //fcns handles type
typedef boost::function<TooN::Vector<>(const TooN::Vector<>&, const TooN::Vector<>&)> RK4_FCN;
typedef boost::function<TooN::Matrix<>(const TooN::Vector<>&, const TooN::Vector<>&)> RK4_JAC_FCN;

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
    TooN::Vector<> u_n_12;


    /*Private fun*/

public:
/*===============CONSTRUCTORS===================*/

    RK4( const int dim_state, const TooN::Vector<>& initial_prec_input, const RK4_FCN& ff, const RK4_JAC_FCN& FF, double Ts );

    RK4( const int dim_state, const int dim_input , const RK4_FCN& ff, const RK4_JAC_FCN& FF, double Ts );

	//RK4(const RK4& rk4);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
    virtual ~RK4() = default;
/*==============================================*/

/*=============GETTER===========================*/
    TooN::Vector<> getPrecInput();
/*==============================================*/

/*=============SETTER===========================*/
    void setPrecInput( const TooN::Vector<>& prec_input);
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    TooN::Vector<> apply_rk4( const TooN::Vector<>& x_n_1, const TooN::Vector<>& u_n );
    TooN::Matrix<> apply_rk4_jac( const TooN::Vector<>& x_n_1, const TooN::Vector<>& __un );
    void estimateMeanInputs(const TooN::Vector<>& u_n);
/*==============================================*/

/*=============VARIE===========================*/
	void display();
    void reset();
/*==============================================*/

};


/*=============STATIC FUNS===========================*/

/*==============================================*/

#endif
