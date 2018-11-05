
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

#include "Discretization/RK4.h"

using namespace TooN;
using namespace std;


/*===============CONSTRUCTORS===================*/

    RK4::RK4( const int dim_state, const Vector<>& initial_prec_input, const RK4_FCN& ff, const RK4_JAC_FCN& FF, double Ts ):
        _Ts(Ts),
        _size_state(dim_state),
        _size_input(initial_prec_input.size()),
        _ff(ff),
        _FF(FF),
        u_n_1(initial_prec_input),
        
        /*Preallocated Vars*/
        Ts_2(Ts/2.0),
        Ts_6(Ts/6.0),
        I( Identity( _size_state ) ),
        k1(Zeros(_size_state)),
        k2(Zeros(_size_state)),
        k3(Zeros(_size_state)),
        k4(Zeros(_size_state)),
        x_n(Zeros(_size_state)),
        jac_k1(Zeros(_size_state,_size_state)),
        jac_k2(Zeros(_size_state,_size_state)),
        jac_k3(Zeros(_size_state,_size_state)),
        jac_k4(Zeros(_size_state,_size_state)),
        jac_n(Zeros(_size_state,_size_state)),
        internal_u_n(Zeros(_size_input)),
        M(Zeros(_size_input)),
        u_n12(Zeros(_size_input)),
        u_n1(Zeros(_size_input))
        {

            cout << WARNCOLOR "[RK4] in this version: Using ZOH in estimateFutureInputs()!" CRESET << endl;

        }

    RK4::RK4( const int dim_state, const int dim_input , const RK4_FCN& ff, const RK4_JAC_FCN& FF, double Ts ):
        RK4( dim_state, Zeros(dim_input), ff, FF, Ts )
        {

        }


	//RK4(const RK4& rk4);
/*==============================================*/

/*===============Private methods===================*/
    
/*==============================================*/

/*=============GETTER===========================*/
    Vector<> RK4::getPrecInput(){
        return u_n_1;
    }
/*==============================================*/

/*=============SETTER===========================*/
    void RK4::setPrecInput( const Vector<>& prec_input){
        u_n_1 = prec_input;
    }
/*==============================================*/


/*=============RUNNER===========================*/

    Vector<> RK4::apply_rk4( const Vector<>& x_n_1, const Vector<>& u_n ){

        k1 = _ff( x_n_1 , u_n );
        k2 = _ff( x_n_1 + Ts_2*k1 , u_n12 );
        k3 = _ff( x_n_1 + Ts_2*k2 , u_n12 );
        k4 = _ff( x_n_1 + _Ts*k3 , u_n1 );

        x_n = x_n_1 + Ts_6 * ( k1 + 2.0*k2 + 2.0*k3 + k4 );

        return x_n;

    }

    Matrix<> RK4::apply_rk4_jac( const Vector<>& x_n_1, const Vector<>& u_n ){

        k1 = _ff( x_n_1 , u_n );
        k2 = _ff( x_n_1 + Ts_2*k1 , u_n12 );
        k3 = _ff( x_n_1 + Ts_2*k2 , u_n12 );

        jac_k1 = _FF( x_n_1 , u_n );
        jac_k2 = _FF( x_n_1 + Ts_2*k1 , u_n12 ) * ( I + Ts_2 * jac_k1 );
        jac_k3 = _FF( x_n_1 + Ts_2*k2 , u_n12 ) * ( I + Ts_2 * jac_k2 );
        jac_k4 = _FF( x_n_1 + _Ts*k3 , u_n1 ) * ( I + _Ts * jac_k3 );

        jac_n = I + Ts_6 * ( jac_k1 + 2.0*jac_k2 + 2.0*jac_k3 + jac_k4 );

        return jac_n;

    }

    void RK4::estimateFutureInputs(const Vector<>& u_n){

        //WARN! this method is a ZOH, you must comment some lines to implement 1^ order interpolation

        if(_b_use_zoh){
            u_n_1 = u_n;
            internal_u_n = u_n;
        }

        u_n_1 = internal_u_n;
        internal_u_n = u_n;

        M = (internal_u_n - u_n_1)/_Ts;
        u_n12 = M*Ts_2 + internal_u_n;
        u_n1 = M*_Ts + internal_u_n;

    }

/*==============================================*/

/*=============VARIE===========================*/
	void RK4::display(){
        printf("RK4: Display()... TODO.... \n");
    }

    void RK4::reset(){
        u_n_1 = Zeros;
    }
/*==============================================*/