
/*
    TF_FIRST_ORDER_FILTER Class

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

#include "TF_SISO/TF_FIRST_ORDER_FILTER.h"

using namespace TooN;

/*===============CONSTRUCTORS===================*/

    TF_FIRST_ORDER_FILTER::TF_FIRST_ORDER_FILTER(double cut_freq, double Ts, double gain):
        TF_SISO(    tf_first_order_get_b_vect(cut_freq, Ts), 
                    tf_first_order_get_a_vect(cut_freq, Ts) ),
        _gain(gain){}

    TF_FIRST_ORDER_FILTER::TF_FIRST_ORDER_FILTER(double cut_freq, double Ts) : 
        TF_FIRST_ORDER_FILTER(cut_freq, Ts, 1.0){};

	//TF_FIRST_ORDER_FILTER(const TF_FIRST_ORDER_FILTER& tf);

    /*
    Clone the object
    */
    TF_FIRST_ORDER_FILTER* TF_FIRST_ORDER_FILTER::clone() const{
        return new TF_FIRST_ORDER_FILTER(*this);
    }
/*==============================================*/
/*===============STATIC FUNCTIONS FOR SIMPLE CONSTRUCTOR WRITING=======*/
    Vector<2> TF_FIRST_ORDER_FILTER::tf_first_order_get_b_vect(double cut_freq, double Ts){
        double tau = 1.0/(2.0*M_PI*cut_freq);
        double _alpha = 2.0*tau/Ts;
        return (1.0/(1.0+_alpha)) * TooN::makeVector( 1.0 , 1.0 );
    }
    Vector<1> TF_FIRST_ORDER_FILTER::tf_first_order_get_a_vect(double cut_freq, double Ts){
        double tau = 1.0/(2.0*M_PI*cut_freq);
        double _alpha = 2.0*tau/Ts;
        return TooN::makeVector( -( (1.0-_alpha)/(1.0+_alpha) ) );
    }
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~TF_FIRST_ORDER_FILTER(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
    void TF_FIRST_ORDER_FILTER::setState(double output){
        output = output/_gain;
        setU_vect( TooN::makeVector( output, output ) );
	    setY_vect( TooN::makeVector( output ) );
	    setYk(output);
    }
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    double TF_FIRST_ORDER_FILTER::apply( double uk){
        return _gain*TF_SISO::apply( uk );
    }
/*==============================================*/

/*=============VARIE===========================*/
/*==============================================*/


/*=============STATIC FUNS===========================*/

/*==============================================*/
