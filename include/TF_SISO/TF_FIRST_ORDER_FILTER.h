
/*
    TF_FIRST_ORDER_FILTER Class

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

#ifndef TF_FIRST_ORDER_FILTER_LIB
#define TF_FIRST_ORDER_FILTER_LIB

#include "TF_SISO/TF_SISO.h"

class TF_FIRST_ORDER_FILTER : public TF_SISO {
private:

	TF_FIRST_ORDER_FILTER(); //NO DEFAULT CONSTRUCTOR

protected:

    double _gain;

public:
/*===============CONSTRUCTORS===================*/
    TF_FIRST_ORDER_FILTER(double cut_freq, double Ts);
    TF_FIRST_ORDER_FILTER(double cut_freq, double Ts, double gain);

	//TF_FIRST_ORDER_FILTER(const TF_FIRST_ORDER_FILTER& tf);
/*==============================================*/
/*===============STATIC FUNCTIONS FOR SIMPLE CONSTRUCTOR WRITING=======*/
    static TooN::Vector<2> tf_first_order_get_b_vect(double cut_freq, double Ts);
    static TooN::Vector<1> tf_first_order_get_a_vect(double cut_freq, double Ts);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~TF_FIRST_ORDER_FILTER(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
    void setState(double output);
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    virtual double apply( double uk);
/*==============================================*/

/*=============VARIE===========================*/
/*==============================================*/

};


/*=============STATIC FUNS===========================*/

/*==============================================*/

#endif
