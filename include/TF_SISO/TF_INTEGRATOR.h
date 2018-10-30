
/*
    TF_INTEGTATOR Class

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

#ifndef TF_INTEGTATOR_LIB
#define TF_INTEGTATOR_LIB

#include "TF_SISO/TF_SISO.h"

class TF_INTEGTATOR : public TF_SISO{
private:

	TF_INTEGTATOR(); //NO DEFAULT CONSTRUCTOR

protected:

    double _gain;

public:
/*===============CONSTRUCTORS===================*/
    TF_INTEGRATOR(double Ts, double gain);
    TF_INTEGRATOR(double Ts);
    
    /*
            Clone the object
    */
    virtual TF_INTEGRATOR* clone() const;

    TF_INTEGRATOR( const TF_INTEGRATOR& tf ) = default;
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~TF_INTEGRATOR(); //(destructor)
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

using TF_INTEGTATOR_Ptr = std::unique_ptr<TF_INTEGTATOR>;

#endif
