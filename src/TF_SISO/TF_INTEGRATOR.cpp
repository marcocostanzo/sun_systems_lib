
/*
    TF_INTEGTATOR Class

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

#include "TF_INTEGTATOR.h"

/*===============CONSTRUCTORS===================*/
    TF_INTEGRATOR::TF_INTEGRATOR(double Ts) : 
        TF_SISO( 
                (Ts/2.0) * TooN::makeVector( 1.0 , 1.0 ),
                TooN::makeVector( 1.0 ),
                Ts) {};
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~TF_INTEGRATOR(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
    void TF_INTEGRATOR::setState(double output){
        setU_vect( TooN::makeVector( 0.0, 0.0 ) );
	    setY_vect( TooN::makeVector( output ) );
	    setYk(output);
    }
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
/*==============================================*/

/*=============VARIE===========================*/
/*==============================================*/


/*=============STATIC FUNS===========================*/

/*==============================================*/
