
/*
    TF_SISO Class Transfer Function SISO (Linear Filter)

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

#ifndef TF_SISO_LIB
#define TF_SISO_LIB

#include "Linear_System.h"

class TF_SISO : public Linear_System {
private:

protected:

    unsigned int _n;
    unsigned int _m;

    TooN::Vector<> _b_vect; //n+1
    TooN::Vector<> _a_vect; //m

    TooN::Vector<> _u_vect; //n+1
    TooN::Vector<> _y_vect; //m
    double _y_k;

public:
/*===============CONSTRUCTORS===================*/
    TF_SISO(TooN::Vector<> b_vect, TooN::Vector<> a_vect, double Ts);
	TF_SISO(TooN::Vector<> b_vect, TooN::Vector<> a_vect);

    //TF_SISO_ZERO Constructors
    TF_SISO(double Ts);
    TF_SISO();

	//TF_SISO(const TF_SISO& tf);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~TF_SISO(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
	const unsigned int getN();
    const unsigned int getM();
    TooN::Vector<> getB_vect();
    TooN::Vector<> getA_vect();
    TooN::Vector<> getU_vect();
    TooN::Vector<> getY_vect();
    double getYk();
/*==============================================*/

/*=============SETTER===========================*/
    void setU_vect(TooN::Vector<> u_vect);
    void setY_vect(TooN::Vector<> y_vect);
    void setYk( double yk );
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
	virtual double apply( double uk);
    TooN::Vector<> apply( TooN::Vector<> input );
/*==============================================*/

/*=============VARIE===========================*/
    virtual void reset();
    virtual void display();
/*==============================================*/

};


/*=============STATIC FUNS===========================*/

/*==============================================*/

#endif
