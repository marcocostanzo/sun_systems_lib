
/*
    SS_System Class (Linear Filter)

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

#ifndef SS_SYSTEM_LIB
#define SS_SYSTEM_LIB

#include "Linear_System.h"

class SS_System : public Linear_System {
private:

protected:

    //State
    TooN::Vector<> _x_k_1;

    //Last output
    TooN::Vector<> _y_k;

    //Matrix of the ss
    TooN::Matrix<> _A, _B, _C, _D;

public:
/*===============CONSTRUCTORS===================*/
    SS_System( TooN::Matrix<> A, TooN::Matrix<> B, TooN::Matrix<> C, TooN::Matrix<> D, TooN::Vector<> x0, double Ts );
    SS_System( TooN::Matrix<> A, TooN::Matrix<> B, TooN::Matrix<> C, TooN::Matrix<> D, TooN::Vector<> x0 );
    SS_System( TooN::Matrix<> A, TooN::Matrix<> B, TooN::Matrix<> C, TooN::Matrix<> D, double Ts );
    SS_System( TooN::Matrix<> A, TooN::Matrix<> B, TooN::Matrix<> C, TooN::Matrix<> D);

    //SS_System Constructors
    SS_System(double Ts);
    SS_System();

	//SS_System(const TF_SISO& tf);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~SS_System(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
	const unsigned int getOrder();
    TooN::Matrix<> getA();
    TooN::Matrix<> getB();
    TooN::Matrix<> getC();
    TooN::Matrix<> getD();
    TooN::Vector<> getX();
    TooN::Vector<> getY();
/*==============================================*/

/*=============SETTER===========================*/
    void setA(TooN::Matrix<> A);
    void setB(TooN::Matrix<> B);
    void setC(TooN::Matrix<> C);
    void setD(TooN::Matrix<> D);
    void setX(TooN::Vector<> x);
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    virtual TooN::Vector<> apply( TooN::Vector<> input );
/*==============================================*/

/*=============VARIE===========================*/
    virtual void reset();
    virtual void display();
    virtual bool checkMatrixDims();
/*==============================================*/

};


/*=============STATIC FUNS===========================*/

/*==============================================*/

#endif
