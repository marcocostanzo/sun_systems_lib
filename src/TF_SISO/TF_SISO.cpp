
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

#include "TF_SISO/TF_SISO.h"

using namespace TooN;
using namespace std;



/*===============CONSTRUCTORS===================*/
//COMPLETE CONSTRUCTOR
TF_SISO::TF_SISO(TooN::Vector<> b_vect, TooN::Vector<> a_vect, double Ts):
    Linear_System(Ts),
    _b_vect(b_vect),
    _a_vect(a_vect),
    _n(b_vect.size()-1),
    _m(a_vect.size()),
    _u_vect(TooN::Zeros(_n+1)),
    _y_vect(TooN::Zeros(_m)),
    _y_k(0.0)
{
    reset();
}

TF_SISO::TF_SISO(TooN::Vector<> b_vect, TooN::Vector<> a_vect): TF_SISO(b_vect, a_vect, -1.0){}

TF_SISO::TF_SISO(double Ts):
    TF_SISO( makeVector(0.0), makeVector(0.0), Ts){};

TF_SISO::TF_SISO():
    TF_SISO(-1.0){};


//TF_SISO(const TF_SISO& tf);


/*==============================================*/

/*===============DESTRUCTOR===================*/
//(default)
//TF_SISO::~TF_SISO(){ //(destructor)
//}
/*==============================================*/

/*=============GETTER===========================*/
    const unsigned int TF_SISO::getN(){return _n;}
    const unsigned int TF_SISO::getM(){return _m;}
    TooN::Vector<> TF_SISO::getB_vect(){return _b_vect;}
    TooN::Vector<> TF_SISO::getA_vect(){return _a_vect;}
    TooN::Vector<> TF_SISO::getU_vect(){return _u_vect;}
    TooN::Vector<> TF_SISO::getY_vect(){return _y_vect;}
    double TF_SISO::getYk(){return _y_k;}
/*==============================================*/

/*=============SETTER===========================*/

    void TF_SISO::setU_vect(TooN::Vector<> u_vect){
        if(u_vect.size()!=_n+1){
            std::cout << "[TF_SISO]: setU_vect(), size error" << std::endl;
            exit(-1);
        }
        _u_vect = u_vect;
    }

    void TF_SISO::setY_vect(TooN::Vector<> y_vect){
        if(y_vect.size()!=_m){
            std::cout << "[TF_SISO]: setY_vect(), size error" << std::endl;
            exit(-1);
        }
        _y_vect = y_vect;
    }

    void TF_SISO::setYk( double y_k ){
        _y_k = y_k;
    }

/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    double TF_SISO::apply( double u_k){

        _u_vect.slice(1,_n) = _u_vect.slice(0,_n);
        _u_vect[0] = u_k;

        _y_vect.slice(1,_m-1) = _y_vect.slice(0,_m-1);
        _y_vect[0] = _y_k;

        _y_k = (_b_vect * _u_vect) + (_a_vect * _y_vect);

        return _y_k;
    
    }

    Vector<> TF_SISO::apply( Vector<> input ){
        return makeVector( apply( input[0] ) );
    }
/*==============================================*/

/*=============VARIE===========================*/

	void TF_SISO::display(){
	std::cout << std::endl << "===========================" << std::endl;
	std::cout << "TF_SISO DISP:" << std::endl;
	std::cout << "b_vect = " << std::endl;
	std::cout << _b_vect << std::endl;
    std::cout << "a_vect = " << std::endl;
	std::cout << _a_vect << std::endl;
    std::cout << "u_vect = " << std::endl;
	std::cout << _u_vect << std::endl;
    std::cout << "y_vect = " << std::endl;
	std::cout << _y_vect << std::endl;
	std::cout << "y_k = " << std::endl;
	std::cout << _y_k << std::endl;
	std::cout << "===========================" << std::endl;
}

    void TF_SISO::reset(){

        _u_vect = TooN::Zeros;
        _y_vect = TooN::Zeros;
        _y_k = 0.0;

    }
/*==============================================*/

/*=============STATIC FUNS===========================*/	

/*===================================================*/
