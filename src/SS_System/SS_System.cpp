
/*
    SS_System Class (Linear Filter)

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

#include "SS_System/SS_System.h"

using namespace TooN;
using namespace std;

/*===============CONSTRUCTORS===================*/
    SS_System::SS_System( Matrix<> A, Matrix<> B, Matrix<> C, Matrix<> D, Vector<> x0, double Ts ):
        Linear_System(Ts),
        _A(A),
        _B(B),
        _C(C),
        _D(D),
        _x_k_1(x0),
        _y_k(Zeros(C.num_rows()))
        {

            if(!checkMatrixDims()){
                cout << BOLDRED "[SS_System]: ERROR in matrix dimentions" << endl;
                exit(-1);
            }

        }
    
    SS_System::SS_System( Matrix<> A, Matrix<> B, Matrix<> C, Matrix<> D, Vector<> x0 ):
        SS_System( A, B, C, D, x0, -1 )
        {}


    SS_System::SS_System( Matrix<> A, Matrix<> B, Matrix<> C, Matrix<> D, double Ts ):
        SS_System( A, B, C, D, Zeros( A.num_rows() ), Ts )
        {}

    SS_System::SS_System( Matrix<> A, Matrix<> B, Matrix<> C, Matrix<> D):
        SS_System( A, B, C, D, Zeros( A.num_rows() ), -1 )
        {}

    //SS_System Constructors
    SS_System::SS_System(double Ts):
        SS_System( Zeros(0.0), Zeros(0.0), Zeros(0.0), Zeros(0.0), Zeros(0.0), Ts )
        {}

    SS_System::SS_System():
        SS_System(-1.0)
        {}

	//SS_System(const TF_SISO& tf);
/*==============================================*/

/*===============DESTRUCTOR===================*/	
//	~SS_System(); //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
	const unsigned int SS_System::getOrder(){
        return _x_k_1.size();
    }
    Matrix<> SS_System::getA(){
        return _A;
    }
    Matrix<> SS_System::getB(){
        return _B;
    }
    Matrix<> SS_System::getC(){
        return _C;
    }
    Matrix<> SS_System::getD(){
        return _D;
    }
    Vector<> SS_System::getX(){
        return _x_k_1;
    }
    Vector<> SS_System::getY(){
        return _y_k;
    }
/*==============================================*/

/*=============SETTER===========================*/
    void SS_System::setA(Matrix<> A){
        _A = A;
        if(!checkMatrixDims()){
            cout << BOLDRED "[SS_System]: ERROR in setA() matrix dimentions" << endl;
            exit(-1);
        }
    }

    void SS_System::setB(Matrix<> B){
        _B = B;
        if(!checkMatrixDims()){
            cout << BOLDRED "[SS_System]: ERROR in setB() matrix dimentions" << endl;
            exit(-1);
        }
    }

    void SS_System::setC(Matrix<> C){
        _C = C;
        if(!checkMatrixDims()){
            cout << BOLDRED "[SS_System]: ERROR in setC() matrix dimentions" << endl;
            exit(-1);
        }
    }

    void SS_System::setD(Matrix<> D){
        _D = D;
        if(!checkMatrixDims()){
            cout << BOLDRED "[SS_System]: ERROR in setD() matrix dimentions" << endl;
            exit(-1);
        }
    }

    void SS_System::setX(Vector<> x){
        _x_k_1 = x;
        if(!checkMatrixDims()){
            cout << BOLDRED "[SS_System]: ERROR in setX() matrix dimentions" << endl;
            exit(-1);
        }
    }
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
    Vector<> SS_System::apply( Vector<> input ){

        _x_k_1 = _A * _x_k_1 + _B * input;
        _y_k = _C * _x_k_1 + _D * input;

        return _y_k;

    }
/*==============================================*/

/*=============VARIE===========================*/
    void SS_System::reset(){
        _x_k_1 = Zeros;
        _y_k = Zeros;
    }

    void SS_System::display(){

        cout << endl;
        cout << "==========================" << endl;
        cout << "SS_System:" << endl;
        cout << "A = " << endl << _A << endl;
        cout << "B = " << endl << _B << endl;
        cout << "C = " << endl << _C << endl;
        cout << "D = " << endl << _D << endl;
        cout << "State = " << _x_k_1 << endl;
        cout << "Last Output = " << _y_k << endl;
        cout << "==========================" << endl;

    }

    bool SS_System::checkMatrixDims(){
        
        //check State eq
        if( _x_k_1.size() != _A.num_rows() || _x_k_1.size() != _A.num_cols() || _x_k_1.size() != _B.num_rows() ){
            return false;
        }

        //check Output eq
        if( _y_k.size() != _C.num_rows() || _x_k_1.size() != _C.num_cols() || _y_k.size() != _D.num_rows() ){
            return false;
        }

        //check input dims
        if( _B.num_cols() != _D.num_cols() ){
            return false;
        }

        return true;

    }
/*==============================================*/



/*=============STATIC FUNS===========================*/

/*==============================================*/
