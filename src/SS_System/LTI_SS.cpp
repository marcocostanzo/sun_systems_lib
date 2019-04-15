
/*
    LTI_SS Class (Linear Filter)

    Copyright 2018-2019 Università della Campania Luigi Vanvitelli

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

#include "SS_System/LTI_SS.h"

using namespace TooN;
using namespace std;

/*===============CONSTRUCTORS===================*/
LTI_SS::LTI_SS( 
        const Matrix<>& A, 
        const Matrix<>& B, 
        const Matrix<>& C, 
        const Matrix<>& D,
        const Vector<>& initial_state,
        double Ts )
        :SS_Interface(initial_state, Ts),
        _A(A),
        _B(B),
        _C(C),
        _D(D)
        {

            if(!checkMatrixDims()){
                cout << WARNCOLOR "[LTI_SS]: ERROR in matrix dimentions" CRESET << endl;
                exit(-1);
            }

        }

LTI_SS::LTI_SS( 
        const Matrix<>& A, 
        const Matrix<>& B, 
        const Matrix<>& C, 
        const Matrix<>& D,
        const Vector<>& initial_state)
            :LTI_SS( 
                A, 
                B, 
                C, 
                D,
                initial_state,
                -1 )
            {}
    
LTI_SS::LTI_SS( 
        const Matrix<>& A, 
        const Matrix<>& B, 
        const Matrix<>& C,
        const Vector<>& initial_state)
            :LTI_SS( 
                A, 
                B, 
                C, 
                Zeros( C.num_rows(), B.num_cols() ),
                initial_state,
                -1 )
            {}

LTI_SS::LTI_SS( 
        const Matrix<>& A, 
        const Matrix<>& B, 
        const Matrix<>& C, 
        const Matrix<>& D,
        double Ts )
            :LTI_SS( 
                A, 
                B, 
                C, 
                D,
                Zeros( A.num_rows() ),
                Ts )
            {}

LTI_SS::LTI_SS( 
        const Matrix<>& A, 
        const Matrix<>& B, 
        const Matrix<>& C, 
        const Matrix<>& D )
            :LTI_SS( 
                A, 
                B, 
                C, 
                D,
                Zeros( A.num_rows() ),
                -1 )
            {}

LTI_SS::LTI_SS( 
        const Matrix<>& A, 
        const Matrix<>& B, 
        const Matrix<>& C )
            :LTI_SS( 
                A, 
                B, 
                C, 
                Zeros( C.num_rows(), B.num_cols() ),
                Zeros( A.num_rows() ),
                -1 )
            {}

/*
    Clone the object
*/
LTI_SS* LTI_SS::clone() const
{
    return new LTI_SS(*this);
}

/*==============================================*/

/*===============DESTRUCTOR===================*/	
/*==============================================*/

/*=============GETTER===========================*/

const Matrix<>& LTI_SS::getA() const
{
    return _A;
}

const Matrix<>& LTI_SS::getB() const
{
    return _B;
}

const Matrix<>& LTI_SS::getC() const
{
    return _C;
}

const Matrix<>& LTI_SS::getD() const
{
    return _D;
}

const Vector<>& LTI_SS::getState() const
{
    return _state;
}

const unsigned int LTI_SS::getDimInput() const 
{
    return _B.num_cols();
}

const unsigned int LTI_SS::getDimOutput() const
{
    return _C.num_rows();
}

/*==============================================*/

/*=============SETTER===========================*/

void LTI_SS::setA(const Matrix<>& A)
{
    //has to be square and same rows as state
    if( A.num_rows() != A.num_cols() || A.num_rows() != _state.size() ){
        cout << WARNCOLOR "[LTI_SS]: ERROR in setA() matrix dimentions" CRESET << endl;
        exit(-1);
    }
    _A = A;
}

void LTI_SS::setB(const Matrix<>& B)
{
    if( B.num_rows() != _state.size() || B.num_cols() != _D.num_cols() ){
        cout << WARNCOLOR "[LTI_SS]: ERROR in setB() matrix dimentions" CRESET << endl;
        exit(-1);
    }
    _B = B;
}

void LTI_SS::setC(const Matrix<>& C)
{
    if( C.num_rows() != _D.num_rows() || C.num_cols() != _state.size() ){
        cout << WARNCOLOR "[LTI_SS]: ERROR in setC() matrix dimentions" CRESET << endl;
        exit(-1);
    }
    _C = C;
}

void LTI_SS::setD(const Matrix<>& D)
{
    if( D.num_rows() != _C.num_rows() || D.num_cols() != _B.num_cols() ){
        cout << WARNCOLOR "[LTI_SS]: ERROR in setD() matrix dimentions" CRESET << endl;
        exit(-1);
    }
    _D = D;
}

void LTI_SS::setState(const Vector<>& state)
{
    if( state.size() != _A.num_rows() ){
        cout << WARNCOLOR "[LTI_SS]: ERROR in setState() vector dimention" CRESET << endl;
        exit(-1);
    }
    _state = state;
}

/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/

Vector<> LTI_SS::state_f( const Vector<>& state, const Vector<>& input ) const
{
    return _A * state + _B * input;
}

Vector<> LTI_SS::output_f( const Vector<>& state, const Vector<>& input ) const
{
    return _C * state + _D * input;
}

/*==============================================*/

/*=============VARIE===========================*/

void LTI_SS::display() 
{
        cout << endl;
        cout << "==========================" << endl;
        cout << "[LTI_SS]:" << endl;
        cout << "A = " << endl << _A << endl;
        cout << "B = " << endl << _B << endl;
        cout << "C = " << endl << _C << endl;
        cout << "D = " << endl << _D << endl;
        cout << "State = " << _state << endl;
        cout << "==========================" << endl;
}

bool LTI_SS::checkMatrixDims() const
{

    //A has to be square
    if( _A.num_rows() != _A.num_cols() ){
        return false;
    }

    //state has to have the same size as _A
    if( _state.size() != _A.num_rows() ){
        return false;
    }

    //B has to have same rows as _A
    if( _B.num_rows() != _A.num_rows() ){
        return false;
    }

    //C has to have same cols as _A
    if( _C.num_cols() != _A.num_cols() ){
        return false;
    }

    //D has to have same cols as B and same rows as C
    if( _D.num_cols() != _B.num_cols() || _D.num_rows() != _C.num_rows() ){
        return false;
    }

    return true;

}
/*==============================================*/



/*=============STATIC FUNS===========================*/

/*==============================================*/
