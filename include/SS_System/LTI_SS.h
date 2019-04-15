
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

#ifndef LTI_SS_H
#define LTI_SS_H

#include "SS_System/SS_Interface.h"
#include "Linear_System.h"

class LTI_SS : public SS_Interface, public Linear_System {
private:

LTI_SS( );

protected:

//Matrix of the ss
TooN::Matrix<> _A, _B, _C, _D;

public:
/*===============CONSTRUCTORS===================*/
LTI_SS( const TooN::Matrix<>& A, 
        const TooN::Matrix<>& B, 
        const TooN::Matrix<>& C, 
        const TooN::Matrix<>& D,
        const TooN::Vector<>& initial_state,
        double Ts );

LTI_SS( const TooN::Matrix<>& A, 
        const TooN::Matrix<>& B, 
        const TooN::Matrix<>& C, 
        const TooN::Matrix<>& D,
        const TooN::Vector<>& initial_state);

LTI_SS( const TooN::Matrix<>& A, 
        const TooN::Matrix<>& B, 
        const TooN::Matrix<>& C,
        const TooN::Vector<>& initial_state);

LTI_SS( const TooN::Matrix<>& A, 
        const TooN::Matrix<>& B, 
        const TooN::Matrix<>& C, 
        const TooN::Matrix<>& D,
        double Ts );

LTI_SS( const TooN::Matrix<>& A, 
        const TooN::Matrix<>& B, 
        const TooN::Matrix<>& C, 
        const TooN::Matrix<>& D );

LTI_SS( const TooN::Matrix<>& A, 
        const TooN::Matrix<>& B, 
        const TooN::Matrix<>& C );

LTI_SS(const LTI_SS& tf) = default;
/*==============================================*/

/*===============DESTRUCTOR===================*/	
virtual ~LTI_SS() = default; //(destructor)
/*==============================================*/

/*=============GETTER===========================*/
virtual const TooN::Matrix<>& getA() const;
virtual const TooN::Matrix<>& getB() const;
virtual const TooN::Matrix<>& getC() const;
virtual const TooN::Matrix<>& getD() const;
virtual const TooN::Vector<>& getState() const;
virtual const unsigned int getDimInput() const override;
virtual const unsigned int getDimOutput() const override;
/*==============================================*/

/*=============SETTER===========================*/
virtual void setA(const TooN::Matrix<>& A);
virtual void setB(const TooN::Matrix<>& B);
virtual void setC(const TooN::Matrix<>& C);
virtual void setD(const TooN::Matrix<>& D);
virtual void setState(const TooN::Vector<>& state);
/*==============================================*/

/*=============SETTER FROM FILE===========================*/
/*========================================================*/

/*=============RUNNER===========================*/
/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state and the current input
*/
virtual TooN::Vector<> state_f( const TooN::Vector<>& state, const TooN::Vector<>& input ) const override;

/*
    This function does not change/use the internal state of the object
    It evaluates the next output given the previous state and the current input
*/
virtual TooN::Vector<> output_f( const TooN::Vector<>& state, const TooN::Vector<>& input ) const override;

/*==============================================*/

/*=============VARIE===========================*/
virtual void display() override;
virtual bool checkMatrixDims() const;
/*==============================================*/

};

using LTI_SS_Ptr = std::unique_ptr<LTI_SS>;


/*=============STATIC FUNS===========================*/

/*==============================================*/

#endif
