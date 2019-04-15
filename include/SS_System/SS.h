/*
    SS Class (State Space System)

    Copyright 2019 Università della Campania Luigi Vanvitelli

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

#ifndef SS_H
#define SS_H

#include "boost/function.hpp"
#include "SS_System/SS_Interface.h"

typedef boost::function<TooN::Vector<>(const TooN::Vector<>&, const TooN::Vector<>&)> SS_FCN;

class SS : public SS_Interface {

private:

SS(); //avoid default constructor

protected:

//State
SS_FCN _state_fcn;
SS_FCN _output_fcn;

unsigned int _dimInput;
unsigned int _dimOutput;

public:
    
virtual ~SS() = default;

SS( unsigned int dim_input,
    unsigned int dim_output,
    const SS_FCN& state_fcn, 
    const SS_FCN& output_fcn, 
    TooN::Vector<>& initial_state, 
    double Ts);

SS( unsigned int dim_input,
    unsigned int dim_output,
    const SS_FCN& state_fcn, 
    const SS_FCN& output_fcn, 
    unsigned int order, 
    double Ts);

virtual const unsigned int getDimInput() const override;
virtual const unsigned int getDimOutput() const override;

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

virtual void display() override;

};

using SS_Ptr = std::unique_ptr<SS>;

#endif