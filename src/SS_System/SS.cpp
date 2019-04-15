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

#include "SS_System/SS.h"

using namespace TooN;

SS::SS( unsigned int dim_input,
        unsigned int dim_output, 
        const SS_FCN& state_fcn, 
        const SS_FCN& output_fcn, 
        Vector<>& initial_state, 
        double Ts) 
    : SS_Interface(initial_state, Ts),
      _state_fcn(state_fcn),
      _output_fcn(output_fcn),
      _dimInput(dim_input),
      _dimOutput(dim_output)
      {}

SS::SS( unsigned int dim_input,
        unsigned int dim_output,
        const SS_FCN& state_fcn, 
        const SS_FCN& output_fcn, 
        unsigned int order, 
        double Ts) 
    : SS_Interface(order, Ts),
      _state_fcn(state_fcn),
      _output_fcn(output_fcn),
      _dimInput(dim_input),
      _dimOutput(dim_output)
      {}

/*
    Clone the object
*/
SS* SS::clone() const
{
    return new SS(*this);
}

const unsigned int SS::getDimInput() const {
    return _dimInput;
}

const unsigned int SS::getDimOutput() const {
    return _dimOutput;
}

/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state and the current input
*/
Vector<> SS::state_f( const Vector<>& state, const Vector<>& input ) const
{
    return _state_fcn( state, input );
}

/*
    This function does not change/use the internal state of the object
    It evaluates the next output given the previous state and the current input
*/
Vector<> SS::output_f( const Vector<>& state, const Vector<>& input ) const
{
    return _output_fcn( state, input );
}

void SS::display()
{
    std::cout << "[SS]" << std::endl <<
                 "state = " << _state << std::endl;
}
