/*
    SS_Interface Class (State Space System)

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

#ifndef SS_INTERFACE_H
#define SS_INTERFACE_H

#include "Generic_System.h"

class SS_Interface : public Generic_System {

private:

SS_Interface(); //avoid default constructor

protected:

//State
TooN::Vector<> _state;

SS_Interface(const TooN::Vector<>& initial_state, double Ts) 
    : Generic_System(Ts),
      _state(initial_state)
      {};

SS_Interface(unsigned int order, double Ts) 
    : SS_Interface( TooN::Zeros(order), Ts )
      {};

public:

virtual ~SS_Interface() = default;

/*
    Clone the object
*/
virtual SS_Interface* clone() const = 0;

virtual const unsigned int getOrder() const 
{
    return _state.size();
}

/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state and the current input
*/
virtual TooN::Vector<> state_f( const TooN::Vector<>& state, const TooN::Vector<>& input ) const = 0;

/*
    This function does not change/use the internal state of the object
    It evaluates the next output given the previous state and the current input
*/
virtual TooN::Vector<> output_f( const TooN::Vector<>& state, const TooN::Vector<>& input ) const = 0;

/*
    This changes the internal state
    Evaluate the next state given the current input (and the internal state)
*/
virtual TooN::Vector<> apply_state( const TooN::Vector<>& input ) 
{

    _state = state_f( _state, input );

    return _state;

}

/*
    This changes the internal state
    Evaluate the next output given the current input (and the internal state)
*/
virtual TooN::Vector<> apply( const TooN::Vector<>& input ) override 
{

    _state = state_f( _state, input );

    return output_f( _state, input );

}

virtual void reset() override
{
    _state = TooN::Zeros;
}

virtual void display() override
{
    std::cout << "[SS_Interface]" << std::endl <<
                 "state = " << _state << std::endl;
}


};

using SS_Interface_Ptr = std::unique_ptr<SS_Interface>;

#endif