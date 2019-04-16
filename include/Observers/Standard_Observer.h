/*
    Standard OBSERVER Class

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

#ifndef STANDARD_OBSERVER_H
#define STANDARD_OBSERVER_H

#include "Observers/Observer_Interface.h"

class Standard_Observer : public Observer_Interface {

private:

Standard_Observer(); //avoid default constructor

protected:

TooN::Matrix<> _L;

public:

Standard_Observer( 
    const SS_Interface& model,
    TooN::Matrix<>& L,
    const TooN::Vector<>& initial_state
    );

Standard_Observer(
    const SS_Interface& model,
    TooN::Matrix<>& L,
    int order);

Standard_Observer( const Standard_Observer& obs ) = default;

virtual Standard_Observer* clone() const override;

virtual ~Standard_Observer() = default;

/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state, the current input and the current measure
*/
virtual TooN::Vector<> state_f( const TooN::Vector<>& state, const TooN::Vector<>& input, const TooN::Vector<>& measure ) const override;

/*
    This changes the internal state
    Evaluate the next output given the current input (and the internal state)
    Since this is an observer the input is the concat of the effective input (u) ans the measure (y)
    i.e. input = [u ; y]
*/
// Code from base class
//virtual TooN::Vector<> apply( const TooN::Vector<>& input ) override;

virtual void display() override;


};

using Standard_Observer_Ptr = std::unique_ptr<Standard_Observer>;

#endif