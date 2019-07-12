/*
    Continuous Time State Observer System Interface Class

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

#ifndef CONTINUOUS_OBSERVER_INTERFACE_H
#define CONTINUOUS_OBSERVER_INTERFACE_H

#include <sun_systems_lib/Continuous/Continuous_System_Interface.cpp>

class Continuous_Observer_Interface : public Continuous_System_Interface
{

private:

protected:

public:

virtual Continuous_Observer_Interface* clone() const = 0;

virtual ~Continuous_Observer_Interface() = default;

virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const = 0;

virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    const unsigned int size_real_input = getSizeRealInput();
    return state_fcn(   x_k_1, 
                        u_k.slice(0, size_real_input), 
                        u_k.slice(size_real_input, getSizeOutput()) 
                        );
}

virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const = 0;

virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    const unsigned int size_real_input = getSizeRealInput();
    return jacob_state_fcn( x_k_1, 
                            u_k.slice(0, size_real_input), 
                            u_k.slice(size_real_input, getSizeOutput()) 
                            );
}

virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

virtual const unsigned int getSizeRealInput() const
{
    return getSizeInput() - getSizeOutput();
}

virtual const unsigned int getSizeInput() const override = 0;

virtual const unsigned int getSizeOutput() const override = 0;

virtual const unsigned int getSizeState() const = 0;

virtual void display() const
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Continuous_Observer_Interface" CRESET << std::endl;
}

};

using Continuous_Observer_Interface_Ptr = std::unique_ptr<Continuous_Observer_Interface>;

#endif