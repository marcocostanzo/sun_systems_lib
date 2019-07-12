/*
    State Space Interface Class

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

#include <sun_systems_lib/Discrete_System_Interface.h>

class SS_Interface : public Discrete_System_Interface
{

private:

protected:

TooN::Vector<> state_;
TooN::Vector<> output_;

SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
            :state_(state),
            output_(output)
            {}

public:

virtual SS_Interface* clone() const override = 0;

virtual ~SS_Interface() override = default;

virtual const TooN::Vector<>& getState() const
{
    return state_;
}

virtual void setState(const TooN::Vector<>& state)
{
    state_ = state;
}

virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const = 0;

virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const = 0;

virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    state_ = state_fcn( state_, input );
    output_ = output_fcn( state_, input );
    return output_;
}

virtual void reset() override
{
    state_ = TooN::Zeros;
    output_ = TooN::Zeros;
}

virtual const unsigned int getSizeInput() const override = 0;

virtual const unsigned int getSizeOutput() const override = 0;

virtual const unsigned int getSizeState() const
{
    return state_.size();
}

virtual void display() const override
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for SS_Interface" CRESET << std::endl;
}

};

using SS_Interface_Ptr = std::unique_ptr<SS_Interface>;

#endif