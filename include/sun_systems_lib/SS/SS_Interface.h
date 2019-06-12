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

#include <sun_systems_lib/System_Interface.h>

class SS_Interface : public System_Interface
{

private:

protected:

public:

virtual SS_Interface* clone() const = 0;

virtual ~SS_Interface() = default;

virtual const TooN::Vector<>& get_state() const = 0;

virtual const TooN::Vector<>& state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) = 0;

virtual const TooN::Vector<>& output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) = 0;

virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) = 0;

virtual void reset() = 0;

virtual void display() const
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for SS_Interface" CRESET << std::endl;
}

};

using SS_Interface_Ptr = std::unique_ptr<SS_Interface>;

#endif