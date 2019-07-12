/*
    Linear_System_Interface Class, Interface for any linear system

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

#ifndef LINEAR_SYSTEM_INTERFACE_H
#define LINEAR_SYSTEM_INTERFACE_H

#include "sun_systems_lib/Discrete_System_Interface.h"

class Linear_System_Interface : public Discrete_System_Interface 
{
private:

protected:

public:

virtual Linear_System_Interface* clone() const override = 0;

virtual ~Linear_System_Interface() override = default;

virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override = 0;

virtual void reset() override = 0;

virtual const unsigned int getSizeInput() const override = 0;

virtual const unsigned int getSizeOutput() const override = 0;

virtual void display() const override
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Linear_System_Interface" CRESET << std::endl;
}

};

using Linear_System_Interface_Ptr = std::unique_ptr<Linear_System_Interface>;

#endif