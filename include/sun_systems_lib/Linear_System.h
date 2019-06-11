/*
    Linear_System Class

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

#ifndef LINEAR_SYSTEM_H
#define LINEAR_SYSTEM_H

#include "sun_systems_lib/System_Interface.h"

class Linear_System : public System_Interface 
{
private:

protected:

public:

virtual Linear_System* clone() const = 0;

virtual ~Linear_System() override = default;

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) = 0;

inline virtual void reset() = 0;

virtual void display() const override
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Linear_System" CRESET << std::endl;
}

};

using Linear_System_Ptr = std::unique_ptr<Linear_System>;

#endif