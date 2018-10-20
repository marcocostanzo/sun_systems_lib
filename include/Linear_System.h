/*
    Linear_System Class

    Copyright 2018 Università della Campania Luigi Vanvitelli

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

#ifndef LINEAR_SYSTEM_LIB
#define LINEAR_SYSTEM_LIB

#include "Generic_System.h"

class Linear_System : public Generic_System {
    private:

    protected:

    public:
    virtual ~Linear_System();
    Linear_System(double Ts) : Generic_System(Ts){};

};

#endif