/*
    Generic_System Class

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

#ifndef GENERIC_SYSTEM_LIB
#define GENERIC_SYSTEM_LIB

#include <TooN/TooN.h>
#include <Helper.h>
#include <memory>

class Generic_System
{
private:

    double _Ts;

protected:
    //Generic_System();
    
    virtual ~Generic_System();

    Generic_System(double Ts) : _Ts(Ts){};

public:

    virtual TooN::Vector<> apply( TooN::Vector<> input ) = 0;

    virtual void reset() = 0;

    virtual double getTs(){return _Ts;}

    virtual void display(){
        std::cout << BOLDYELLOW "WARNING! disp() not implemented for derived of Generic_System" CRESET << std::endl;
    }

    /*
            Clone the object
    */
    virtual Generic_System* clone() const = 0;

};

using Generic_System_Ptr = std::unique_ptr<Generic_System>;

#endif