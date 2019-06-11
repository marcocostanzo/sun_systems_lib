/*
    Zero_SISO Class

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

#ifndef ZERO_SISO_H
#define ZERO_SISO_H

#include "sun_systems_lib/SISO_System.h"

class Zero_SISO : public SISO_System 
{
private:

protected:

TooN::Vector<> zero_v_ = TooN::makeVector(0.0);

public:

Zero_SISO(){}

virtual Zero_SISO* clone() const override
{
    return new Zero_SISO(*this);
}

virtual ~Zero_SISO() override = default;

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    return zero_v_;
}

inline virtual double apply( double input ) override
{
    return 0.0;
}

inline virtual void reset() override {}

virtual void display() const override
{
    std::cout <<
    "Zero_SISO" << std::endl;
}

};

using Zero_SISO_Ptr = std::unique_ptr<Zero_SISO>;

#endif