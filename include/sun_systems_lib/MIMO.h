/*
    MIMO Class (array of SISO Systems)

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

#ifndef MIMO_H
#define MIMO_H

#include "sun_systems_lib/MIMO/MIMO_row.h"

class MIMO : public System_Interface
{

private:

protected:

std::vector<MIMO_row_Ptr> siso_vect_;//std::vector representing the Matrix of the MIMO SYSTEM (row wise)

double ts_;

public:

MIMO( unsigned int dim_input, unsigned int dim_output, double Ts = NAN )
:ts_(Ts),
{
    for(int i=0; i<dim_output; i++)
        siso_vect_.push_back(MIMO_row_Ptr( new MIMO_row(dim_input) ));
}

TF_MIMO( const TF_MIMO& tf ) = default;

virtual ~TF_MIMO() = default;

virtual TF_MIMO* clone() const
{
    return new MINO(*this);
}

//Operators


inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) = 0;

inline virtual void reset() = 0;

virtual void display() const
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for System_Interface" CRESET << std::endl;
}

};

#endif