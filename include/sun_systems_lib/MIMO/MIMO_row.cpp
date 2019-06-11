/*
    MIMO_row Class (array of SISO Systems)

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

#ifndef MIMO_ROW_H
#define MIMO_ROW_H

#include "sun_systems_lib/Zero_SISO.h"

class MIMO_row : public System_Interface
{

private:

protected:

std::vector<SISO_System_Ptr> siso_vect_;//std::vector representing the row vector of the SYSTEM (row wise)

TooN::Vector<> y_k_ = TooN::makeVector(0.0);

public:

MIMO_row( unsigned int dim_input )
{
    for( int i=0; i<dim_input; i++ )
        siso_vect_.push_back( SISO_System_Ptr( new Zero_SISO() ) );
}

MIMO_row( const MIMO_row& row )
{
    y_k_ = row.y_k_;
    for( auto& siso : row.siso_vect_ )
        siso_vect_.push_back( SISO_System_Ptr(siso->clone()) );
}

virtual ~MIMO_row() override = default;

virtual MIMO_row* clone() const override
{
    return new MIMO_row(*this);
}

//Operators
SISO_System& operator[](unsigned int i)
{
    return *siso_vect_[i];
}

const SISO_System& operator[](unsigned int i) const
{
    return *siso_vect_[i];
}

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    y_k_[0] = 0.0;
    for( int i = 0; i<siso_vect_.size(); i++ )
    {
        y_k_[0] += siso_vect_[i]->apply( input[i] );
    }
    return y_k_;
}

inline virtual void reset()
{
    for( auto& siso : siso_vect_ )
        siso->reset();
}

virtual void display() const
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for MIMO_row" CRESET << std::endl;
}

};

using MIMO_row_Ptr = std::unique_ptr<MIMO_row>;

#endif