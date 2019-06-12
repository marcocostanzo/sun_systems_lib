/*
    MIMO Diagonal Class

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

#ifndef MIMO_DIAGONAL_H
#define MIMO_DIAGONAL_H

#include <sun_systems_lib/MIMO/MIMO_Matrix.h>

class MIMO_Diagonal : public MIMO_Matrix
{

private:

MIMO_Diagonal();

protected:

public:

MIMO_Diagonal( unsigned int dim )
:MIMO_Matrix(dim,dim)
{}

MIMO_Diagonal( const MIMO_Diagonal& mimo )
:MIMO_Matrix(mimo)
{}

virtual MIMO_Diagonal* clone() const override
{
    return new MIMO_Diagonal(*this);
}

virtual ~MIMO_Diagonal() override = default;

///////////////////////////////////

virtual void setSISO( unsigned int index_row, unsigned int index_col, const SISO_System& siso ) override
{
    if( index_row != index_col )
    {
        throw std::out_of_range( "[MIMO_Diagonal::setSISO] It is possible to set only SISO on the diagonal" );
    }

    MIMO_Matrix::setSISO( index_row, index_col, siso );

}

virtual void setSISO( unsigned int index_diag, const SISO_System& siso )
{
    MIMO_Matrix::setSISO( index_diag, index_diag, siso );
}

//////////////////////////////////

virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    int dim_output = siso_vect_.size()/dim_input_;
    for(int i = 0; i < dim_input_ ; i++)
    {
        y_k_[i] = siso_vect_[ i*dim_input_ + i ]->apply(input[i]);
    }
    return y_k_;
}

virtual void display() const
{
    std::stringstream str;
    str <<
    "MIMO_Diagonal " << dim_input_ << "x" << dim_input_ << ":" << std::endl;

    for(int i = 0; i < dim_input_ ; i++)
    {
        str << "Position [" << i << "][" << i << "]" << std::endl;
        siso_vect_[ i*dim_input_ + i ]->display();
        str << "-----------------------------------" << std::endl;
    }

    str << "MIMO_Diagonal [END]" << std::endl;

    std::cout << str.str();
}

};

using MIMO_Diagonal_Ptr = std::unique_ptr<MIMO_Diagonal>;

#endif