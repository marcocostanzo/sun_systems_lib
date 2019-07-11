/*
    TF MIMO Diagonal Class

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

#ifndef TF_MIMO_DIAGONAL_H
#define TF_MIMO_DIAGONAL_H

#include <sun_systems_lib/TF/TF_MIMO.cpp>

class TF_MIMO_DIAGONAL : public TF_MIMO
{

private:

TF_MIMO_DIAGONAL();

protected:

public:

TF_MIMO_DIAGONAL( unsigned int dim )
:TF_MIMO(dim,dim)
{}

TF_MIMO_DIAGONAL( const TF_MIMO_DIAGONAL& mimo )
:TF_MIMO(mimo)
{}

virtual TF_MIMO_DIAGONAL* clone() const override
{
    return new TF_MIMO_DIAGONAL(*this);
}

virtual ~TF_MIMO_DIAGONAL() override = default;

///////////////////////////////////

virtual void setSISO( unsigned int index_row, unsigned int index_col, const TF_SISO& siso ) override
{
    if( index_row != index_col )
    {
        throw std::out_of_range( "[TF_MIMO_DIAGONAL::setSISO] It is possible to set only SISO on the diagonal" );
    }

    TF_MIMO::setSISO( index_row, index_col, siso );

}

virtual void setSISO( unsigned int index_diag, const TF_SISO& siso )
{
    TF_MIMO::setSISO( index_diag, index_diag, siso );
}

//////////////////////////////////

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
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
    "TF_MIMO_DIAGONAL " << dim_input_ << "x" << dim_input_ << ":" << std::endl;

    for(int i = 0; i < dim_input_ ; i++)
    {
        str << "Position [" << i << "][" << i << "]" << std::endl;
        siso_vect_[ i*dim_input_ + i ]->display();
        str << "-----------------------------------" << std::endl;
    }

    str << "TF_MIMO_DIAGONAL [END]" << std::endl;

    std::cout << str.str();
}

};

using TF_MIMO_DIAGONAL_Ptr = std::unique_ptr<TF_MIMO_DIAGONAL>;

#endif