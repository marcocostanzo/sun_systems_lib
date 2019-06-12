/*
    MIMO Matrix Class

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

#ifndef MIMO_MATRIX_H
#define MIMO_MATRIX_H

#include <sun_systems_lib/Zero_SISO.h>

class MIMO_Matrix : public System_Interface
{

private:

MIMO_Matrix();

protected:

std::vector<SISO_System_Ptr> siso_vect_;

unsigned int dim_input_;

TooN::Vector<> y_k_;

public:

MIMO_Matrix( unsigned int dim_input, unsigned int dim_output )
:dim_input_(dim_input),
y_k_( TooN::Zeros(dim_output) )
{
    int num_siso = dim_input * dim_output;
    for(int i=0; i<num_siso; i++)
    {
        siso_vect_.push_back( SISO_System_Ptr( new Zero_SISO() ) );
    }
}

MIMO_Matrix( const MIMO_Matrix& mimo )
:y_k_(mimo.siso_vect_.size()/mimo.dim_input_)
{
    dim_input_ = mimo.dim_input_;
    for( const auto& siso : mimo.siso_vect_ )
    {
        siso_vect_.push_back( SISO_System_Ptr( siso->clone() ) );
    }
}

virtual MIMO_Matrix* clone() const override
{
    return new MIMO_Matrix(*this);
}

virtual ~MIMO_Matrix() override = default;

///////////////////////////////////

virtual void setSISO( unsigned int index_row, unsigned int index_col, const SISO_System& siso )
{
    if( index_row > (siso_vect_.size()/dim_input_) || index_col > dim_input_ )
    {
        throw std::out_of_range( "[MIMO_Matrix::setSISO] Index out of range" );
    }

    siso_vect_[ index_row*dim_input_ + index_col ] = SISO_System_Ptr( siso.clone() );

}

//////////////////////////////////

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    y_k_ = TooN::Zeros;
    int dim_output = siso_vect_.size()/dim_input_;
    for(int i = 0; i < dim_output ; i++)
    {
        for(int j = 0; j < dim_input_; j++ )
        {
            y_k_[i] += siso_vect_[ i*dim_input_ + j ]->apply(input[j]);
        }
    }
    return y_k_;
}

inline virtual void reset() override
{
    y_k_ = TooN::Zeros;
    for( auto& siso: siso_vect_ )
    {
        siso->reset();
    }
}

virtual void display() const
{
    int dim_output = siso_vect_.size()/dim_input_;
    std::stringstream str;
    str <<
    "MIMO_Matrix " << dim_output << "x" << dim_input_ << ":" << std::endl;

    for(int i = 0; i < dim_output ; i++)
    {
        for(int j = 0; j < dim_input_; j++ )
        {
            str << "Position [" << i << "][" << j << "]" << std::endl;
            siso_vect_[ i*dim_input_ + j ]->display();
            str << "-----------------------------------" << std::endl;
        }
    }

    str << "MIMO_Matrix [END]" << std::endl;

    std::cout << str.str();
}

};

using MIMO_Matrix_Ptr = std::unique_ptr<MIMO_Matrix>;

#endif