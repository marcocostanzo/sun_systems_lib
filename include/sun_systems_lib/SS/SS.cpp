/*
    Generic SS from external functions

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

#ifndef SS_H
#define SS_H

#include <sun_systems_lib/SS/SS_Interface.cpp>
#include "boost/function.hpp"

typedef boost::function<TooN::Vector<>(const TooN::Vector<>&, const TooN::Vector<>&)> SS_FCN;
typedef boost::function<TooN::Matrix<>(const TooN::Vector<>&, const TooN::Vector<>&)> SS_JACOB_FCN;

class SS : public SS_Interface
{

private:

protected:

SS_FCN state_fcn_, output_fcn_;
SS_JACOB_FCN jacob_state_fcn_, jacob_output_fcn_;
unsigned int dim_output_, dim_input_;

public:

SS( unsigned int dim_state, 
    unsigned int dim_output, 
    unsigned int dim_input, 
    const SS_FCN& state_fcn, 
    const SS_FCN& output_fcn, 
    const SS_JACOB_FCN& jacob_state_fcn = NULL, 
    const SS_JACOB_FCN& jacob_output_fcn = NULL )
    :SS_Interface( TooN::Zeros(dim_state), TooN::Zeros(dim_output) ),
    state_fcn_(state_fcn),
    output_fcn_(output_fcn),
    jacob_state_fcn_(jacob_state_fcn),
    jacob_output_fcn_(jacob_output_fcn),
    dim_output_(dim_output),
    dim_input_(dim_input)
    {}

SS(const SS& ss) = default;

virtual SS* clone() const override
{
    return new SS(*this);
}

virtual ~SS() override = default;

inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    return state_fcn_(x_k_1, u_k);
}

inline virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return output_fcn_(x_k, u_k);
}

inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    return jacob_state_fcn_(x_k_1, u_k);
}

inline virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return jacob_output_fcn_(x_k, u_k);
}

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    state_ = state_fcn_( state_, input );
    output_ = output_fcn_( state_, input );
    return output_;
}

virtual const unsigned int getSizeInput() const override
{
    return dim_input_;
}

virtual const unsigned int getSizeOutput() const override
{
    return dim_output_;
}

virtual void display() const override
{
    std::cout <<
    "SS:" << std::endl <<
    "Functions are defined run-time" << std::endl <<
    "state: " << state_ << std::endl <<
    "SS [END]" << std::endl;
}

};

using SS_Ptr = std::unique_ptr<SS>;

#endif