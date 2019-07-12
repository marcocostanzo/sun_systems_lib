/*
    State Space Continuous Time System Class

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

#ifndef CONTINUOUS_SYSTEM_H
#define CONTINUOUS_SYSTEM_H

#include <sun_systems_lib/Continuous/Continuous_System_Interface.cpp>
#include "boost/function.hpp"

#ifndef SS_FUNCTION_TYPES
#define SS_FUNCTION_TYPES 
typedef boost::function<TooN::Vector<>(const TooN::Vector<>&, const TooN::Vector<>&)> SS_FCN;
typedef boost::function<TooN::Matrix<>(const TooN::Vector<>&, const TooN::Vector<>&)> SS_JACOB_FCN;
#endif

class Continuous_System : public Continuous_System_Interface
{

private:

protected:

SS_FCN state_fcn_, output_fcn_;
SS_JACOB_FCN jacob_state_fcn_, jacob_output_fcn_;
unsigned int dim_state_, dim_output_, dim_input_;

public:

Continuous_System( 
    unsigned int dim_state, 
    unsigned int dim_output, 
    unsigned int dim_input, 
    const SS_FCN& state_fcn, 
    const SS_FCN& output_fcn, 
    const SS_JACOB_FCN& jacob_state_fcn = NULL, 
    const SS_JACOB_FCN& jacob_output_fcn = NULL )
    :
    dim_state_(dim_state),
    dim_output_(dim_output),
    dim_input_(dim_input),
    state_fcn_(state_fcn),
    output_fcn_(output_fcn),
    jacob_state_fcn_(jacob_state_fcn),
    jacob_output_fcn_(jacob_output_fcn)
    {}

/*
Continuous_System(  
    const SS_FCN& state_fcn, 
    const SS_FCN& output_fcn, 
    const SS_JACOB_FCN& jacob_state_fcn = NULL, 
    const SS_JACOB_FCN& jacob_output_fcn = NULL 
    )
    :Continuous_System( 
        0, 
        0, 
        0, 
        state_fcn, 
        output_fcn, 
        jacob_state_fcn, 
        jacob_output_fcn 
        )
    {}
    */

Continuous_System(const Continuous_System& ss) = default;

virtual Continuous_System* clone() const override
{
    return new Continuous_System(*this);
}

virtual ~Continuous_System() override = default;

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

virtual const unsigned int getSizeInput() const override
{
    return dim_input_;
}

virtual const unsigned int getSizeOutput() const override
{
    return dim_output_;
}

virtual const unsigned int getSizeState() const override
{
    return dim_state_;
}

virtual void display() const
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Continuous_System_Interface" CRESET << std::endl;
}

};

using Continuous_System_Ptr = std::unique_ptr<Continuous_System>;

#endif