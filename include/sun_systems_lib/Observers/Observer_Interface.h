/*
    State Observer Interface Class

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

#ifndef OBSERVER_INTERFACE_H
#define OBSERVER_INTERFACE_H

#include <sun_systems_lib/SS/SS_Interface.h>

class Observer_Interface : public SS_Interface
{

private:

protected:

//TooN::Vector<> state_;
//TooN::Vector<> output_;

//SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
//            :state_(state),
//            output_(output)
//            {}

Observer_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
            :SS_Interface( state, output )
            {}

public:

virtual Observer_Interface* clone() const override = 0;

virtual ~Observer_Interface() override = default;

//virtual const TooN::Vector<>& getState() const
//{
//    return state_;
//}

//virtual void setState(const TooN::Vector<>& state)
//{
//    state_ = state;
//}

virtual const TooN::Vector<> obs_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const = 0;

inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    const unsigned int size_real_input = getSizeRealInput();
    return obs_state_fcn(   x_k_1, 
                        u_k.slice(0, size_real_input), 
                        u_k.slice(size_real_input, getSizeOutput()) 
                        );
}

virtual const TooN::Vector<> obs_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

inline virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    const unsigned int size_real_input = getSizeRealInput();
    return obs_output_fcn(   x_k, 
                        u_k.slice(0, size_real_input)
                        );
}

virtual const TooN::Matrix<> obs_jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const = 0;

virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    const unsigned int size_real_input = getSizeRealInput();
    return obs_jacob_state_fcn( x_k_1, 
                            u_k.slice(0, size_real_input), 
                            u_k.slice(size_real_input, getSizeOutput()) 
                            );
}

virtual const TooN::Matrix<> obs_jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    const unsigned int size_real_input = getSizeRealInput();
    return obs_jacob_output_fcn(   x_k, 
                        u_k.slice(0, size_real_input)
                        );
}

virtual const TooN::Vector<>& obs_apply( const TooN::Vector<>& input, const TooN::Vector<>& measure )
{
    state_ = obs_state_fcn( state_, input, measure );
    output_ = obs_output_fcn( state_, input );
    return output_;
} 

//virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
//{
//    state_ = state_fcn( state_, input );
//    output_ = output_fcn( state_, input );
//    return output_;
//}

//virtual void reset() override
//{
//    state_ = TooN::Zeros;
//    output_ = TooN::Zeros;
//}

virtual const unsigned int getSizeRealInput() const
{
    return getSizeInput() - getSizeOutput();
}

virtual const unsigned int getSizeInput() const override = 0;

virtual const unsigned int getSizeOutput() const override = 0;

//virtual const unsigned int getSizeState() const
//{
//    return state_.size();
//}

virtual void display() const override
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Observer_Interface" CRESET << std::endl;
}

inline virtual TooN::Vector<> buildFullInput( const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const
{
    TooN::Vector<> full_input = TooN::Zeros( getSizeInput() );
    const unsigned int size_real_input = getSizeRealInput();
    full_input.slice( 0, size_real_input ) = u_k;
    full_input.slice( size_real_input, getSizeOutput() ) = y_k;
    return full_input;
}

};

using Observer_Interface_Ptr = std::unique_ptr<Observer_Interface>;

#endif