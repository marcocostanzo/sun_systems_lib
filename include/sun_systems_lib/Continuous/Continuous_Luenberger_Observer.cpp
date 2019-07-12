/*
    Continuous Time State Luenberger Observer System Class

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

#ifndef CONTINUOUS_LUENBERGER_OBSERVER_H
#define CONTINUOUS_LUENBERGER_OBSERVER_H

#include <sun_systems_lib/Continuous/Continuous_Observer_Interface.cpp>

class Continuous_Luenberger_Observer : public Continuous_Observer_Interface
{

private:

protected:

Continuous_System_Interface_Ptr system_;
TooN::Matrix<> L_;

public:

Continuous_Luenberger_Observer(const Continuous_System_Interface& system, const TooN::Matrix<> L )
    :system_( system.clone() ),
    L_(L)
    {
        if(!chek_dimensions())
        {
            throw std::invalid_argument( "[Continuous_Luenberger_Observer] Invalid matrix dimensions" );
        }
    }

Continuous_Luenberger_Observer(const Continuous_Luenberger_Observer& ss)
:system_( ss.system_->clone() ),
L_(ss.L_)
{}

virtual Continuous_Luenberger_Observer* clone() const override
{
    return new Continuous_Luenberger_Observer(*this);
}

virtual ~Continuous_Luenberger_Observer() override = default;

inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const override
{
    return system_->state_fcn(x_k_1,u_k) + L_ * ( y_k - system_->output_fcn(x_k_1,u_k) );
}

//virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
//{
//    const unsigned int size_real_input = getSizeRealInput();
//    return state_fcn(   x_k_1, 
//                        u_k.slice(0, size_real_input), 
//                        u_k.slice(size_real_input, getSizeOutput()) 
//                        );
//}

inline virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return system_->output_fcn( x_k, u_k );
}

inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k, const TooN::Vector<>& y_k ) const override
{
    return system_->jacob_state_fcn(x_k_1,u_k) - L_ * system_->jacob_output_fcn(x_k_1,u_k);
}

//virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
//{
//    const unsigned int size_real_input = getSizeRealInput();
//    return jacob_state_fcn( x_k_1, 
//                            u_k.slice(0, size_real_input), 
//                            u_k.slice(size_real_input, getSizeOutput()) 
//                            );
//}

inline virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return system_->jacob_output_fcn( x_k, u_k );
}

inline virtual const unsigned int getSizeRealInput() const
{
    return getSizeInput() - getSizeOutput();
}

inline virtual const unsigned int getSizeInput() const override
{
    return system_->getSizeInput() + system_->getSizeOutput();
}

inline virtual const unsigned int getSizeOutput() const override
{
    return system_->getSizeOutput();
}

inline virtual const unsigned int getSizeState() const
{
    return system_->getSizeState();
}

virtual void display() const
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Continuous_Luenberger_Observer" CRESET << std::endl;
}

virtual bool chek_dimensions() const
{
    if(L_.num_rows() != system_->getSizeState())
        return false;
    if(L_.num_cols() != system_->getSizeOutput())
        return false;

    return true;
}

};

using Continuous_Luenberger_Observer_Ptr = std::unique_ptr<Continuous_Luenberger_Observer>;

#endif