/*
    RK4 Runge Kutta 4 State Space Discretizator Interface Class, Continuous to Discrete

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

#ifndef RK4_H
#define RK4_H

#include <sun_systems_lib/Discretization/Discretizator_Interface.cpp>
#include <sun_systems_lib/Continuous/Continuous_System_Interface.cpp>

class RK4 : public Discretizator_Interface
{

private:

protected:

////TooN::Vector<> state_;
TooN::Vector<> u_n_1_;
////TooN::Vector<> output_;
Continuous_System_Interface_Ptr system_;
double Ts_, Ts_2_, Ts_6_;
TooN::Matrix<> Identity_x_;

////SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
////            :state_(state),
////            output_(output)
////            {}

//Discretizator_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
//    :SS_Interface( state, output )
//    {}

public:

RK4(
    const Continuous_System_Interface& system,
    double Ts
    )
    :Discretizator_Interface( TooN::Zeros(system.getSizeState()), TooN::Zeros(system.getSizeOutput()) ),
    u_n_1_(TooN::Zeros(system.getSizeOutput())),
    system_( system.clone() ),
    Ts_(Ts),
    Ts_2_(Ts/2.0),
    Ts_6_(Ts/6.0),
    Identity_x_( TooN::Identity(system.getSizeState()) )
    {}

RK4(const RK4& ss)
    :Discretizator_Interface( ss ),
    u_n_1_(ss.u_n_1_),
    system_( ss.system_->clone() ),
    Ts_(ss.Ts_),
    Ts_2_(ss.Ts_2_),
    Ts_6_(ss.Ts_6_),
    Identity_x_( ss.Identity_x_ )
    {}

virtual RK4* clone() const override
{
    return new RK4(*this);
}

virtual ~RK4() override = default;

////virtual const TooN::Vector<>& getState() const
////{
////    return state_;
////}

////virtual void setState(const TooN::Vector<>& state)
////{
////    state_ = state;
////}

inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_n_1, const TooN::Vector<>& u_n, const TooN::Vector<>& u_n_1 ) const
{
    TooN::Vector<> u_n_12 = estimateMeanInputs(u_n, u_n_1);

    TooN::Vector<> k1 = system_->state_fcn( x_n_1 , u_n_1 );
    TooN::Vector<> k2 = system_->state_fcn( x_n_1 + Ts_2_*k1 , u_n_12 );
    TooN::Vector<> k3 = system_->state_fcn( x_n_1 + Ts_2_*k2 , u_n_12 );
    TooN::Vector<> k4 = system_->state_fcn( x_n_1 + Ts_*k3 , u_n );

    return x_n_1 + Ts_6_ * ( k1 + 2.0*k2 + 2.0*k3 + k4 ); //=x_n
}

// To make this function stateless, u_k_1 = u_k
inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    return state_fcn( x_k_1, u_k, u_k );
}

inline virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return system_->output_fcn( x_k , u_k );
}

inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_n_1, const TooN::Vector<>& u_n, const TooN::Vector<>& u_n_1 ) const
{
    TooN::Vector<> u_n_12 = estimateMeanInputs(u_n, u_n_1);

    TooN::Vector<> k1 = system_->state_fcn( x_n_1 , u_n_1 );
    TooN::Vector<> k2 = system_->state_fcn( x_n_1 + Ts_2_*k1 , u_n_12 );
    TooN::Vector<> k3 = system_->state_fcn( x_n_1 + Ts_2_*k2 , u_n_12 );

    TooN::Matrix<> jac_k1 = system_->jacob_state_fcn( x_n_1 , u_n_1 );
    TooN::Matrix<> jac_k2 = system_->jacob_state_fcn( x_n_1 + Ts_2_*k1 , u_n_12 ) * ( Identity_x_ + Ts_2_ * jac_k1 );
    TooN::Matrix<> jac_k3 = system_->jacob_state_fcn( x_n_1 + Ts_2_*k2 , u_n_12 ) * ( Identity_x_ + Ts_2_ * jac_k2 );
    TooN::Matrix<> jac_k4 = system_->jacob_state_fcn( x_n_1 + Ts_*k3 , u_n ) * ( Identity_x_ + Ts_ * jac_k3 );

    return Identity_x_ + Ts_6_ * ( jac_k1 + 2.0*jac_k2 + 2.0*jac_k3 + jac_k4 ); //=jac_n
}

// To make this function stateless, u_k_1 = u_k
inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    return jacob_state_fcn( x_k_1, u_k, u_k );
}

virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return system_->jacob_output_fcn( x_k , u_k );
}

virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    state_ = state_fcn( state_, input, u_n_1_ );
    u_n_1_ = input;
    output_ = output_fcn( state_, input );
    return output_;
}

inline virtual TooN::Vector<> estimateMeanInputs(const TooN::Vector<>& u_n, const TooN::Vector<>& u_n_1) const
{
    return (u_n + u_n_1)/2.0; //=u_n_12
}

virtual void reset() override
{
    Discretizator_Interface::reset();
    u_n_1_ = TooN::Zeros;
}

virtual const unsigned int getSizeInput() const override
{
    return system_->getSizeInput();
}

virtual const unsigned int getSizeOutput() const override
{
    return system_->getSizeInput();
}

////virtual const unsigned int getSizeState() const
////{
////    return state_.size();
////}

virtual void display() const override
{
    std::cout << BOLDYELLOW "WARNING! display() not implemented for RK4" CRESET << std::endl;
}

};

using RK4_Ptr = std::unique_ptr<RK4>;

#endif