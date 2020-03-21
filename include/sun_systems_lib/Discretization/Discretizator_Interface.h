/*
    State Space Discretizator Interface Class, Continuous to Discrete

    Copyright 2019-2020 Università della Campania Luigi Vanvitelli

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

#ifndef DISCRETIZATOR_INTERFACE_H
#define DISCRETIZATOR_INTERFACE_H

/*! \file Discretizator_Interface.h
    \brief This class is an interface for a generic Discretizzator
*/

#include <sun_systems_lib/SS/SS_Interface.h>

namespace sun
{
//!  Discretizator_Interface class: interface for a generic Discretizzator.
/*!
    Is a State Space system obtained as discretizzation of a continuous system.

    \sa Discretizator_Interface, RK4, Discrete_System_Interface
*/
class Discretizator_Interface : public SS_Interface
{
private:
protected:
  // TooN::Vector<> state_;
  // TooN::Vector<> output_;

  // SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
  //            :state_(state),
  //            output_(output)
  //            {}

  //! Constructor
  /*
    This constructor simply forward the arguments to SS_Interface(const TooN::Vector<>&,unsigned int)
  */
  Discretizator_Interface(const TooN::Vector<>& state, unsigned int dim_output) : SS_Interface(state, dim_output)
  {
  }

public:
  virtual Discretizator_Interface* clone() const override = 0;

  //! Desctructor
  virtual ~Discretizator_Interface() override = default;

  // virtual const TooN::Vector<>& getState() const
  //{
  //    return state_;
  //}

  // virtual void setState(const TooN::Vector<>& state)
  //{
  //    state_ = state;
  //}

  // virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const = 0;

  // virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

  // virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const = 0;

  // virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const = 0;

  // virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
  //{
  //    state_ = state_fcn( state_, input );
  //    output_ = output_fcn( state_, input );
  //    return output_;
  //}

  // virtual void reset() override
  //{
  //    state_ = TooN::Zeros;
  //    output_ = TooN::Zeros;
  //}

  // virtual const unsigned int getSizeInput() const override = 0;

  // virtual const unsigned int getSizeOutput() const override = 0;

  // virtual const unsigned int getSizeState() const
  //{
  //    return state_.size();
  //}

  virtual void display() const override
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Discretizator_Interface" CRESET << std::endl;
  }
};

using Discretizator_Interface_Ptr = std::unique_ptr<Discretizator_Interface>;

}  // namespace sun

#endif