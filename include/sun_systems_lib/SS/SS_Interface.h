/*
    State Space Interface Class

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

#ifndef SS_INTERFACE_H
#define SS_INTERFACE_H

/*! \file SS_Interface.h
    \brief This class represents a generic Discrete Time State Space System
*/

#include <sun_systems_lib/Discrete_System_Interface.h>

namespace sun
{
//!  SS_Interface class: represents a generic Discrete Time State Space System.
/*!
    This class is a generic discrete time SS system

    \verbatim

    u(k) /-------\  y(k)
    ---->|  Sys  |---->
         \-------/
           ^    |
       x(k)|    |x(k-1)
           \----/

    x(k) = f(x(k-1),u(k))
    y(k) = h(x(k),u(k))

    \endverbatim

    It stores the internal system state

    \sa Discretizator_Interface, RK4, Discrete_System_Interface
*/
class SS_Interface : public Discrete_System_Interface
{
private:
protected:
  //! Sytem state
  TooN::Vector<> state_;

  //! Output (for internal reference)
  TooN::Vector<> output_;

  //!  Full Constructor
  /*!
      Costructor that inizialize the state
      \param state initial state
      \param dim_output output dimention
  */
  SS_Interface(const TooN::Vector<>& state, unsigned int dim_output) : state_(state), output_(TooN::Zeros(dim_output))
  {
  }

public:
  virtual SS_Interface* clone() const override = 0;

  //! Desctructor
  virtual ~SS_Interface() override = default;

  //! Get the internal State
  virtual const TooN::Vector<>& getState() const
  {
    return state_;
  }

  //! Set the internal state
  virtual void setState(const TooN::Vector<>& state)
  {
    state_ = state;
  }

  //!  State transition function
  /*!
      This is the state space trnasition funcion. This computes:

      \verbatim
      x(k) = f(x(k-1),u(k))  
      \endverbatim

      Note: This method does NOT update the internal state, use apply instead

      \param x_k_1 x(k-1), previous state
      \param u_k input
  */
  virtual const TooN::Vector<> state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k) const = 0;

  //!  Output function
  /*!
      This is the output funcion. This computes:

      \verbatim
      y(k) = h(x(k),u(k))  
      \endverbatim

      Note: This method does NOT update the internal state, use apply instead

      \param x_k x(k), state
      \param u_k input
  */
  virtual const TooN::Vector<> output_fcn(const TooN::Vector<>& x_k, const TooN::Vector<>& u_k) const = 0;

  //!  State function Jacobian
  /*!
      This is the state funcion Jacobian. This computes:

      \verbatim
      F = jacob_state(x(k-1),u(k))  
      \endverbatim

      Note: This method does NOT update the internal state, use apply instead

      \param x_k_1 x(k-1), previous state
      \param u_k input
  */
  virtual const TooN::Matrix<> jacob_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k) const = 0;

  //!  Output function Jacobian
  /*!
      This is the output funcion Jacobian. This computes:

      \verbatim
      H = jacob_output(x(k),u(k))  
      \endverbatim

      Note: This method does NOT update the internal state, use apply instead

      \param x_k x(k), state
      \param u_k input
  */
  virtual const TooN::Matrix<> jacob_output_fcn(const TooN::Vector<>& x_k, const TooN::Vector<>& u_k) const = 0;

  //! Apply the system, compute the output and update the internal state
  /*!
    Go one discrete step ahead, apply the input u(k), update the internal state for the next step,
    return the output y(k)

    \verbatim
      x(k) = f(x(k-1),u(k))
      y(k) = h(x(k),u(k))
    \endverbatim

    \param u_k Input at the current step u(k)
    \return system output y(k)
  */
  virtual const TooN::Vector<>& apply(const TooN::Vector<>& u_k) override
  {
    state_ = state_fcn(state_, u_k);
    output_ = output_fcn(state_, u_k);
    return output_;
  }

  virtual void reset() override
  {
    state_ = TooN::Zeros;
    output_ = TooN::Zeros;
  }

  virtual const unsigned int getSizeInput() const override = 0;

  virtual const unsigned int getSizeOutput() const override = 0;

  virtual const unsigned int getSizeState() const
  {
    return state_.size();
  }

  virtual void display() const override
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for SS_Interface" CRESET << std::endl;
  }
};

using SS_Interface_Ptr = std::unique_ptr<SS_Interface>;

}  // namespace sun

#endif