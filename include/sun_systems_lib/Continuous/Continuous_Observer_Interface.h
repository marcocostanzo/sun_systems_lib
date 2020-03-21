/*
    Continuous Time State Observer System Interface Class

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

#ifndef CONTINUOUS_OBSERVER_INTERFACE_H
#define CONTINUOUS_OBSERVER_INTERFACE_H

/*! \file Continuous_Observer_Interface.h
    \brief This Interface class represents a generic continuous Observer.
*/

#include <sun_systems_lib/Continuous/Continuous_System_Interface.h>

namespace sun
{
//!  Continuous_Observer_Interface class: represents a generic continuous Observer.
/*!
    This class is a generic continuous observer interface.

    A continuous observer is also a continuous state space system (Continuous_System_Interface).
    So, you can use the same interface. The only difference is the input that has to be redefined as:

    input = [observed_system_input; observed_system_output]

    This is because the observer takes both the observed_system_input and observed_system_output as input.

    In any case, you can use the specific observer methods: obs_state_fcn, obs_output_fcn, ... ,
    that take as input the observed_system_input and observed_system_output separately.

    Note: This is a Continuous system, in order to simulate it you have to discretize it.

    \sa Continuous_Luemberger_Observer, Continuous_System_Interface, Discretizator_Interface, RK4
*/
class Continuous_Observer_Interface : public Continuous_System_Interface
{
private:
protected:
public:
  //! Clone the object
  virtual Continuous_Observer_Interface* clone() const = 0;

  //! A destructor
  virtual ~Continuous_Observer_Interface() = default;

  //!  Observer State function x_hat_dot = f_obs(x_hat,u,y)
  /*!
      returns the estimated state derivative x_hat_dot
      \param x_hat The estimated state
      \param u The observed system input
      \param y The observed system measure
      \return The estimated state derivative x_hat_dot
  */
  virtual const TooN::Vector<> obs_state_fcn(const TooN::Vector<>& x_hat, const TooN::Vector<>& u,
                                             const TooN::Vector<>& y) const = 0;

  //!  Observer as StateSpaceSystem - State function x_hat_dot = f(x_hat,u_compleate)
  /*!
      returns the estimated state derivative x_hat_dot
      This method is to be coherent with the Continuous_System_Interface.
      In this method the input u must be [observed_system_input; observed_system_output]
      \param x_hat The estimated state
      \param u_compleate The compleate system input [observed_system_input; observed_system_output]
      \return The estimated state derivative x_hat_dot
  */
  inline virtual const TooN::Vector<> state_fcn(const TooN::Vector<>& x_hat,
                                                const TooN::Vector<>& u_compleate) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_state_fcn(x_hat, u_compleate.slice(0, size_real_input),
                         u_compleate.slice(size_real_input, getSizeOutput()));
  }

  //!  Observer Output function y_hat = h_obs(x_hat,u)
  /*!
      returns the estimated output

      \param x_hat The estimated state
      \param u The observed system input
      \return The estimated output y_hat
  */
  virtual const TooN::Vector<> obs_output_fcn(const TooN::Vector<>& x_hat, const TooN::Vector<>& u) const = 0;

  //!  Observer as StateSpaceSystem - Output function y_hat = h(x_hat,u_compleate)
  /*!
      returns the estimated output y_hat
      This method is to be coherent with the Continuous_System_Interface.
      In this method the input u must be [observed_system_input; observed_system_output]
      \param x_hat The estimated state
      \param u_compleate The compleate system input [observed_system_input; observed_system_output]
      \return The estimated output y_hat
  */
  inline virtual const TooN::Vector<> output_fcn(const TooN::Vector<>& x_hat,
                                                 const TooN::Vector<>& u_compleate) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_output_fcn(x_hat, u_compleate.slice(0, size_real_input));
  }

  //!  Observer state function Jacobian F_obs = jac_state_obs(x_hat,u,y)
  /*!
      returns the Jacobian F_obs of the observer state function f_obs

      \param x_hat The estimated state
      \param u The observed system input
      \param y The observed system measure
      \return The Observer state function Jacobian F_obs
  */
  virtual const TooN::Matrix<> obs_jacob_state_fcn(const TooN::Vector<>& x_hat, const TooN::Vector<>& u,
                                                   const TooN::Vector<>& y) const = 0;

  //!  Observer as StateSpaceSystem - state function Jacobian F_obs = jac_state(x_hat,u_compleate)
  /*!
      returns the Jacobian F_obs of the observer state function f_obs
      This method is to be coherent with the Continuous_System_Interface.
      In this method the input u must be [observed_system_input; observed_system_output]
      \param x_hat The estimated state
      \param u_compleate The compleate system input [observed_system_input; observed_system_output]
      \return The Observer state function Jacobian F_obs
  */
  virtual const TooN::Matrix<> jacob_state_fcn(const TooN::Vector<>& x_hat,
                                               const TooN::Vector<>& u_compleate) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_jacob_state_fcn(x_hat, u_compleate.slice(0, size_real_input),
                               u_compleate.slice(size_real_input, getSizeOutput()));
  }

  //!  Observer Output function Jacobian H_obs = jac_output_obs(x_hat,u)
  /*!
      returns the output function Jacobian of the observer H_obs

      \param x_hat The estimated state
      \param u The observed system input
      \return The Observer output function Jacobian H_obs
  */
  virtual const TooN::Matrix<> obs_jacob_output_fcn(const TooN::Vector<>& x_hat, const TooN::Vector<>& u) const = 0;

  //!  Observer as StateSpaceSystem - output function Jacobian H_obs = jac_output(x_hat,u_compleate)
  /*!
      returns the output function Jacobian H_obs
      This method is to be coherent with the Continuous_System_Interface.
      In this method the input u must be [observed_system_input; observed_system_output]
      \param x_hat The estimated state
      \param u_compleate The compleate system input [observed_system_input; observed_system_output]
      \return The output function Jacobian H_obs
  */
  virtual const TooN::Matrix<> jacob_output_fcn(const TooN::Vector<>& x_k, const TooN::Vector<>& u_k) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_jacob_output_fcn(x_k, u_k.slice(0, size_real_input));
  }

  //! The size of the real input u
  /*!
      \return the size of the observed system input
  */
  virtual const unsigned int getSizeRealInput() const
  {
    return getSizeInput() - getSizeOutput();
  }

  //! Observer as StateSpaceSystem - The input size of the equivalent state space system
  /*!
      The observer is also a state space system.
      The input of the equivalent state space system is organized as [ observed_system_input; observed_system_output ].
      The function returns the size of this equivalent input
  */
  virtual const unsigned int getSizeInput() const override = 0;

  //! The output size of the observer
  /*!
      It is the same of the observed system output size
  */
  virtual const unsigned int getSizeOutput() const override = 0;

  //! The state size of the observer
  /*!
      It is the same of the observed system state size
  */
  virtual const unsigned int getSizeState() const = 0;

  //! Display the observer
  /*!
      Display the observer on the standard output
  */
  virtual void display() const
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Continuous_Observer_Interface" CRESET << std::endl;
  }
};

using Continuous_Observer_Interface_Ptr = std::unique_ptr<Continuous_Observer_Interface>;

}  // namespace sun

#endif