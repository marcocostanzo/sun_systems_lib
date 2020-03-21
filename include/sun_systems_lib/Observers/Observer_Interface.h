/*
    State Observer Interface Class

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

#ifndef OBSERVER_INTERFACE_H
#define OBSERVER_INTERFACE_H

/*! \file Observer_Interface.h
    \brief This interface represents a generic Discrete Observer
*/

#include <sun_systems_lib/SS/SS_Interface.h>

namespace sun
{
//!  Observer_Interface class: represents a generic Discrete Observer.
/*!
    This class is a generic discrete time Observer, it is also a SS system

    \verbatim

    u(k) /-------\  y_hat(k)
    ---->|  Sys  |---->
    ---->|       |
    y(k) \-------/
           ^    |
   x_hat(k)|    |x_hat(k-1)
           \----/

    x_hat(k) = f_obs(x_hat(k-1),u(k),y(k))
    y_hat(k) = h(x_hat(k),u(k))

    \endverbatim

    It stores the internal system state

    A Discrete observer is also a Discrete state space system (SS_Interface).
    So, you can use the same interface. The only difference is the input that has to be redefined as:

    \verbatim
    input = [observed_system_input; observed_system_output]
    \endverbatim

    This is because the observer takes both the observed_system_input and observed_system_output as input.

    In any case, you can use the specific observer methods: obs_state_fcn, obs_output_fcn, ... ,
    that take as input the observed_system_input and observed_system_output separately.

    \sa Kalman_Filter, Luemberger_Observer, Discrete_System_Interface
*/
class Observer_Interface : public SS_Interface
{
private:
protected:
  // TooN::Vector<> state_;
  // TooN::Vector<> output_;

  // SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
  //            :state_(state),
  //            output_(output)
  //            {}

  //!  Constructor
  /*!
      Costructor that simply calls SS_Interface constructor
  */
  Observer_Interface(const TooN::Vector<>& state, unsigned int dim_output) : SS_Interface(state, dim_output)
  {
  }

public:
  virtual Observer_Interface* clone() const override = 0;

  //! Desctructor
  virtual ~Observer_Interface() override = default;

  // virtual const TooN::Vector<>& getState() const
  //{
  //    return state_;
  //}

  // virtual void setState(const TooN::Vector<>& state)
  //{
  //    state_ = state;
  //}

  //!  Observer State function x_hat(k) = f_obs(x_hat(k-1),u(k),y(k))
  /*!
      returns the estimated state x_hat(k)

      This method is stateless, i.e., it does NOT update the internal state, use obs_apply instead
      \param x_hat_k_1 The estimated state at previous step x(k-1)
      \param u The observed system input u(k)
      \param y The observed system measure y(k)
      \return The estimated state x(k)
  */
  virtual const TooN::Vector<> obs_state_fcn(const TooN::Vector<>& x_hat_k_1, const TooN::Vector<>& u_k,
                                             const TooN::Vector<>& y_k) const = 0;

  //!  Observer as StateSpaceSystem - State function x(k) = f(x(k-1),u_compleate(k))
  /*!
      returns the estimated state derivative x_hat_dot
      This method is to be coherent with the SS_Interface.
      In this method the input u_k must be [observed_system_input_k; observed_system_output_k]

      This method is stateless, i.e., it does NOT update the internal state, use apply instead
      \param x_hat_k_1 The estimated state
      \param u_compleate_k The compleate system input [observed_system_input; observed_system_output]
      \return The estimated state derivative x_hat_dot
      \sa obs_state_fcn
  */
  inline virtual const TooN::Vector<> state_fcn(const TooN::Vector<>& x_hat_k_1,
                                                const TooN::Vector<>& u_compleate_k) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_state_fcn(x_hat_k_1, u_compleate_k.slice(0, size_real_input),
                         u_compleate_k.slice(size_real_input, getSizeOutput()));
  }

  //!  Observer Output function y_hat_k = h_obs(x_hat(k),u(k))
  /*!
      returns the estimated output

      \param x_hat_k The estimated state
      \param u_k The observed system input
      \return The estimated output y_hat(k)
  */
  virtual const TooN::Vector<> obs_output_fcn(const TooN::Vector<>& x_hat_k, const TooN::Vector<>& u_k) const = 0;

  //!  Observer as StateSpaceSystem - Output function y(k) = h(x_hat(k),u_compleate(k))
  /*!
      returns the estimated output y_hat
      This method is to be coherent with the SS_Interface.
      In this method the input u_k must be [observed_system_input_k; observed_system_output_k]
      \param x_hat_k_1 The estimated state
      \param u_compleate_k The compleate system input [observed_system_input; observed_system_output]
      \return The estimated output y_hat(k)
  */
  inline virtual const TooN::Vector<> output_fcn(const TooN::Vector<>& x_hat_k_1,
                                                 const TooN::Vector<>& u_k) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_output_fcn(x_hat_k_1, u_k.slice(0, size_real_input));
  }

  //!  Observer state function Jacobian F_obs(k) = jac_state_obs(x_hat(k-1),u(k),y(k))
  /*!
      returns the Jacobian F_obs of the observer state function f_obs

      \param x_hat_k_1 The estimated state
      \param u_k The observed system input
      \param y_k The observed system measure
      \return The Observer state function Jacobian F_obs
  */
  virtual const TooN::Matrix<> obs_jacob_state_fcn(const TooN::Vector<>& x_hat_k_1, const TooN::Vector<>& u_k,
                                                   const TooN::Vector<>& y_k) const = 0;

  //!  Observer as StateSpaceSystem - state function Jacobian F_obs(k) = jac_state(x_hat(k-1),u_compleate(k))
  /*!
      returns the Jacobian F_obs of the observer state function f_obs
      This method is to be coherent with the SS_Interface.
      In this method the input u_k must be [observed_system_input_k; observed_system_output_k]
      \param x_hat_k_1 The estimated state
      \param u_compleate_k The compleate system input [observed_system_input; observed_system_output]
      \return The Observer state function Jacobian F_obs
  */
  virtual const TooN::Matrix<> jacob_state_fcn(const TooN::Vector<>& x_hat_k_1,
                                               const TooN::Vector<>& u_compleate_k) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_jacob_state_fcn(x_hat_k_1, u_compleate_k.slice(0, size_real_input),
                               u_compleate_k.slice(size_real_input, getSizeOutput()));
  }

  //!  Observer Output function Jacobian H_obs(k) = jac_output_obs(x_hat(k),u(k))
  /*!
      returns the output function Jacobian of the observer H_obs

      \param x_hat_k The estimated state
      \param u_k The observed system input
      \return The Observer output function Jacobian H_obs
  */
  virtual const TooN::Matrix<> obs_jacob_output_fcn(const TooN::Vector<>& x_hat_k, const TooN::Vector<>& u_k) const = 0;

  //!  Observer as StateSpaceSystem - output function Jacobian H_obs(k) = jac_output(x_hat(k),u_compleate(k))
  /*!
      returns the output function Jacobian H_obs
      This method is to be coherent with the Continuous_System_Interface.
      In this method the input u_k must be [observed_system_input_k; observed_system_output_k]
      \param x_hat_k The estimated state
      \param u_compleate_k The compleate system input [observed_system_input; observed_system_output]
      \return The output function Jacobian H_obs
  */
  virtual const TooN::Matrix<> jacob_output_fcn(const TooN::Vector<>& x_hat_k,
                                                const TooN::Vector<>& u_compleate_k) const override
  {
    const unsigned int size_real_input = getSizeRealInput();
    return obs_jacob_output_fcn(x_hat_k, u_compleate_k.slice(0, size_real_input));
  }

  //! Apply the observer, compute the estimated output and update the internal state
  /*!
    Go one discrete step ahead, apply the input u(k) and the measure y(k), update the internal state for the next step,
    return the estimated output y_hat(k)

    \verbatim
      x_hat(k) = f_obs(x_hat(k-1),u(k),y(k))
      y_hat(k) = h_obs(x(k),u(k))
    \endverbatim

    \param u_k Observed System Input at the current step u(k)
    \param y_k Observed System Measure at the current step y(k)
    \return estimated system output y_hat(k)
  */
  virtual const TooN::Vector<>& obs_apply(const TooN::Vector<>& u_k, const TooN::Vector<>& y_k)
  {
    state_ = obs_state_fcn(state_, u_k, y_k);
    output_ = obs_output_fcn(state_, u_k);
    return output_;
  }

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

  //! The size of the real input u
  /*!
      \return the size of the observed system input
  */
  virtual const unsigned int getSizeRealInput() const
  {
    return getSizeInput() - getSizeOutput();
  }

  virtual const unsigned int getSizeInput() const override = 0;

  virtual const unsigned int getSizeOutput() const override = 0;

  // virtual const unsigned int getSizeState() const
  //{
  //    return state_.size();
  //}

  virtual void display() const override
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Observer_Interface" CRESET << std::endl;
  }

  //! [Internal] Build the full input
  /*!
      Build the full input for the equivalent state space system [observed_system_input; observed_system_output]
  */
  inline virtual TooN::Vector<> buildFullInput(const TooN::Vector<>& u_k, const TooN::Vector<>& y_k) const
  {
    TooN::Vector<> full_input = TooN::Zeros(getSizeInput());
    const unsigned int size_real_input = getSizeRealInput();
    full_input.slice(0, size_real_input) = u_k;
    full_input.slice(size_real_input, getSizeOutput()) = y_k;
    return full_input;
  }
};

using Observer_Interface_Ptr = std::unique_ptr<Observer_Interface>;

}  // namespace sun

#endif