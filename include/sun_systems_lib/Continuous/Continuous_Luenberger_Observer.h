/*
    Continuous Time State Luenberger Observer System Class

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

#ifndef CONTINUOUS_LUENBERGER_OBSERVER_H
#define CONTINUOUS_LUENBERGER_OBSERVER_H

/*! \file Continuous_Luenberger_Observer.h
    \brief This class represents a continuous Luemberger Observer
*/

#include <sun_systems_lib/Continuous/Continuous_Observer_Interface.h>

namespace sun
{
//!  Continuous_Luenberger_Observer class: represents a continuous Luemberger Observer.
/*!
    This class takes a Continuous State Space System (Continuous_System_Interface), and a gain Matrix L.

    Then represents a continuous Luemberger observer in the form:

    x_hat_dot = f(x_hat,u) + L*(y - h(x_hat,u))
    
    y_hat = h(x_hat,u)

    Note: This is a Continuous system, in order to simulate it you have to discretize it.

    NB. see Continuous_Observer_Interface to understand the relations between the Continuous_Observer_Interface and
    Continuous_System_Interface classes. An observer is a continuous state space system, but the input is both the
    observed system input and the observed system output, in order to use the observer as Continuous_System_Interface
    you have to reorder the input as:

    observer_input = [system_input; system_output]

    In any case, you can just use the Continuous_Observer_Interface methods and forget that the observer is a state
    space system: obs_state_fcn, obs_output_fcn ...

    \sa Continuous_Observer_Interface, Continuous_System_Interface, Discretizator_Interface, RK4
*/
class Continuous_Luenberger_Observer : public Continuous_Observer_Interface
{
private:
protected:
  //!  Observed System.
  Continuous_System_Interface_Ptr system_;

  //!  Gain Matrix.
  TooN::Matrix<> L_;

public:
  //!  Full Constructor
  /*!
      Costructor that takes the observed system system and the gain Matrix L
      \param system The observed System
      \param L The observer gain Matrix
  */
  Continuous_Luenberger_Observer(const Continuous_System_Interface& system, const TooN::Matrix<>& L)
    : system_(system.clone()), L_(L)
  {
    if (!chek_dimensions())
    {
      throw std::invalid_argument("[Continuous_Luenberger_Observer] Invalid matrix dimensions");
    }
  }

  //! A copy constructor
  Continuous_Luenberger_Observer(const Continuous_Luenberger_Observer& ss) : system_(ss.system_->clone()), L_(ss.L_)
  {
  }

  //! Clone the object
  virtual Continuous_Luenberger_Observer* clone() const override
  {
    return new Continuous_Luenberger_Observer(*this);
  }

  //! A destructor
  virtual ~Continuous_Luenberger_Observer() override = default;

  //!  Observer State function x_hat_dot = f_obs(x_hat,u,y)
  /*!
      returns the estimated state derivative x_hat_dot as

      x_hat_dot = f(x_hat,u) + L*(y - h(x_hat,u))

      where f and h are the state and output function of the observed system respectively
      \param x_hat The estimated state
      \param u The observed system input
      \param y The observed system measure
      \return The estimated state derivative x_hat_dot
  */
  inline virtual const TooN::Vector<> obs_state_fcn(const TooN::Vector<>& x_hat, const TooN::Vector<>& u,
                                                    const TooN::Vector<>& y) const override
  {
    return system_->state_fcn(x_hat, u) + L_ * (y - system_->output_fcn(x_hat, u));
  }

  // inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const
  // override
  //{
  //    const unsigned int real_input = getSizeRealInput();
  //    return obs_state_fcn(   x_k_1,
  //                        u_k.slice(0, real_input ),
  //                        u_k.slice(real_input, getSizeOutput())
  //                        );
  //}

  //!  Observer Output function y_hat = h_obs(x_hat,u)
  /*!
      returns the estimated output as

      y_hat = h(x_hat,u)

      where h is the output function of the observed system
      \param x_hat The estimated state
      \param u The observed system input
      \return The estimated output y_hat
  */
  inline virtual const TooN::Vector<> obs_output_fcn(const TooN::Vector<>& x_hat,
                                                     const TooN::Vector<>& u) const override
  {
    return system_->output_fcn(x_hat, u);
  }

  // inline virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const
  // override
  //{
  //    const unsigned int size_real_input = getSizeRealInput();
  //    return obs_output_fcn(   x_k_1,
  //                        u_k.slice(0, size_real_input),
  //                        u_k.slice(size_real_input, getSizeOutput())
  //                        );
  //}

  //!  Observer state function Jacobian F_obs = jac_state_obs(x_hat,u,y)
  /*!
      returns the state function Jacobian of the observer F_obs as

      F_obs = jac_state(x_hat,u) - L*jac_output(x_hat,u)

      where jac_state and jac_output are the state and output function jacobians of the observed system respectively
      \param x_hat The estimated state
      \param u The observed system input
      \param y The observed system measure
      \return The Observer state function Jacobian F_obs
  */
  inline virtual const TooN::Matrix<> obs_jacob_state_fcn(const TooN::Vector<>& x_hat, const TooN::Vector<>& u,
                                                          const TooN::Vector<>& y) const override
  {
    return system_->jacob_state_fcn(x_hat, u) - L_ * system_->jacob_output_fcn(x_hat, u);
  }

  // inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const
  // override
  //{
  //    return obs_jacob_state_fcn( x_k_1,
  //                            u_k.slice(0,system_->getSizeInput()),
  //                            u_k.slice(system_->getSizeInput(),system_->getSizeOutput())
  //                            );
  //}

  //!  Observer Output function Jacobian H_obs = jac_output_obs(x_hat,u)
  /*!
      returns the output function Jacobian of the observer H_obs as

      H_obs = jac_output(x_hat,u)

      where jac_output is the output function jacobian of the observed system
      Note: observer output function jacobian is the same as the observed system output function jacobian
      \param x_hat The estimated state
      \param u The observed system input
      \return The Observer output function Jacobian H_obs
  */
  inline virtual const TooN::Matrix<> obs_jacob_output_fcn(const TooN::Vector<>& x_hat,
                                                           const TooN::Vector<>& u) const override
  {
    return system_->jacob_output_fcn(x_hat, u);
  }

  //! The size of the real input u
  /*!
      \return the size of the observed system input
  */
  inline virtual const unsigned int getSizeRealInput() const override
  {
    return system_->getSizeInput();
  }

  //! The input size of the equivalent state space system
  /*!
      The observer is also a state space system.
      The input of the equivalent state space system is organized as [ observed_system_input; observed_system_output ].
      The function returns the size of this equivalent input
  */
  inline virtual const unsigned int getSizeInput() const override
  {
    return system_->getSizeInput() + system_->getSizeOutput();
  }

  //! The output size of the observer
  /*!
      It is the same of the observed system output size
  */
  inline virtual const unsigned int getSizeOutput() const override
  {
    return system_->getSizeOutput();
  }

  //! The state size of the observer
  /*!
      It is the same of the observed system state size
  */
  inline virtual const unsigned int getSizeState() const
  {
    return system_->getSizeState();
  }

  //! Display the observer
  /*!
      Display the observer on the standard output
  */
  virtual void display() const
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Continuous_Luenberger_Observer" CRESET << std::endl;
  }

  //! [Internal function]
  /*!
      Check the dimension of the gain matrix
  */
  virtual bool chek_dimensions() const
  {
    if (L_.num_rows() != system_->getSizeState())
      return false;
    if (L_.num_cols() != system_->getSizeOutput())
      return false;

    return true;
  }
};

using Continuous_Luenberger_Observer_Ptr = std::unique_ptr<Continuous_Luenberger_Observer>;

}  // namespace sun

#endif