/*
    State Luenberger_Observer Class

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

#ifndef LUENBERGER_OBSERVER_H
#define LUENBERGER_OBSERVER_H

/*! \file Luenberger_Observer.h
    \brief Implementation of a Luenberger Observer
*/

#include <sun_systems_lib/Observers/Observer_Interface.h>

namespace sun
{
//!  Luenberger_Observer class: Implementation of Observer_Interface.
/*!
    This class is a Discrete Luemberger Observer, it is also a Discrete SS system

    \verbatim

    u(k) /-------\  y_hat(k)
    ---->|  Sys  |---->
    ---->|       |
    y(k) \-------/
           ^    |
   x_hat(k)|    |x_hat(k-1)
           \----/

    x_hat(k) = f_obs(x_hat(k-1),u(k),y(k)) = f(x_hat(k-1),u(k)) + L * (y(k) - h(x_hat(k-1),u(k)))
    y_hat(k) = h(x_hat(k),u(k))

    \endverbatim

    It stores the internal system state

    A Discrete Luenberger Observer is also a Discrete state space system (SS_Interface).
    So, you can use the same interface. The only difference is the input that has to be redefined as:

    \verbatim
    input = [observed_system_input; observed_system_output]
    \endverbatim

    This is because the observer takes both the observed_system_input and observed_system_output as input.

    In any case, you can use the specific observer methods: obs_state_fcn, obs_output_fcn, ... ,
    that take as input the observed_system_input and observed_system_output separately.

    \sa Linear_System_Interface, Kalman_Filter, Observer_Interface, Discrete_System_Interface
*/
class Luenberger_Observer : public Observer_Interface
{
private:
protected:
  ////TooN::Vector<> state_;
  ////TooN::Vector<> output_;

  //! Observed system
  SS_Interface_Ptr system_;

  //! Observer Gain Matrix
  TooN::Matrix<> L_;

  ////SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
  ////            :state_(state),
  ////            output_(output)
  ////            {}

  // Observer_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
  //            :SS_Interface( state, output )
  //            {}

public:

  //! Constructor
  /*!
    Build the observer
    \param system the observed system
    \param L The observer gain matrix
  */
  Luenberger_Observer(const SS_Interface& system, const TooN::Matrix<>& L)
    : Observer_Interface(system.getState(), system.getSizeOutput()), system_(system.clone()), L_(L)
  {
    if (!chek_dimensions())
    {
      throw std::invalid_argument("[Luenberger_Observer] Invalid matrix dimensions");
    }
  }

  //! Copy Constructor
  Luenberger_Observer(const Luenberger_Observer& ss) : Observer_Interface(ss), system_(ss.system_->clone()), L_(ss.L_)
  {
  }

  virtual Luenberger_Observer* clone() const override
  {
    return new Luenberger_Observer(*this);
  }

  //! Destructor
  virtual ~Luenberger_Observer() override = default;

  ////virtual const TooN::Vector<>& getState() const
  ////{
  ////    return state_;
  ////}

  ////virtual void setState(const TooN::Vector<>& state)
  ////{
  ////    state_ = state;
  ////}

  inline virtual const TooN::Vector<> obs_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k,
                                                    const TooN::Vector<>& y_k) const override
  {
    return system_->state_fcn(x_k_1, u_k) + L_ * (y_k - system_->output_fcn(x_k_1, u_k));
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

  inline virtual const TooN::Vector<> obs_output_fcn(const TooN::Vector<>& x_k,
                                                     const TooN::Vector<>& u_k) const override
  {
    return system_->output_fcn(x_k, u_k);
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

  inline virtual const TooN::Matrix<> obs_jacob_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k,
                                                          const TooN::Vector<>& y_k) const override
  {
    return system_->jacob_state_fcn(x_k_1, u_k) - L_ * system_->jacob_output_fcn(x_k_1, u_k);
  }

  // inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const
  // override
  //{
  //    return obs_jacob_state_fcn( x_k_1,
  //                            u_k.slice(0,system_->getSizeInput()),
  //                            u_k.slice(system_->getSizeInput(),system_->getSizeOutput())
  //                            );
  //}

  inline virtual const TooN::Matrix<> obs_jacob_output_fcn(const TooN::Vector<>& x_k,
                                                           const TooN::Vector<>& u_k) const override
  {
    return system_->jacob_output_fcn(x_k, u_k);
  }

  // virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const
  // override
  //{
  //    const unsigned int size_real_input = getSizeRealInput();
  //    return obs_jacob_output_fcn(   x_k_1,
  //                        u_k.slice(0, size_real_input),
  //                        u_k.slice(size_real_input, getSizeOutput())
  //                        );
  //}

  // inline virtual const TooN::Vector<>& obs_apply( const TooN::Vector<>& input, const TooN::Vector<>& measure )
  //{
  //    state_ = obs_state_fcn( state_, input, measure );
  //    output_ = obs_output_fcn( state_, input );
  //    return output_;
  //}

  ////inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
  ////{
  ////    state_ = state_fcn( state_, input );
  ////    output_ = output_fcn( state_, input );
  ////    return output_;
  ////}

  ////virtual void reset() override
  ////{
  ////    state_ = TooN::Zeros;
  ////    output_ = TooN::Zeros;
  ////}

  // virtual const unsigned int getSizeRealInput() const
  //{
  //    return getSizeInput() - getSizeOutput();
  //}

  virtual const unsigned int getSizeInput() const override
  {
    return system_->getSizeInput() + system_->getSizeOutput();
  }

  virtual const unsigned int getSizeOutput() const override
  {
    return system_->getSizeOutput();
  }

  ////virtual const unsigned int getSizeState() const
  ////{
  ////    return state_.size();
  ////}

  virtual void display() const override
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Luenberger_Observer" CRESET << std::endl;
  }

  virtual bool chek_dimensions() const
  {
    if (state_.size() != system_->getSizeState())
      return false;
    if (output_.size() != system_->getSizeOutput())
      return false;
    if (L_.num_rows() != system_->getSizeState())
      return false;
    if (L_.num_cols() != system_->getSizeOutput())
      return false;

    return true;
  }
};

using Luenberger_Observer_Ptr = std::unique_ptr<Luenberger_Observer>;

}  // namespace sun

#endif