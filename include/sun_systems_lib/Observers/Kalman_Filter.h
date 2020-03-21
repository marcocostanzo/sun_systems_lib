/*
    Kalman Filter Class

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

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/*! \file Luenberger_Observer.h
    \brief Implementation of a Luenberger Observer
*/

#include <sun_systems_lib/Observers/Observer_Interface.h>
#include "TooN/SVD.h"

namespace sun
{
//!  Kalman_Filter class: Implementation of Observer_Interface.
/*!
    This class is a Discrete Extended Kalman Filter, Observer, it is also a Discrete SS system

    \verbatim

    u(k) /----------------------------------\
    ---->|                                  |
    ---->|                                  |
    y(k) |                                  |  y_hat(k)
         |              EKF                 |---->
    W(k) |                                  |
    ---->|                                  |
    ---->|                                  |
    V(k) \----------------------------------/
           ^    |                  ^     |
   x_hat(k)|    |x_hat(k-1)  P(k|k)|     |P(k-1|k-1)
           \----/                  \-----/

    \endverbatim

    It stores the internal system state

    A Discrete Discrete Extended Kalman Filter is also a Discrete state space system (SS_Interface).
  
    \warning This class is not fully implemented, it can be used only as a Kalman_Filter, do NOT
    try to use it as Observer_Interface or SS_Interface because the inheritance is NOT fully implemented.
    
    \warning Please,  use ONLY the specific kf apply method: kf_apply, obs_apply.

    \warning Do not try to use the stateless *_fcn methods. Future implementations will handle this.

    https://en.wikipedia.org/wiki/Extended_Kalman_filter    

    \sa Luenberger_Observer, Observer_Interface, Discrete_System_Interface
*/
class Kalman_Filter : public Observer_Interface
{
private:
protected:
  ////TooN::Vector<> state_;
  ////TooN::Vector<> output_;

  //! Observed System
  SS_Interface_Ptr system_;
  //! Xovariance estimate
  TooN::Matrix<> P_;
  //! Covariance Noise Matrix (state transition function)
  TooN::Matrix<> W_;
  //! Covariance Noise Matrix (output function)
  TooN::Matrix<> V_;
  //! Identity matrix. Internal Use
  TooN::Matrix<> Identity_x_;

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
    \param system Observed system
    \param W Initial Covariance Noise Matrix (state transition function)
    \param V Initial Covariance Noise Matrix (output function)
  */
  Kalman_Filter(const SS_Interface& system, const TooN::Matrix<>& W, const TooN::Matrix<>& V)
    : Observer_Interface(system.getState(), system.getSizeOutput())
    , system_(system.clone())
    , W_(W)
    , V_(V)
    , P_(W)
    , Identity_x_(TooN::Identity(system.getSizeState()))
  {
  }

  //! Copy Constructor
  Kalman_Filter(const Kalman_Filter& ss)
    : Observer_Interface(ss), system_(ss.system_->clone()), P_(ss.P_), W_(ss.W_), V_(ss.V_), Identity_x_(ss.Identity_x_)
  {
  }

  virtual Kalman_Filter* clone() const override
  {
    return new Kalman_Filter(*this);
  }

  //! Destructor
  virtual ~Kalman_Filter() override = default;

  ////virtual const TooN::Vector<>& getState() const
  ////{
  ////    return state_;
  ////}

  ////virtual void setState(const TooN::Vector<>& state)
  ////{
  ////    state_ = state;
  ////}

  //! Set W Matrix
  inline virtual void setW(const TooN::Matrix<>& W)
  {
    W_ = W;
  }

  //! Set V Matrix
  inline virtual void setV(const TooN::Matrix<>& V)
  {
    V_ = V;
  }

  //! Apply the EKF, compute the estimated output and update the internal state
  /*!
    Specific appy function for the EKF, is is similar to obs_apply(), but takes as input
    also the matrices W_k and V_k

    It is equivalent to:
    \verbatim
      kf.setW(W_k);
      kf.setV(V_k);
      return obs_apply(u_k, y_k);
    \endverbatim

    This method updates the internal state

    \param u_k Observed System Input at the current step u(k)
    \param y_k Observed System Measure at the current step y(k)
    \param W_k New value for the W matrix
    \param V_k New value for the V matrix
    \return estimated system output y_hat(k)
  */
  inline virtual const TooN::Vector<>& kf_apply(const TooN::Vector<>& u_k, const TooN::Vector<>& y_k,
                                                const TooN::Matrix<>& W_k, const TooN::Matrix<>& V_k)
  {
    setW(W_k);
    setV(V_k);
    return obs_apply(u_k, y_k);
  }

  inline virtual const TooN::Vector<>& obs_apply(const TooN::Vector<>& u_k, const TooN::Vector<>& y_k) override
  {

// TODO: review symbolism https://en.wikipedia.org/wiki/Extended_Kalman_filter

#define u_k1 u_k

#define x_hat_k_k state_
#define x_hat_k1_k1 state_

#define P_k_k P_
#define P_k1_k1 P_

#define W_k1 W_
#define V_k V_

#define y_hat_k_k output_

    // x_hat_k1_k1 = x_hat_k_k;
    // P_k1_k1 = P_k_k;

    /*PREDICT*/
    // predict state estimate
    TooN::Vector<> x_hat_k_k1 = system_->state_fcn(x_hat_k1_k1, u_k1);

    // Predicted covariance estimate
    TooN::Matrix<> F_k1 = system_->jacob_state_fcn(x_hat_k1_k1, u_k1);
    TooN::Matrix<> P_k_k1 = F_k1 * P_k1_k1 * (F_k1.T()) + W_k1;

    /*UPDATE*/
    // Innovation or measurement residual
    TooN::Vector<> y_hat_k_k1 = system_->output_fcn(x_hat_k_k1, u_k1);
    TooN::Vector<> y_tilde_k = y_k - y_hat_k_k1;

    // Innovation (or residual) covariance
    TooN::Matrix<> H_k = system_->jacob_output_fcn(x_hat_k_k1, u_k1);
    TooN::Matrix<> S_k = H_k * P_k_k1 * (H_k.T()) + V_k;

    // Near-optimal Kalman gain
    TooN::SVD<> S_k_SVD(S_k);
    TooN::Matrix<> K_k = P_k_k1 * (H_k.T()) * S_k_SVD.get_pinv();

    // Update state estimate
    x_hat_k_k = x_hat_k_k1 + K_k * y_tilde_k;

    // Update covariance estimate
    P_k_k = (Identity_x_ - K_k * H_k) * P_k_k1;

    // Update Output
    y_hat_k_k = system_->output_fcn(x_hat_k_k, u_k1);

    return y_hat_k_k;

#undef x_hat_k_k
#undef x_hat_k1_k1
#undef P_k_k
#undef P_k1_k1
#undef W_k1
#undef V_k
  }

  //! DO NOT USE THIS FUNCTION FOR Kalman_Filter
  /*!
    NOT IMPLEMENTED
  */
  virtual const TooN::Vector<> obs_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k,
                                             const TooN::Vector<>& y_k) const override
  {
    throw std::runtime_error("[Kalman_Filter]::state_fcn not implemented");
  }

  // virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
  //{
  //    const unsigned int size_real_input = getSizeRealInput();
  //    return state_fcn(   x_k_1,
  //                        u_k.slice(0, size_real_input),
  //                        u_k.slice(size_real_input, getSizeOutput())
  //                        );
  //}

  //! DO NOT USE THIS FUNCTION FOR Kalman_Filter
  /*!
    NOT IMPLEMENTED
  */
  virtual const TooN::Vector<> obs_output_fcn(const TooN::Vector<>& x_k, const TooN::Vector<>& u_k) const override
  {
    throw std::runtime_error("[Kalman_Filter]::output_fcn not implemented");
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

  //! DO NOT USE THIS FUNCTION FOR Kalman_Filter
  /*!
    NOT IMPLEMENTED
  */
  virtual const TooN::Matrix<> obs_jacob_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k,
                                                   const TooN::Vector<>& y_k) const override
  {
    throw std::runtime_error("[Kalman_Filter]::jacob_state_fcn not implemented");
  }

  // virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const
  // override
  //{
  //    const unsigned int size_real_input = getSizeRealInput();
  //    return obs_jacob_state_fcn( x_k_1,
  //                            u_k.slice(0, size_real_input),
  //                            u_k.slice(size_real_input, getSizeOutput())
  //                            );
  //}

  //! DO NOT USE THIS FUNCTION FOR Kalman_Filter
  /*!
    NOT IMPLEMENTED
  */
  virtual const TooN::Matrix<> obs_jacob_output_fcn(const TooN::Vector<>& x_k, const TooN::Vector<>& u_k) const override
  {
    throw std::runtime_error("[Kalman_Filter]::jacob_output_fcn not implemented");
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

  // virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
  //{
  //    state_ = state_fcn( state_, input );
  //    output_ = output_fcn( state_, input );
  //    return output_;
  //}

  virtual void reset() override
  {
    Observer_Interface::reset();
    state_ = TooN::Zeros;
    output_ = TooN::Zeros;
    system_->reset();
    P_ = W_;
  }

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
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Kalman_Filter" CRESET << std::endl;
  }
};

using Kalman_Filter_Ptr = std::unique_ptr<Kalman_Filter>;

}  // namespace sun

#endif