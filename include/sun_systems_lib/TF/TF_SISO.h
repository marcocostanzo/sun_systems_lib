
/*
    TF_SISO Class Discrete Time Transfer Function SISO (Linear Filter)

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

#ifndef TF_SISO_LIB
#define TF_SISO_LIB

/*! \file TF_SISO.h
    \brief This class represents a generic Discrete Time Transfer Function System.
*/

#include "sun_systems_lib/SISO_System_Interface.h"

/*
         b0 + b1*z-1 + b2*z-2 + ... + bn*z-n
    yk = ------------------------------------ uk
         a0 + a1*z-1 + a2*z-2 + ... + am*z-m

    yk = ([b0 b1 b2 ... bn]/a0)*[uk uk-1 uk-2 ... uk-n]T - ([a1 a2 ... am]/a0)*[yk-1 yk-2 ... yk-m]T

    b_vec = [b0 b1 b2 ... bn]T / a0  -> size n+1

    a_vec = [a1 a2 ... am]T / a0 -> size m

    u_vec = [uk uk-1 uk-2 ... uk-n]T

    y_vec = [yk-1 yk-2 ... yk-m]^T

    yk = b_vecT * u_vec - a_vecT * y_vec

*/

namespace sun
{
//!  TF_SISO class: represents a generic Discrete Time Transfer Function System.
/*!
    This class is a generic Discrete Time Transfer Function System in the form:

    \verbatim

    Z transform:

            b0 + b1*z^-1 + b2*z^-2 + ... + bn*z^-n
    Y(z) = ---------------------------------------- U(z)
            a0 + a1*z^-1 + a2*z^-2 + ... + am*z^-m

    Discrete Time (time k,  ^T = transpose):

    y(k) = ([b0 b1 b2 ... bn]/a0)*[u(k) u(k-1) u(k-2) ... u(k-n)]^T - ([a1 a2 ... am]/a0)*[y(k-1) y(k-2) ... y(k-m)]^T

    Short formulation:

    b_vec = [b0 b1 b2 ... bn]^T / a0  -> size n+1

    a_vec = [a1 a2 ... am]^T / a0 -> size m

    u_vec = [u(k) u(k-1) u(k-2) ... u(k-n)]^T

    y_vec = [y(k-1) y(k-2) ... y(k-m)]^T

    yk = b_vec^T * u_vec - a_vec^T * y_vec

    \endverbatim

    It stores the internal system state

    \sa Linear_System_Interface, TF_INTEGRATOR, TF_FIRST_ORDER_FILTER, TF_MIMO
*/
class TF_SISO : public SISO_System_Interface
{
public:
  // STATIC

  //! INTERNAL simplify numerator
  /*!
    This function transforms the numerator vector [b0 b1 ... bn 0 0 ... 0]T in b_vec=[b0 b1 ... bn]T / a0
    \param num_coeff numerator coefficients
    \param den_coeff denominator coefficients
    \return simplified numerator coefficients
  */
  inline static TooN::Vector<> simplifyNumerator(const TooN::Vector<>& num_coeff, const TooN::Vector<>& den_coeff)
  {
    if (den_coeff[0] == 0)
    {
      throw std::domain_error("[TF_SISO::simplifyNumerator] Division by 0");
    }

    if (num_coeff.size() == 0)  // case void vector
    {
      return TooN::makeVector(0.0);
    }

    // find first nonzero coeff
    unsigned int effective_size = num_coeff.size();
    while (effective_size != 1)
    {
      if (num_coeff[effective_size - 1] != 0.0)
        break;
      effective_size--;
    }

    return num_coeff.slice(0, effective_size) / den_coeff[0];
  }

  //
  //! INTERNAL simplify and reduce the denominator
  /*!
    This function transforms the denominator [a0 a1 ... am 0 0 ... 0]T in a_vec=[a1 a2 ... am]T / a0
    \param den_coeff denominator coefficients
    \return simplified denominator coefficients
  */
  inline static TooN::Vector<> simplifyAndReduceDenominator(const TooN::Vector<>& den_coeff)
  {
    if (den_coeff[0] == 0)
    {
      throw std::domain_error("[TF_SISO::simplifyAndReduceDenominator] Division by 0");
    }

    // find first nonzero coeff
    unsigned int effective_size = den_coeff.size();
    while (effective_size != 1)
    {
      if (den_coeff[effective_size - 1] != 0.0)
        break;
      effective_size--;
    }

    if (effective_size == 1)
    {
      return TooN::Vector<>(0);  // return a void vector
    }

    return den_coeff.slice(1, effective_size - 1) / den_coeff[0];
  }

private:
protected:
  //! sampling time, this is just a here to be stored, not used
  double ts_;

  //! Numerator coefficients, size n+1
  TooN::Vector<> b_vec_;  // n+1
  //! Reduced Denominator coefficients, without a0, size m
  TooN::Vector<> a_vec_;  // m

  //! State vector, last inputs, size n+1
  TooN::Vector<> u_vec_;  // n+1
  //! State vector, last outputs, size m
  TooN::Vector<> y_vec_;  // m
  //! 1D vector, very last output, size 1
  TooN::Vector<> y_k_ = TooN::Zeros(1);  // last output

public:
  /*===============CONSTRUCTORS===================*/

  //! Contructor
  /*!
    Construct the discrete time transfer function

    \param num_coeff numerator coefficients [b0, ...., bn]
    \param den_coeff denominator coefficients [a0, ...., an]
    \param Ts Sampling time, it is just internally stored, default = NaN
  */
  TF_SISO(const TooN::Vector<>& num_coeff, const TooN::Vector<>& den_coeff, double Ts = NAN)
    : ts_(Ts)
    , b_vec_(simplifyNumerator(num_coeff, den_coeff))
    , a_vec_(simplifyAndReduceDenominator(den_coeff))
    , u_vec_(TooN::Zeros(b_vec_.size()))
    , y_vec_(TooN::Zeros(a_vec_.size()))
  {
    // if(getNumeratorOrder() > getDenominatorOrder())
    // {
    //     throw std::domain_error("[TF_SISO] getNumDim() > getDenDim(). Non Causal System");
    // }
  }

  //! Zero Contructor
  /*!
    Construct a ZERO discrete time transfer function

    \param Ts Sampling time, it is just internally stored, default = NaN
  */
  TF_SISO(double Ts = NAN) : TF_SISO(TooN::makeVector(0.0), TooN::makeVector(1.0), Ts)
  {
  }

  //! Copy Constructor
  TF_SISO(const TF_SISO& tf) = default;

  virtual ~TF_SISO() override = default;

  //! Clone the object
  virtual TF_SISO* clone() const override
  {
    return new TF_SISO(*this);
  }

  /*==============================================*/

  /*=============GETTER===========================*/

  //! Get the numerator order n
  inline virtual const unsigned int getNumeratorOrder() const
  {
    return (b_vec_.size() - 1);
  }

  //! Get the denominator order m
  inline virtual const unsigned int getDenominatorOrder() const
  {
    return (a_vec_.size());
  }

  //! Get the sampling time
  inline virtual double getTs() const
  {
    return ts_;
  }

  /*==============================================*/

  /*=============SETTER===========================*/

  //! Set the sampling time
  inline virtual void setTs(double Ts)
  {
    ts_ = Ts;
  }

  /*==============================================*/

  /*=============RUNNER===========================*/

  inline virtual const TooN::Vector<>& apply(const TooN::Vector<>& input) override
  {
    if (input.size() != 1)
    {
      throw std::domain_error("[TF_SISO::apply(Vector)] The input has to be scalar");
    }
    apply(input[0]);
    return y_k_;
  }

  inline virtual double apply(double u_k) override
  {
    // Shift u_vec
    const unsigned int no = u_vec_.size() - 1;
    if (no != 0)
    {
      TooN::Vector<> tmp_u_vec = u_vec_;
      u_vec_.slice(1, no) = tmp_u_vec.slice(0, no);
    }
    u_vec_[0] = u_k;

    // Shift y_vec
    const unsigned int deno = y_vec_.size();
    if (deno == 0)
    {
      y_k_[0] = b_vec_ * u_vec_;
      return y_k_[0];
    }
    if (deno != 1)
    {
      TooN::Vector<> tmp_y_vec = y_vec_;
      y_vec_.slice(1, deno - 1) = tmp_y_vec.slice(0, deno - 1);
    }
    y_vec_[0] = y_k_[0];

    y_k_[0] = (b_vec_ * u_vec_) - (a_vec_ * y_vec_);

    return y_k_[0];
  }

  /*==============================================*/

  /*=============VARIE===========================*/
  inline virtual void reset() override
  {
    u_vec_ = TooN::Zeros;
    if (y_vec_.size() != 0)
      y_vec_ = TooN::Zeros;
    y_k_ = TooN::Zeros;
  }

  virtual const unsigned int getSizeInput() const override
  {
    return 1;
  }

  virtual const unsigned int getSizeOutput() const override
  {
    return 1;
  }

  //! Get the last output
  virtual double getLastOutput() const
  {
    return y_k_[0];
  }

  virtual void display() const override
  {
    display_tf();
  }

  //! Display the transfer function on the std out
  virtual void display_tf() const
  {
    std::cout << "TF_SISO:" << std::endl
              << "   Num_Coeff= " << b_vec_ << std::endl
              << "   Den_Coeff= 1.0 " << a_vec_ << std::endl
              << "   State" << std::endl
              << "   u_vec= " << u_vec_ << std::endl
              << "   y_vec= " << y_vec_ << std::endl
              << "   y_k= " << y_k_ << std::endl;
  }

  /*==============================================*/
};

using TF_SISO_Ptr = std::unique_ptr<TF_SISO>;

}  // namespace sun

#endif
