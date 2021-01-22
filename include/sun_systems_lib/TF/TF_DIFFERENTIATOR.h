
/*
    TF_DIFFERENTIATOR Class

    DIFFERENTIATOR transfer function generic

    Copyright 2021 Università della Campania Luigi Vanvitelli

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

#ifndef TF_DIFFERENTIATOR_H
#define TF_DIFFERENTIATOR_H

/*! \file TF_DIFFERENTIATOR.h
    \brief This class represents a differentiator as Discrete Time Transfer Function System.
*/

#include "sun_systems_lib/TF/TF_SISO.h"

namespace sun
{
//!  TF_DIFFERENTIATOR class: represents a differentiator as Discrete Time Transfer Function System.
/*!
    This class is an differentiator as Discrete Time Transfer Function System.

    It is an abstract class.

    \sa Linear_System_Interface, TF_DIFFERENTIATOR_2POLES, TF_INTEGRATOR
*/
class TF_DIFFERENTIATOR : public TF_SISO
{
private:
  TF_DIFFERENTIATOR();  // NO DEFAULT CONSTRUCTOR

protected:
  //! Differentiator gain
  double gain_;

  //! Constructor
  /*!
      Internal usage in implementation.
  */
  TF_DIFFERENTIATOR(double Ts, const TooN::Vector<>& num_coeff, const TooN::Vector<>& den_coeff, double gain = 1.0)
    : TF_SISO(num_coeff, den_coeff, Ts), gain_(gain)
  {
  }

public:
  /*===============CONSTRUCTORS===================*/

  //! Destructor
  virtual ~TF_DIFFERENTIATOR() override = default;

  virtual TF_DIFFERENTIATOR* clone() const override
  {
    return new TF_DIFFERENTIATOR(*this);
  }

  //! Copy Constructor
  TF_DIFFERENTIATOR(const TF_DIFFERENTIATOR& tf) = default;
  /*==============================================*/

  /*=============GETTER===========================*/
  /*==============================================*/

  /*=============SETTER===========================*/
  inline virtual void setTs(double Ts) override
  {
    throw "[TF_DIFFERENTIATOR::setTs] Cannot set Ts on TF_DIFFERENTIATOR";
  }

  /*==============================================*/

  /*=============RUNNER===========================*/
  inline virtual double apply(double uk) override
  {
    return TF_SISO::apply(gain_ * uk);
  }
  /*==============================================*/

  /*=============VARIE===========================*/

  virtual void display_tf() const override
  {
    std::cout << "TF_DIFFERENTIATOR:" << std::endl << "   Ts: " << ts_ << std::endl << "   gain: " << gain_ << std::endl;
    std::cout << "  TF_DIFFERENTIATOR(tf):" << std::endl << "    "; 
    TF_SISO::display_tf();
  }

  /*==============================================*/
};

/*=============STATIC FUNS===========================*/
/*==============================================*/

using TF_DIFFERENTIATOR_Ptr = std::unique_ptr<TF_DIFFERENTIATOR>;

}  // namespace sun

#endif
