
/*
    TF_INTEGTATOR Class

    Integrator transfer function using the trapez method

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

#ifndef TF_INTEGRATOR_H
#define TF_INTEGRATOR_H

/*! \file TF_INTEGRATOR.h
    \brief This class represents an integrator as Discrete Time Transfer Function System.
*/

#include "sun_systems_lib/TF/TF_SISO.h"

namespace sun
{
//!  TF_INTEGRATOR class: represents an integrator as Discrete Time Transfer Function System.
/*!
    This class is an integrator as Discrete Time Transfer Function System.

    It is discretized using the tustin method.

    \sa Linear_System_Interface, TF_FIRST_ORDER_FILTER, TF_SISO, TF_MIMO
*/
class TF_INTEGRATOR : public TF_SISO
{
private:
  TF_INTEGRATOR();  // NO DEFAULT CONSTRUCTOR

protected:
  //! Integrator gain
  double gain_;

public:
  /*===============CONSTRUCTORS===================*/

  //! Constructor
  /*!
      \param Ts sampling time
      \param gain gain of the integrator, default = 1
  */
  TF_INTEGRATOR(double Ts, double gain = 1.0)
    : TF_SISO((Ts / 2.0) * TooN::makeVector(1.0, 1.0), TooN::makeVector(1.0, -1.0), Ts), gain_(gain)
  {
  }

  //! Destructor
  virtual ~TF_INTEGRATOR() override = default;

  virtual TF_INTEGRATOR* clone() const override
  {
    return new TF_INTEGRATOR(*this);
  }

  //! Copy Constructor
  TF_INTEGRATOR(const TF_INTEGRATOR& tf) = default;
  /*==============================================*/

  /*=============GETTER===========================*/
  /*==============================================*/

  /*=============SETTER===========================*/
  inline virtual void setTs(double Ts) override
  {
    throw "[TF_INTEGRATOR::setTs] Cannot set Ts on TF_INTEGRATOR";
  }

  //! Change the state so that the output is "output"
  /*!
      \param output output after the state change
  */
  inline virtual void setOutput(double output)
  {
    u_vec_[0] = 0.0;
    u_vec_[1] = 0.0;
    y_vec_[0] = output;
    y_k_[0] = output;
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
    std::cout << "TF_INTEGRATOR:" << std::endl << "   Ts: " << ts_ << std::endl << "   gain: " << gain_ << std::endl;
  }

  /*==============================================*/
};

/*=============STATIC FUNS===========================*/
/*==============================================*/

using TF_INTEGRATOR_Ptr = std::unique_ptr<TF_INTEGRATOR>;

}  // namespace sun

#endif
