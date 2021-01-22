
/*
    TF_DIFFERENTIATOR_2POLES Class

    DIFFERENTIATOR transfer function with 2 poles

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

#ifndef TF_DIFFERENTIATOR_2POLES_H
#define TF_DIFFERENTIATOR_2POLES_H

/*! \file TF_DIFFERENTIATOR_2POLES.h
    \brief This class represents a differentiator with 2 poles as Discrete Time Transfer Function System.
*/

#include "sun_systems_lib/TF/TF_DIFFERENTIATOR.h"

namespace sun
{
  //!  TF_DIFFERENTIATOR class: represents a differentiator as Discrete Time Transfer Function System.
  /*!
    This class is an differentiator as Discrete Time Transfer Function System.

    It is an abstract class.

    \sa Linear_System_Interface, TF_DIFFERENTIATOR, TF_INTEGRATOR
*/
  class TF_DIFFERENTIATOR_2POLES : public TF_DIFFERENTIATOR
  {
  private:
    TF_DIFFERENTIATOR_2POLES(); // NO DEFAULT CONSTRUCTOR

  protected:

    double tau_;

  public:
    /*===============CONSTRUCTORS===================*/

    //! Constructor
    /*!
      \param Ts sampling time
      \param tau poles time constant
      \param gain gain of the integrator, default = 1
  */
    TF_DIFFERENTIATOR_2POLES(double Ts, double tau, double gain = 1.0)
        : 
        TF_DIFFERENTIATOR(Ts, 
          TooN::makeVector(
            2.0 * Ts, 
            0.0, 
            -2.0 * Ts
            ), 
          TooN::makeVector(
            pow(Ts + 2.0 * tau, 2),
            2.0 * (pow(Ts, 2) - 4.0 * pow(tau, 2)), 
            pow(Ts - 2.0 * tau, 2)), 
        gain)
        , tau_(tau)
    {
    }

    //! Destructor
    virtual ~TF_DIFFERENTIATOR_2POLES() override = default;

    virtual TF_DIFFERENTIATOR_2POLES *clone() const override
    {
      return new TF_DIFFERENTIATOR_2POLES(*this);
    }

    //! Copy Constructor
    TF_DIFFERENTIATOR_2POLES(const TF_DIFFERENTIATOR_2POLES &tf) = default;
    /*==============================================*/

    /*=============GETTER===========================*/
    /*==============================================*/

    /*=============SETTER===========================*/

    /*==============================================*/

    /*=============RUNNER===========================*/
    /*==============================================*/

    /*=============VARIE===========================*/

    virtual void display_tf() const override
    {
      std::cout << "TF_DIFFERENTIATOR_2POLES:" << std::endl
                << "   Ts: " << ts_ << std::endl
                << "   tau: " << tau_ << std::endl
                << "   gain: " << gain_ << std::endl;
    }

    /*==============================================*/
  };

  /*=============STATIC FUNS===========================*/
  /*==============================================*/

  using TF_DIFFERENTIATOR_2POLES_Ptr = std::unique_ptr<TF_DIFFERENTIATOR_2POLES>;

} // namespace sun

#endif
