/*
    SISO_System_Interface Class, Interface for Discrete single-input-single-output systems

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

#ifndef SISO_SYSTEM_INTERFACE_H
#define SISO_SYSTEM_INTERFACE_H

/*! \file SISO_System_Interface.h
    \brief This class represents a generic Discrete Time SISO System
*/

#include "sun_systems_lib/Discrete_System_Interface.h"

namespace sun
{
//!  SISO_System_Interface class: represents a generic Discrete Time SISO system.
/*!
    This class is a generic discrete time SISO system

    \verbatim

    u_k  /-------\  y_k
    ---->|  Sys  |---->
         \-------/

    \endverbatim

    It stores the internal system state

    \sa Discretizator_Interface, RK4, Discrete_System_Interface
*/
class SISO_System_Interface : public Discrete_System_Interface
{
private:
protected:
public:

  virtual SISO_System_Interface* clone() const override = 0;

  //! Destructor
  virtual ~SISO_System_Interface() override = default;

  //! Apply the system, compute the output and update the internal state
  /*!
    Go one discrete step ahead, apply the input u(k), update the internal state for the next step,
    return the output y(k)
    This is a SISO system, the input and output must be 1D Vectors.
    Consider to use apply(double)
    \param u_k Input at the current step u(k)
    \return system output y(k)
  */
  virtual const TooN::Vector<>& apply(const TooN::Vector<>& u_k) override = 0;

  //! Apply the system, compute the output and update the internal state
  /*!
    Go one discrete step ahead, apply the input u(k), update the internal state for the next step,
    return the output y(k)
    This is the SISO system overload, scalar input and output
    \param u_k Input at the current step u(k)
    \return system output y(k)
  */
  virtual double apply(double u_k) = 0;

  virtual void reset() override = 0;

  virtual const unsigned int getSizeInput() const override
  {
    return 1;
  }

  virtual const unsigned int getSizeOutput() const override
  {
    return 1;
  }

  virtual void display() const override
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for SISO_System_Interface" CRESET << std::endl;
  }
};

using SISO_System_Interface_Ptr = std::unique_ptr<SISO_System_Interface>;

}  // namespace sun

#endif