/*
    Discrete Time System Interface Class

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

#ifndef DISCRETE_SYSTEM_INTERFACE_H
#define DISCRETE_SYSTEM_INTERFACE_H

/*! \file Discrete_System_Interface.h
    \brief This class represents a generic discrete System
*/

#include <TooN/TooN.h>
#include <memory>

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLD "\033[1m"                /* Bold */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */
/*===============================*/

#endif

namespace sun
{
//!  Discrete_System_Interface class: represents a generic discrete time system.
/*!
    This class is a generic discrete system

    \verbatim

    u_k  /-------\  y_k
    ---->|  Sys  |---->
         \-------/

    \endverbatim

    It stores the internal system state

    \sa Discretizator_Interface, RK4
*/
class Discrete_System_Interface
{
private:
protected:
public:
  //! Clone the object in the heap
  virtual Discrete_System_Interface* clone() const = 0;

  //! Destructor
  virtual ~Discrete_System_Interface() = default;

  //! Apply the system, compute the output and update the internal state
  /*!
    Go one discrete step ahead, apply the input u(k), update the internal state for the next step,
    return the output y(k)
    \param u_k Input at the current step u(k)
    \return system output y(k)
  */
  virtual const TooN::Vector<>& apply(const TooN::Vector<>& u_k) = 0;

  //! Reset the system
  /*!
    Reset the system state to a zero value
  */
  virtual void reset() = 0;

  //! Get Size of the input
  virtual const unsigned int getSizeInput() const = 0;

  //! Get Size of the output
  virtual const unsigned int getSizeOutput() const = 0;

  //! Dysplay the system on the std out
  virtual void display() const
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Discrete_System_Interface" CRESET << std::endl;
  }
};

using Discrete_System_Interface_Ptr = std::unique_ptr<Discrete_System_Interface>;

}  // namespace sun

#endif