/*
    State Space Continuous Time System Interface Class

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

#ifndef CONTINUOUS_SYSTEM_INTERFACE_H
#define CONTINUOUS_SYSTEM_INTERFACE_H

/*! \file Continuous_System_Interface.h
    \brief This class represents a generic continuous state space System
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
//!  Continuous_System_Interface class: represents a generic continuous state space system.
/*!
    This class is a generic continuous state space system in the form:

    x_dot = f(x,u)

    y = h(x,u)

    You can't directly simulate a continuous system, you have to discretize it. See Discretizator_Interface, RK4.

    \sa Continuous_System, Discretizator_Interface, RK4
*/
class Continuous_System_Interface
{
private:
protected:
public:
  //! Clone the object
  virtual Continuous_System_Interface* clone() const = 0;

  //! A destructor
  virtual ~Continuous_System_Interface() = default;

  //!  The state function
  /*!
      returns the state derivative x_dot
      \param x The system state
      \param u The system input
      \return The state derivative x_dot
  */
  virtual const TooN::Vector<> state_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u) const = 0;

  //!  The output function
  /*!
      returns the system output y
      \param x The system state
      \param u The system input
      \return The system output y
  */
  virtual const TooN::Vector<> output_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u) const = 0;

  //!  The state function Jacobian
  /*!
      returns the state function Jacobian F
      \param x The system state
      \param u The system input
      \return The state function Jacobian F
  */
  virtual const TooN::Matrix<> jacob_state_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u) const = 0;

  //!  The output function Jacobian
  /*!
      returns the output function Jacobian H
      \param x The system state
      \param u The system input
      \return The output function Jacobian H
  */
  virtual const TooN::Matrix<> jacob_output_fcn(const TooN::Vector<>& x, const TooN::Vector<>& u) const = 0;

  //!  The input size
  /*!
      \return the system input size
  */
  virtual const unsigned int getSizeInput() const = 0;

  //!  The output size
  /*!
      \return the system output size
  */
  virtual const unsigned int getSizeOutput() const = 0;

  //!  The state size
  /*!
      \return the system state size
  */
  virtual const unsigned int getSizeState() const = 0;

  //! Display the system on the std out
  virtual void display() const
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Continuous_System_Interface" CRESET << std::endl;
  }
};

using Continuous_System_Interface_Ptr = std::unique_ptr<Continuous_System_Interface>;

}  // namespace sun

#endif