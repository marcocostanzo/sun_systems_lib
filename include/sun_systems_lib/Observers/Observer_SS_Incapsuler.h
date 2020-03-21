/*
    State Observer Class from SS Class
    incapsulate a SS class so that the input u is splitted in an actual input and in a measured output

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

#ifndef OBSERVER_SS_INCAPSULER_H
#define OBSERVER_SS_INCAPSULER_H

#include <sun_systems_lib/Observers/Observer_Interface.h>

namespace sun
{
class Observer_SS_Incapsuler : public Observer_Interface
{
private:
protected:
  SS_Interface_Ptr system_;

  ////SS_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
  ////            :state_(state),
  ////            output_(output)
  ////            {}

  // Observer_Interface( const TooN::Vector<>& state, const TooN::Vector<>& output )
  //            :SS_Interface( state, output )
  //            {}

public:
  Observer_SS_Incapsuler(const SS_Interface& system)
    : Observer_Interface(TooN::Vector<>(0), 0), system_(system.clone())
  {
  }

  Observer_SS_Incapsuler(const Observer_SS_Incapsuler& ss) : Observer_Interface(ss), system_(ss.system_->clone())
  {
  }

  virtual Observer_SS_Incapsuler* clone() const override
  {
    return new Observer_SS_Incapsuler(*this);
  }

  virtual ~Observer_SS_Incapsuler() override = default;

  inline virtual const TooN::Vector<>& getState() const override
  {
    return system_->getState();
  }

  inline virtual void setState(const TooN::Vector<>& state) override
  {
    system_->setState(state);
  }

  inline virtual const TooN::Vector<> obs_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k,
                                                    const TooN::Vector<>& y_k) const override
  {
    return system_->state_fcn(x_k_1, buildFullInput(u_k, y_k));
  }

  inline virtual const TooN::Vector<> state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k) const override
  {
    return system_->state_fcn(x_k_1, u_k);
  }

  inline virtual const TooN::Vector<> obs_output_fcn(const TooN::Vector<>& x_k,
                                                     const TooN::Vector<>& u_k) const override
  {
    return system_->output_fcn(x_k, buildFullInput(u_k, TooN::Zeros(getSizeOutput())));
  }

  inline virtual const TooN::Vector<> output_fcn(const TooN::Vector<>& x_k, const TooN::Vector<>& u_k) const override
  {
    return system_->output_fcn(x_k, u_k);
  }

  inline virtual const TooN::Matrix<> obs_jacob_state_fcn(const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k,
                                                          const TooN::Vector<>& y_k) const override
  {
    return system_->jacob_state_fcn(x_k_1, buildFullInput(u_k, y_k));
  }

  inline virtual const TooN::Matrix<> jacob_state_fcn(const TooN::Vector<>& x_k_1,
                                                      const TooN::Vector<>& u_k) const override
  {
    return system_->jacob_state_fcn(x_k_1, u_k);
  }

  inline virtual const TooN::Matrix<> obs_jacob_output_fcn(const TooN::Vector<>& x_k,
                                                           const TooN::Vector<>& u_k) const override
  {
    return system_->jacob_output_fcn(x_k, buildFullInput(u_k, TooN::Zeros(getSizeOutput())));
  }

  inline virtual const TooN::Matrix<> jacob_output_fcn(const TooN::Vector<>& x_k,
                                                       const TooN::Vector<>& u_k) const override
  {
    return system_->jacob_output_fcn(x_k, u_k);
  }

  virtual const TooN::Vector<>& obs_apply(const TooN::Vector<>& input, const TooN::Vector<>& measure)
  {
    return system_->apply(buildFullInput(input, measure));
  }

  inline virtual const TooN::Vector<>& apply(const TooN::Vector<>& input) override
  {
    return system_->apply(input);
  }

  virtual void reset() override
  {
    Observer_Interface::reset();  // Not needed...
    system_->reset();
  }

  // virtual const unsigned int getSizeRealInput() const
  //{
  //    return getSizeInput() - getSizeOutput();
  //}

  inline virtual const unsigned int getSizeInput() const override
  {
    return system_->getSizeInput();
  }

  inline virtual const unsigned int getSizeOutput() const override
  {
    return system_->getSizeOutput();
  }

  inline virtual const unsigned int getSizeState() const override
  {
    return system_->getSizeState();
  }

  virtual void display() const override
  {
    std::cout << BOLDYELLOW "WARNING! display() not implemented for Observer_SS_Incapsuler" CRESET << std::endl;
  }
};

using Observer_SS_Incapsuler_Ptr = std::unique_ptr<Observer_SS_Incapsuler>;

}  // namespace sun

#endif