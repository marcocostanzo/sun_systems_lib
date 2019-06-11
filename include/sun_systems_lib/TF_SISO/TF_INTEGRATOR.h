
/*
    TF_INTEGTATOR Class

    Copyright 2019 Università della Campania Luigi Vanvitelli

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

#include "sun_systems_lib/TF_SISO/TF_SISO.h"

class TF_INTEGRATOR : public TF_SISO
{
private:

TF_INTEGRATOR(); //NO DEFAULT CONSTRUCTOR

protected:

double gain_;

public:
/*===============CONSTRUCTORS===================*/
TF_INTEGRATOR(double Ts, double gain = 1.0)
:TF_SISO( (Ts/2.0) * TooN::makeVector( 1.0 , 1.0 ),
          TooN::makeVector(1.0, -1.0 ),
          Ts
        ),
gain_(gain)
{}

virtual ~TF_INTEGRATOR() override = default;
    
/*
    Clone the object
*/
virtual TF_INTEGRATOR* clone() const override
{
    return new TF_INTEGRATOR(*this);
}

TF_INTEGRATOR( const TF_INTEGRATOR& tf ) = default;
/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
inline virtual void setTs(double Ts) override
{
    throw "[TF_INTEGRATOR::setTs] Cannot set Ts on TF_INTEGRATOR";
}

inline virtual void setOutput(double output)
{
    if(gain_!=0.0)
    {
        output = output/gain_;
        y_vec_[0] = output;
        y_k_[0] = output;
    } else
    {
        y_vec_[0] = 0.0;
        y_k_[0] = 0.0;
    }
    u_vec_ = TooN::Zeros;
}
/*==============================================*/

/*=============RUNNER===========================*/
inline virtual double apply( double uk) override
{
    y_k_[0] = gain_*TF_SISO::apply( uk );
    return y_k_[0];
}
/*==============================================*/

/*=============VARIE===========================*/
virtual void display() const override
{
    std::cout << 
    "TF_INTEGRATOR:" << std::endl <<
    "   Ts: " << ts_ << std::endl <<
    "   gain: " << gain_ << std::endl;
}
/*==============================================*/

};

/*=============STATIC FUNS===========================*/
/*==============================================*/

using TF_INTEGRATOR_Ptr = std::unique_ptr<TF_INTEGRATOR>;

#endif
