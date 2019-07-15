
/*
    TF_FIRST_ORDER_FILTER Class

    First order Low_Pass_Filter

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

#ifndef TF_FIRST_ORDER_FILTER_H
#define TF_FIRST_ORDER_FILTER_H

#include "sun_systems_lib/TF/TF_SISO.h"

class TF_FIRST_ORDER_FILTER : public TF_SISO 
{
private:

TF_FIRST_ORDER_FILTER(); //NO DEFAULT CONSTRUCTOR

protected:

double gain_;

public:
/*===============CONSTRUCTORS===================*/

TF_FIRST_ORDER_FILTER(double cut_freq, double Ts, double gain = 1.0)
:TF_SISO(   tf_first_order_get_num_coeff(cut_freq, Ts), 
            tf_first_order_get_den_coeff(cut_freq, Ts),
            Ts ),
gain_(gain)
{}

TF_FIRST_ORDER_FILTER(const TF_FIRST_ORDER_FILTER& tf) = default;

virtual ~TF_FIRST_ORDER_FILTER() override = default;

/*
    Clone the object
*/
virtual TF_FIRST_ORDER_FILTER* clone() const override
{
    return new TF_FIRST_ORDER_FILTER(*this);
}

/*==============================================*/

/*===============STATIC FUNCTIONS FOR SIMPLE CONSTRUCTOR WRITING=======*/
static TooN::Vector<2> tf_first_order_get_num_coeff(double cut_freq, double Ts)
{
    double tau = 1.0/(2.0*M_PI*cut_freq);
    double _alpha = 2.0*tau/Ts;
    return (1.0/(1.0+_alpha)) * TooN::makeVector( 1.0 , 1.0 );
}

static TooN::Vector<2> tf_first_order_get_den_coeff(double cut_freq, double Ts)
{
    double tau = 1.0/(2.0*M_PI*cut_freq);
    double _alpha = 2.0*tau/Ts;
    return TooN::makeVector( 1.0, ( (1.0-_alpha)/(1.0+_alpha) ) );
}

/*==============================================*/

/*=============GETTER===========================*/
/*==============================================*/

/*=============SETTER===========================*/
inline virtual void setTs(double Ts) override
{
    throw "[TF_FIRST_ORDER_FILTER::setTs] Cannot set Ts on TF_FIRST_ORDER_FILTER";
}

//Change the state so that the output is "output"
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
    u_vec_ = TooN::makeVector( output, output );
}
/*==============================================*/

/*=============RUNNER===========================*/
inline virtual double apply( double uk) override
{
    return gain_*TF_SISO::apply( uk );
}
/*==============================================*/

/*=============VARIE===========================*/
virtual void display() const override
{
    std::cout << 
    "TF_FIRST_ORDER_FILTER:" << std::endl <<
    "   Ts: " << ts_ << std::endl <<
    "   cut_freq: " << (1.0/(((1.0/b_vec_[0])-1.0)*ts_/2.0))/(2.0*M_PI) << std::endl <<
    "   gain: " << gain_ << std::endl;
}

virtual void display_tf() const override
{
    TF_SISO::display();
}

/*==============================================*/

};

/*=============STATIC FUNS===========================*/
/*==============================================*/

using TF_FIRST_ORDER_FILTER_Ptr = std::unique_ptr<TF_FIRST_ORDER_FILTER>;

#endif
