
/*
    TF_SISO Class Discrete Time Transfer Function SISO (Linear Filter)

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

#ifndef TF_SISO_LIB
#define TF_SISO_LIB

#include "sun_systems_lib/SISO_System.h"
#include "sun_systems_lib/Linear_System.h"

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

    s.t. n<=m

*/

class TF_SISO : public Linear_System, public SISO_System
{

public:
//STATIC

inline static TooN::Vector<> simplifyNumerator(const TooN::Vector<>& num_coeff, const TooN::Vector<>& den_coeff)
{
    if(den_coeff[0] == 0)
    {
        throw std::domain_error("[TF_SISO::simplifyNumerator] Division by 0");
    }
    
    if(num_coeff.size() == 0) //case void vector
    {
        return TooN::makeVector(0.0);
    }

    //find first nonzero coeff
    unsigned int effective_size = num_coeff.size();
    while(effective_size!=1)
    {
        if(num_coeff[effective_size-1] != 0.0)
            break;
        effective_size --;
    }

    return num_coeff.slice(0,effective_size)/den_coeff[0];

}

inline static TooN::Vector<> simplifyAndReduceDenominator(const TooN::Vector<>& den_coeff)
{
    if(den_coeff[0] == 0)
    {
        throw std::domain_error("[TF_SISO::simplifyAndReduceDenominator] Division by 0");
    }

    if(den_coeff.size() == 1)
    {
        return TooN::Vector<>(0); //return a void vector
    }

    //find first nonzero coeff
    unsigned int effective_size = den_coeff.size();
    while(effective_size!=1)
    {
        if(den_coeff[effective_size-1] != 0.0)
            break;
        effective_size --;
    }

    return den_coeff.slice(1,effective_size-1) / den_coeff[0];
}

private:

protected:

double ts_; //sampling time

TooN::Vector<> b_vec_; //n+1
TooN::Vector<> a_vec_; //m

TooN::Vector<> u_vec_; //n+1
TooN::Vector<> y_vec_; //m
TooN::Vector<> y_k_ = TooN::Zeros(1); // last output

public:
/*===============CONSTRUCTORS===================*/

//Full
TF_SISO(    const TooN::Vector<>& num_coeff, 
            const TooN::Vector<>& den_coeff, 
            double Ts
        )
:ts_(Ts),
b_vec_( simplifyNumerator(num_coeff, den_coeff) ),
a_vec_( simplifyAndReduceDenominator(den_coeff) ),
u_vec_( TooN::Zeros( b_vec_.size() ) ),
y_vec_( TooN::Zeros( a_vec_.size() ) )
{
    if(getNumeratorOrder() > getDenominatorOrder())
    {
        throw std::domain_error("[TF_SISO] getNumDim() > getDenDim(). Non Causal System");
    }
}

TF_SISO(    const TooN::Vector<>& num_coeff, 
            const TooN::Vector<>& den_coeff
       )
:TF_SISO(   num_coeff, 
            den_coeff, 
            NAN
        )
{}

//TF_SISO_ZERO Constructors
TF_SISO(double Ts)
:TF_SISO(   TooN::makeVector(0.0), 
            TooN::makeVector(1.0), 
            Ts
        )
{}

TF_SISO()
:TF_SISO(NAN)
{}

TF_SISO(const TF_SISO& tf) = default;

virtual ~TF_SISO() override = default;

/*
    Clone the object
*/
virtual TF_SISO* clone() const override
{
    return new TF_SISO(*this);
}

/*==============================================*/

/*=============GETTER===========================*/

inline virtual const unsigned int getNumeratorOrder() const
{
    return (b_vec_.size() - 1 );
}

inline virtual const unsigned int getDenominatorOrder() const
{
    return (a_vec_.size());
}

inline virtual double getTs() const
{
    return ts_;
}

/*==============================================*/

/*=============SETTER===========================*/

inline virtual void setTs(double Ts)
{
    ts_ = Ts;
}

/*==============================================*/

/*=============RUNNER===========================*/

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    if(input.size()!=1)
    {
        throw std::domain_error("[TF_SISO::apply(Vector)] The input has to be scalar");
    }
    apply(input[0]);
    return y_k_;    
}

inline virtual double apply( double u_k) override
{

    //Shift u_vec
    const unsigned int no = u_vec_.size() - 1 ;
    if( no!=0 )
    {
        TooN::Vector<> tmp_u_vec = u_vec_;
        u_vec_.slice(1,no) = tmp_u_vec.slice(0,no);
    }
    u_vec_[0] = u_k;

    //Shift y_vec
    const unsigned int deno = y_vec_.size();
    if(deno == 0)
    {
        y_k_[0] = b_vec_ * u_vec_;
        return y_k_[0];
    }
    if(deno!=1)
    {
        TooN::Vector<> tmp_y_vec = y_vec_;
        y_vec_.slice(1,deno-1) = tmp_y_vec.slice(0,deno-1);
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
    if(y_vec_.size()!=0)
        y_vec_ = TooN::Zeros;
    y_k_ = TooN::Zeros;
}

virtual void display() const override
{
    std::cout << "TF_SISO:" << std::endl <<
    "Num_Coeff= " << b_vec_ << std::endl <<
    "Den_Coeff= 1.0 " << a_vec_ << std::endl <<
    "State" << std::endl <<
    "u_vec= " << u_vec_ << std::endl <<
    "y_vec= " << y_vec_ << std::endl <<
    "y_k= " << y_k_ << std::endl; 
}
/*==============================================*/

};

using TF_SISO_Ptr = std::unique_ptr<TF_SISO>;

#endif
