/*
    Linea State Space Class

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

#ifndef SS_LINEAR_H
#define SS_LINEAR_H

#include <sun_systems_lib/SS/SS_Interface.h>
#include <sun_systems_lib/Linear_System.h>

class SS_Linear : public SS_Interface, public Linear_System
{

private:

SS_Linear();

protected:

TooN::Matrix<> A_, B_, C_, D_;
TooN::Vector<> state_;

TooN::Vector<> tmp_state_;
TooN::Vector<> tmp_output_;

public:

SS_Linear(  const TooN::Matrix<>& A, 
            const TooN::Matrix<>& B, 
            const TooN::Matrix<>& C, 
            const TooN::Matrix<>& D 
        )
:A_(A),
B_(B),
C_(C),
D_(D),
state_( TooN::Zeros(A.num_rows()) ),
tmp_state_( TooN::Zeros(A.num_rows()) ),
tmp_output_( TooN::Zeros(C.num_rows()) )
{
    if(!chek_dimensions())
    {
        throw std::invalid_argument( "[SS_Linear] Invalid matrix dimensions" );
    }
}

SS_Linear(const SS_Linear& ss) = default;

virtual SS_Linear* clone() const
{
    return new SS_Linear(*this);
}

virtual ~SS_Linear() = default;

/////////////////////////////////////////

inline virtual const TooN::Vector<>& get_state() const override
{
    return state_;
}

////////////////////////////////////////

inline virtual const TooN::Vector<>& state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) override
{
    tmp_state_ = A_*x_k_1 + B_*u_k;
    return tmp_state_;
}

inline virtual const TooN::Vector<>& output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) override
{
    tmp_output_ = C_*x_k + D_*u_k;
    return tmp_output_;
}

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    state_ = A_*state_ + B_*input;
    tmp_output_ = C_*state_ + D_*input;
    return tmp_output_;
}

virtual void reset() override
{
    state_ = TooN::Zeros;
    tmp_state_ = TooN::Zeros;
    tmp_output_ = TooN::Zeros;
}

virtual void display() const
{
    std::cout <<
    "SS_Linear:" << std::endl <<
    "A:" << std::endl <<
    A_ << std::endl <<
    "B:" << std::endl <<
    B_ << std::endl <<
    "C:" << std::endl <<
    C_ << std::endl <<
    "D:" << std::endl <<
    D_ << std::endl <<
    "state: " << state_ << std::endl <<
    "SS_Linear [END]" << std::endl;
}

virtual bool chek_dimensions()
{
    if(A_.num_rows() != A_.num_cols())
        return false;
    if(A_.num_rows() != B_.num_rows())
        return false;
    if(A_.num_cols() != C_.num_cols())
        return false;

    if(B_.num_cols() != D_.num_cols())
        return false;
    
    if(C_.num_rows() != D_.num_rows())

    if(state_.size() != A_.num_rows())
        return false;
    if(tmp_state_.size() != state_.size())
        return false;
    if(tmp_output_.size() != C_.num_rows())
        return false;

    return true;
}

};

using SS_Linear_Ptr = std::unique_ptr<SS_Linear>;

#endif