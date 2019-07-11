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

#include <sun_systems_lib/SS/SS_Interface.cpp>
#include <sun_systems_lib/Linear_System_Interface.cpp>

class SS_LINEAR : public SS_Interface, public Linear_System_Interface
{

private:

SS_LINEAR();

protected:

TooN::Matrix<> A_, B_, C_, D_;

public:

SS_LINEAR(  const TooN::Matrix<>& A, 
            const TooN::Matrix<>& B, 
            const TooN::Matrix<>& C, 
            const TooN::Matrix<>& D 
        )
:A_(A),
B_(B),
C_(C),
D_(D),
SS_Interface( TooN::Zeros(A.num_rows()), TooN::Zeros(C.num_rows()) )
{
    if(!chek_dimensions())
    {
        throw std::invalid_argument( "[SS_LINEAR] Invalid matrix dimensions" );
    }
}

SS_LINEAR(const SS_LINEAR& ss) = default;

virtual SS_LINEAR* clone() const override
{
    return new SS_LINEAR(*this);
}

virtual ~SS_LINEAR() override = default;

/////////////////////////////////////////

virtual const unsigned int getSizeInput() const override
{
    return B_.num_cols();
}

virtual const unsigned int getSizeOutput() const override
{
    return C_.num_rows();
}

////////////////////////////////////////

inline virtual const TooN::Vector<> state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    return A_*x_k_1 + B_*u_k;
}

inline virtual const TooN::Vector<> output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return C_*x_k + D_*u_k;
}

inline virtual const TooN::Matrix<> jacob_state_fcn( const TooN::Vector<>& x_k_1, const TooN::Vector<>& u_k ) const override
{
    return A_;
}

inline virtual const TooN::Matrix<> jacob_output_fcn( const TooN::Vector<>& x_k, const TooN::Vector<>& u_k ) const override
{
    return C_;
}

inline virtual const TooN::Vector<>& apply( const TooN::Vector<>& input ) override
{
    return SS_Interface::apply(input);
}

virtual void reset() override
{
    return SS_Interface::reset();
}

virtual void display() const override
{
    std::cout <<
    "SS_LINEAR:" << std::endl <<
    "A:" << std::endl <<
    A_ << std::endl <<
    "B:" << std::endl <<
    B_ << std::endl <<
    "C:" << std::endl <<
    C_ << std::endl <<
    "D:" << std::endl <<
    D_ << std::endl <<
    "state: " << state_ << std::endl <<
    "SS_LINEAR [END]" << std::endl;
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
    if(output_.size() != C_.num_rows())
        return false;

    return true;
}

};

using SS_LINEAR_Ptr = std::unique_ptr<SS_LINEAR>;

#endif