/*
    TF_MIMO Class, a matrix of TF_SISO

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

#ifndef MIMO_MIMO_H
#define MIMO_MIMO_H

/*! \file TF_MIMO.h
    \brief This class represents a Discrete Time MIMO Transfer Function System.
*/

#include <sun_systems_lib/Discrete_System_Interface.h>
#include <sun_systems_lib/TF/TF_SISO.h>

namespace sun
{
//!  TF_MIMO class: represents a Discrete Time MIMO Transfer Function System.
/*!
    This class is basically a matrix of SISO Systems.

    \verbatim
    --                         --
    | TF_11   TF_12  ...  TF_1N |
    | TF_21   TF_22  ...  TF_2N |
    | ...     ...    ...  ...   |
    | TF_M1   TF_M2  ...  TF_MN |
    --                         --
    \endverbatim

    \sa Linear_System_Interface, TF_FIRST_ORDER_FILTER, TF_SISO, TF_MIMO_DIAGONAL
*/
class TF_MIMO : public Discrete_System_Interface
{
private:
  TF_MIMO();

protected:
  //! Storage fro the SISO TF systems
  std::vector<TF_SISO_Ptr> siso_vect_;

  //! Input dimension
  unsigned int dim_input_;

  //! Last output
  TooN::Vector<> y_k_;

public:
  //! Zero Constructor
  /*
    Construct a MIMO system composed by all Zero TF System
    Use setSISO after construction to set the SISO elements
    \param dim_input input dimension
    \param dim_output output dimension
  */
  TF_MIMO(unsigned int dim_input, unsigned int dim_output) : dim_input_(dim_input), y_k_(TooN::Zeros(dim_output))
  {
    int num_siso = dim_input_ * dim_output;
    for (int i = 0; i < num_siso; i++)
    {
      siso_vect_.push_back(TF_SISO_Ptr(new TF_SISO()));
    }
  }

  //! Copy constructor
  TF_MIMO(const TF_MIMO& mimo) : y_k_(mimo.y_k_)
  {
    dim_input_ = mimo.dim_input_;
    for (const auto& siso : mimo.siso_vect_)
    {
      siso_vect_.push_back(TF_SISO_Ptr(siso->clone()));
    }
  }

  virtual TF_MIMO* clone() const override
  {
    return new TF_MIMO(*this);
  }

  //! Destructor
  virtual ~TF_MIMO() override = default;

  ///////////////////////////////////

  //! Set a SISO element
  /*!
    Access to the element (index_row,index_col) and set it to siso
    \param index_row
    \param index_col
    \param siso the new siso element
  */
  virtual void setSISO(unsigned int index_row, unsigned int index_col, const TF_SISO& siso)
  {
    if (index_row > getSizeOutput() || index_col > dim_input_)
    {
      throw std::out_of_range("[TF_MIMO::setSISO] Index out of range");
    }

    siso_vect_[index_row * dim_input_ + index_col] = TF_SISO_Ptr(siso.clone());
  }

  //////////////////////////////////

  inline virtual const TooN::Vector<>& apply(const TooN::Vector<>& input) override
  {
    y_k_ = TooN::Zeros;
    int dim_output = getSizeOutput();
    for (int i = 0; i < dim_output; i++)
    {
      for (int j = 0; j < dim_input_; j++)
      {
        y_k_[i] += siso_vect_[i * dim_input_ + j]->apply(input[j]);
      }
    }
    return y_k_;
  }

  inline virtual void reset() override
  {
    y_k_ = TooN::Zeros;
    for (auto& siso : siso_vect_)
    {
      siso->reset();
    }
  }

  virtual const unsigned int getSizeInput() const override
  {
    return dim_input_;
  }

  virtual const unsigned int getSizeOutput() const override
  {
    return siso_vect_.size() / dim_input_;
  }

  virtual void display() const override
  {
    int dim_output = getSizeOutput();
    std::stringstream str;
    str << "TF_MIMO " << dim_output << "x" << dim_input_ << ":" << std::endl;

    for (int i = 0; i < dim_output; i++)
    {
      for (int j = 0; j < dim_input_; j++)
      {
        str << "Position [" << i << "][" << j << "]" << std::endl;
        siso_vect_[i * dim_input_ + j]->display_tf();
        str << "-----------------------------------" << std::endl;
      }
    }

    str << "TF_MIMO [END]" << std::endl;

    std::cout << str.str();
  }
};

using TF_MIMO_Ptr = std::unique_ptr<TF_MIMO>;

}  // namespace sun

#endif