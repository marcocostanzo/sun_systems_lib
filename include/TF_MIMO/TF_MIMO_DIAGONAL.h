/*
    TF_MIMO_DIAGONAL Class (linear diagonal system)

    Copyright 2018 Università della Campania Luigi Vanvitelli

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

#ifndef TF_MIMO_DIAGONAL_LIB
#define TF_MIMO_DIAGONAL_LIB

#include "TF_MIMO/TF_MIMO.h"

class TF_MIMO_DIAGONAL : public TF_MIMO {
    private:

    protected:
        // from super class: std::vector<TF_SISO> _siso_vect;//std::vector representing the diagonal of TF Matrix of the MIMO SYSTEM

    public:

    /*===============CONSTRUCTORS===================*/

    TF_MIMO_DIAGONAL(const std::vector<TF_SISO_Ptr>& siso_diag_vect, double Ts);
    TF_MIMO_DIAGONAL(const std::vector<TF_SISO_Ptr>& siso_diag_vect);
    TF_MIMO_DIAGONAL(int dim_diag, const TF_SISO& base_tf_system, double Ts);
    TF_MIMO_DIAGONAL(int dim_diag, const TF_SISO& base_tf_system);
    TF_MIMO_DIAGONAL(int dim_diag, double Ts);
    TF_MIMO_DIAGONAL(int dim_diag);

    TF_MIMO_DIAGONAL( const TF_MIMO_DIAGONAL& tf );

    virtual ~TF_MIMO_DIAGONAL() = default;

    virtual TF_MIMO_DIAGONAL* clone() const;

    /*==============================================*/
    /*=============GETTER===========================*/
	const unsigned int getDimDiag();
    
    //return std::vector representing the TF Matrix of the MIMO SYSTEM
    std::vector<TF_SISO_Ptr> getSISOMatrixVect();
    std::vector<TF_SISO_Ptr> getSISODiagVect();
    /*==============================================*/
    /*=============SETTER===========================*/
    //Warn! This method will consider only the elements on the diagonal!
    virtual void setSISOMatrixVect(const std::vector<TF_SISO_Ptr>& siso_matrix_vect );
    virtual void setSISODiagVect(const std::vector<TF_SISO_Ptr>& siso_diag_vect );
    /*==============================================*/
    /*=============RUNNER===========================*/
    virtual TooN::Vector<> apply( const TooN::Vector<>& input );
    /*==============================================*/
    /*=============VARIE===========================*/
    virtual void display();
    static std::vector<TF_SISO_Ptr> expandDiagToMatrix(const std::vector<TF_SISO_Ptr>& siso_diag_vect);
    /*==============================================*/

    /*=============OPERATORS================*/
    virtual TF_SISO* operator()(int row, int col);
    virtual TF_SISO* operator()(int diag_index);
    /*==============================================*/

};

#endif