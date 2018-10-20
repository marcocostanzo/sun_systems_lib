/*
    TF_MIMO Class (linear system)

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

#ifndef TF_MIMO_LIB
#define TF_MIMO_LIB

#include "Linear_System.h"
#include "TF_SISO/TF_SISO.h"

class TF_MIMO : public Linear_System {
    private:

    protected:
        std::vector<TF_SISO> _siso_vect;//std::vector representing the TF Matrix of the MIMO SYSTEM
        unsigned int _mimo_dim_input;
        unsigned int _mimo_dim_output;

    public:

    /*===============CONSTRUCTORS===================*/
    virtual ~TF_MIMO();
    TF_MIMO(int dim_input, int dim_output, std::vector<TF_SISO> siso_matrix_vect, double Ts);
    TF_MIMO(int dim_input, int dim_output, std::vector<TF_SISO> siso_matrix_vect);
    TF_MIMO(int dim_input, int dim_output, double Ts);
    TF_MIMO(int dim_input, int dim_output);
    TF_MIMO(int dim_input, std::vector<TF_SISO> siso_matrix_vect, double Ts);
    TF_MIMO(int dim_input, std::vector<TF_SISO> siso_matrix_vect);
    /*==============================================*/
    /*=============GETTER===========================*/
	const unsigned int getDimInput();
    const unsigned int getDimOutput();
    
    //return std::vector representing the TF Matrix of the MIMO SYSTEM
    virtual std::vector<TF_SISO> getSISOMatrixVect();
    /*==============================================*/
    /*=============SETTER===========================*/
    virtual void setSISOMatrixVect(std::vector<TF_SISO> siso_matrix_vect );
    /*==============================================*/
    /*=============RUNNER===========================*/
    virtual TooN::Vector<> apply( TooN::Vector<> input );
    /*==============================================*/
    /*=============VARIE===========================*/
    virtual void reset();
    virtual void display();
    /*==============================================*/
    /*=============OPERATORS================*/
    virtual TF_SISO& operator()(int row, int col);
    /*==============================================*/


};

#endif