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

#include "TF_MIMO/TF_MIMO.h"

using namespace std;
using namespace TooN;

/*===============CONSTRUCTORS===================*/
    TF_MIMO::TF_MIMO(int dim_input, int dim_output, const vector<TF_SISO_Ptr>& siso_matrix_vect, double Ts):
        Linear_System(Ts),
        _mimo_dim_input(dim_input),
        _mimo_dim_output(dim_output)
        {
            if( siso_matrix_vect.size() != dim_input*dim_output ){
                cout << ERRORCOLOR "[TF_MIMO Constructor]: Invalid std::vector size!" CRESET << endl;
                exit(-1);
            }
            for( const auto &element : siso_matrix_vect ){
                _siso_vect.push_back( TF_SISO_Ptr( element->clone() ) );
            }
        }
    
    TF_MIMO::TF_MIMO(int dim_input, int dim_output, const vector<TF_SISO_Ptr>& siso_matrix_vect):
        TF_MIMO(dim_input, dim_output, siso_matrix_vect, -1){}

    TF_MIMO::TF_MIMO(int dim_input, int dim_output, double Ts):
        Linear_System(Ts),
        _mimo_dim_input(dim_input),
        _mimo_dim_output(dim_output),
        _siso_vect(){

            for(int i = 0; i < dim_input*dim_output; i++)
                _siso_vect.push_back( TF_SISO_Ptr( new TF_SISO() ) );
        }

    TF_MIMO::TF_MIMO(int dim_input, int dim_output):
        TF_MIMO(dim_input, dim_output, 1.0){}

    TF_MIMO::TF_MIMO(int dim_input, const vector<TF_SISO_Ptr>& siso_matrix_vect, double Ts):
        TF_MIMO(dim_input, siso_matrix_vect.size()/dim_input, siso_matrix_vect, Ts){}

    TF_MIMO::TF_MIMO(int dim_input, const vector<TF_SISO_Ptr>& siso_matrix_vect):
        TF_MIMO(dim_input, siso_matrix_vect, -1){}

/*==============================================*/

/*=============GETTER===========================*/
	const unsigned int TF_MIMO::getDimInput(){
        return _mimo_dim_input;
    }
    const unsigned int TF_MIMO::getDimOutput(){
        return _mimo_dim_output;
    }
    
    //return std::vector representing the TF Matrix of the MIMO SYSTEM
    vector<TF_SISO_Ptr> TF_MIMO::getSISOMatrixVect(){
        vector<TF_SISO_Ptr> out;
        for( const auto &element : _siso_vect ){
            out.push_back( TF_SISO_Ptr( element->clone() )  );
        }
        return out;
    }

    /*=============SETTER===========================*/
    void TF_MIMO::setSISOMatrixVect(const std::vector<TF_SISO_Ptr>& siso_matrix_vect ){
        if( siso_matrix_vect.size() != _mimo_dim_input*_mimo_dim_output ){
            cout << ERRORCOLOR "[TF_MIMO::setSISOMatrixVect]: Invalid std::vector size!" CRESET << endl;
            exit(-1);
        }
        _siso_vect.clear();
        for( const auto &element : siso_matrix_vect ){
            _siso_vect.push_back( TF_SISO_Ptr( element->clone() ) );
        }
    }
    /*==============================================*/

    /*=============RUNNER===========================*/

    Vector<> TF_MIMO::apply( TooN::Vector<> input ){

        Vector<> out = Zeros(_mimo_dim_output);
        
        for(int i = 0; i < _mimo_dim_output ; i++){
            for(int j = 0; j < _mimo_dim_input; j++ ){
                out[i] += _siso_vect[ i*_mimo_dim_input + j ]->apply(input[j]);
            }
        }

        return out;

    }

    /*==============================================*/

    /*=============VARIE===========================*/

    void TF_MIMO::reset(){
        for(const auto &tf : _siso_vect){
            tf->reset();
        }
    }

    void TF_MIMO::display(){
        cout << endl << "===========================" << endl;
	    cout << "===============[TF_MIMO DISP START]===============" << endl;
        for(int i = 0; i < _mimo_dim_output ; i++){
            for(int j = 0; j < _mimo_dim_input; j++ ){
                cout << "TF " << i+1 << "/" << _mimo_dim_output << "|" << j+1 << "/" << _mimo_dim_input << ":" << endl;
                _siso_vect[ i*_mimo_dim_input + j ]->display();
            }
        }
        cout << endl << "===========================" << endl;
	    cout << "===============[TF_MIMO DISP STOP]===============" << endl;
    }

    /*==============================================*/

    /*=============OPERATORS================*/
    TF_SISO* TF_MIMO::operator()(int row, int col){
        return _siso_vect[ row*_mimo_dim_input + col ].get();
    }
    /*==============================================*/