/*
    TF_MIMO_DIAGONAL Class (linear diagonal system)

    Copyright 2018 Università della Campania Luigi Vanvitelli

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


#include "TF_MIMO_DIAGONAL.h"

using namespace std;
using namespace TooN;

/*===============CONSTRUCTORS===================*/

    TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(std::vector<TF_SISO> siso_diag_vect, double Ts):
        TF_MIMO(siso_diag_vect.size(), siso_diag_vect.size(), siso_diag_vect, Ts){}

    TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(std::vector<TF_SISO> siso_diag_vect):
        TF_MIMO_DIAGONAL(siso_diag_vect, -1){}

    TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(int dim_diag, TF_SISO base_tf_system, double Ts):
        TF_MIMO(dim_diag, dim_diag, Ts)
        {

            _siso_vect.clear();

            for(int i = 0; i < dim_diag; i++)
                _siso_vect.push_back(base_tf_system);

        }

    TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(int dim_diag, TF_SISO base_tf_system):
        TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(dim_diag, base_tf_system, -1.0){}

    TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(int dim_diag, double Ts) : 
        TF_MIMO_DIAGONAL(dim_diag, TF_SISO() ){}

    TF_MIMO_DIAGONAL::TF_MIMO_DIAGONAL(int dim_diag):
        TF_MIMO_DIAGONAL(dim_diag, -1.0){}

    /*==============================================*/

    /*=============GETTER===========================*/
	const unsigned int TF_MIMO_DIAGONAL::getDimDiag(){
        return _mimo_dim_input;
    }
    
    //return std::vector representing the TF Matrix of the MIMO SYSTEM
    std::vector<TF_SISO> TF_MIMO_DIAGONAL::getSISOMatrixVect(){
        return expandDiagToMatrix(_siso_vect);
    }

    std::vector<TF_SISO> TF_MIMO_DIAGONAL::getSISODiagVect(){
        return _siso_vect;
    }
    /*==============================================*/

    /*=============SETTER===========================*/
    //Warn! This method will consider only the elements on the diagonal!
    void TF_MIMO_DIAGONAL::setSISOMatrixVect(std::vector<TF_SISO> siso_matrix_vect ){
        if( siso_matrix_vect.size() != _mimo_dim_input*_mimo_dim_output ){
            cout << BOLDRED "[TF_MIMO_DIAGONAL::setSISOMatrixVect]: Invalid std::vector size!" CRESET << endl;
            exit(-1);
        }
        
        vector<TF_SISO> diag_vec;   

        for(int i = 0; i<_mimo_dim_input; i++)
            diag_vec.push_back( siso_matrix_vect[i*_mimo_dim_input + i] );

        setSISODiagVect(diag_vec);

    }

    void TF_MIMO_DIAGONAL::setSISODiagVect(std::vector<TF_SISO> siso_diag_vect ){
        if( siso_diag_vect.size() != _mimo_dim_input ){
            cout << BOLDRED "[TF_MIMO_DIAGONAL::setSISODiagVect]: Invalid std::vector size!" CRESET << endl;
            exit(-1);
        }
        _siso_vect.clear();
        _siso_vect = siso_diag_vect;
    }
    /*==============================================*/

    /*=============RUNNER===========================*/
    Vector<> TF_MIMO_DIAGONAL::apply( Vector<> input ){

        Vector<> out = Zeros(_mimo_dim_output);
        
        for(int i = 0; i < _mimo_dim_output ; i++){
            out[i] = _siso_vect[i].apply(input[i]);
        }

        return out;
    }
    /*==============================================*/
    /*=============VARIE===========================*/
    
    void TF_MIMO_DIAGONAL::display(){
        cout << endl << "===========================" << endl;
	    cout << "===============[TF_MIMO DIAGONAL DISP START]===============" << endl;
        for(int i = 0; i < _mimo_dim_output ; i++){
            cout << "TF DIAGONAL ELEMENT " << i+1 << "/" << _mimo_dim_output << ":" << endl;
            _siso_vect[i].display();
        }
        cout << endl << "===========================" << endl;
	    cout << "===============[TF_MIMO DIAGONAL DISP STOP]===============" << endl;
    }

    vector<TF_SISO> TF_MIMO_DIAGONAL::expandDiagToMatrix(vector<TF_SISO> siso_diag_vect){

        vector<TF_SISO> siso_matrix_vect;

        for( int i = 0; i<siso_diag_vect.size(); i++){
            siso_matrix_vect.push_back( siso_diag_vect[i] );
            if( !( i == (siso_diag_vect.size()-1) ) ){
                for( int j = 0; j<siso_diag_vect.size(); j++ ){
                    siso_matrix_vect.push_back( TF_SISO() );
                }
            }
        }

        return siso_matrix_vect;

    }
    /*==============================================*/

    /*=============OPERATORS================*/

    TF_SISO& TF_MIMO_DIAGONAL::operator()(int row, int col){
        if(row == col){
            return _siso_vect[row];
        } else{
            cout << BOLDYELLOW "[TF_MIMO_DIAGONAL::operator()(int row, int col)] row!=col in diagonal system!... row used as index" CRESET << endl;
            return _siso_vect[row];
        }
    }

    TF_SISO& TF_MIMO_DIAGONAL::operator()(int diag_index){
        return _siso_vect[diag_index];
    }
    /*==============================================*/