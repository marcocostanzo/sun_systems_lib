/*
    Standard OBSERVER Class

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

#include "Observers/Standard_Observer.h"

using namespace TooN;
using namespace std;

Standard_Observer::Standard_Observer( 
    const SS_Interface& model,
    Matrix<>& L,
    const Vector<>& initial_state
    )
    :Observer_Interface(
        model, 
        initial_state
    ),
    _L(L)
    {
        if(L.num_rows() != getOrder() || L.num_cols() != getDimOutput() ){
            cout << WARNCOLOR "[Standard_Observer]: ERROR in invalid L dim" CRESET << endl;
            exit(-1);
        }
    }

Standard_Observer::Standard_Observer(
    const SS_Interface& model,
    Matrix<>& L,
    int order) 
    :Standard_Observer(
        model,
        L,
        Zeros(order)
    )
    {}

/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state, the current input and the current measure
*/
Vector<> Standard_Observer::state_f( const Vector<>& state, const Vector<>& input, const Vector<>& measure ) const
{
    return _model->state_f( state, input ) + _L * ( measure - _model->output_f( state, input ) );
}

void Standard_Observer::display()
{
    cout << "[Standard_Observer]" << endl <<
            "state = " << _state << endl;
}