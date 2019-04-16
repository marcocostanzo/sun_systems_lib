/*
    OBSERVER_Interface Class

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

#ifndef OBSERVER_INTERFACE_H
#define OBSERVER_INTERFACE_H

#include "SS_System/SS_Interface.h"

class Observer_Interface : public SS_Interface {

private:

Observer_Interface(); //avoid default constructor

protected:

SS_Interface_Ptr _model;

Observer_Interface(const SS_Interface& model, const TooN::Vector<>& initial_state) 
    : SS_Interface(initial_state, model.getTs() ),
    _model(model.clone())
    {}

Observer_Interface(const SS_Interface& model, int order) 
    : SS_Interface( order, model.getTs() ),
    _model(model.clone())
    {}

Observer_Interface( const Observer_Interface& obs )
    :SS_Interface(obs._state, obs._model->getTs() ),
    _model(obs._model->clone()) {}

public:

virtual Observer_Interface* clone() const override = 0;

virtual ~Observer_Interface() = default;

virtual const unsigned int getDimInput() const 
{
    return _model->getDimInput();
}

virtual const unsigned int getDimOutput() const {
    return _model->getDimOutput();
}

virtual const SS_Interface_Ptr& getModel() const
{
    return _model;
}

/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state, the current input and the current measure
*/
virtual TooN::Vector<> state_f( const TooN::Vector<>& state, const TooN::Vector<>& input, const TooN::Vector<>& measure ) const = 0;

/*
    This function does not change/use the internal state of the object
    It evaluates the next state given the previous state and the current input
    Since this is an observer the input is the concat of the effective input (u) ans the measure (y)
    i.e. input = [u ; y]
*/
virtual TooN::Vector<> state_f( const TooN::Vector<>& state, const TooN::Vector<>& input ) const
{
    unsigned int real_dim_input = getDimInput();
    return state_f( state, input.slice(0,real_dim_input), input.slice( real_dim_input, input.size() - real_dim_input )  );
}

/*
    This function does not change/use the internal state of the object
    It evaluates the next output given the previous state and the current input
    Since this is an observer the input can be both the concat of the effective input (u) ans the measure (y)
    (i.e. input = [u ; y]) and just the input itself
*/
virtual TooN::Vector<> output_f( const TooN::Vector<>& state, const TooN::Vector<>& input ) const
{
    unsigned int real_dim_input = getDimInput();
    unsigned int aumented_dim_input = real_dim_input + getDimOutput();
    if( input.size() == real_dim_input ){
        return _model->output_f( state, input );
    } else if( input.size() == aumented_dim_input ){
        return _model->output_f( state, input.slice(0,real_dim_input) );
    } else {
        std::cout << WARNCOLOR "[Observer_Interface]: ERROR in output_f(), non valid input dimention" CRESET << std::endl;
        exit(-1);
    } 
}

/*
    This changes the internal state
    Evaluate the next state given the current input, the current measure (and the internal state)
*/
virtual TooN::Vector<> apply_state( const TooN::Vector<>& input, const TooN::Vector<>& measure ) {
    _state = state_f( _state, input, measure );
    return _state;
}

/*
    This changes the internal state
    Evaluate the next state given the current input (and the internal state)
    Since this is an observer the input is the concat of the effective input (u) ans the measure (y)
    i.e. input = [u ; y]
*/
// Code from base class
//virtual TooN::Vector<> apply_state( const TooN::Vector<>& input );

/*
    This changes the internal state
    Evaluate the next output given the current input, the current measure (and the internal state)
*/
virtual TooN::Vector<> apply( const TooN::Vector<>& input, const TooN::Vector<>& measure ){
    _state = state_f( _state, input, measure );
    return output_f( _state, input );
}

/*
    This changes the internal state
    Evaluate the next output given the current input (and the internal state)
    Since this is an observer the input is the concat of the effective input (u) ans the measure (y)
    i.e. input = [u ; y]
*/
// Code from base class
//virtual TooN::Vector<> apply( const TooN::Vector<>& input ) override;

virtual void display() override
{
    std::cout << "[Observer_Interface]" << std::endl <<
                 "state = " << _state << std::endl;
}


};

using Observer_Interface_Ptr = std::unique_ptr<Observer_Interface>;

#endif