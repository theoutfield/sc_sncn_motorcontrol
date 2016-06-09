/*
 * protection.h
 *
 *  Created on: Oct 29, 2015
 *      Author: ramin
 */


#ifndef PROTECTION_H_
#define PROTECTION_H_



interface system_state
{
    void protect(int fault_id);
};

interface fault_type
{
    void set_fault_type(int);
};


#endif /* PROTECTION_H_ */
