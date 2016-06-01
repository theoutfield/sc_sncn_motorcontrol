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


#define V_DC_MAX    2000
#define V_DC_MIN    0

#define I_MAX          3000
#define TEMP_BOARD_MAX 100



#endif /* PROTECTION_H_ */
