//
// Created by 1 on 2024-05-20.
//

#ifndef PANDOGIN_DOG_SUBORDINATE_DESK_H
#define PANDOGIN_DOG_SUBORDINATE_DESK_H

#include "main.h"
#include "cmsis_os.h"

#define Length_of_Desk 10

extern uint8_t Desk_Data[Length_of_Desk];
extern uint16_t TOF_Distance;

void Process(void );

#endif //PANDOGIN_DOG_SUBORDINATE_DESK_H
