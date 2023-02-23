//
// Created by michal on 18.02.23.
//

#include "mia.h"


void car_toggle_gear(car_t * car)
{
    if(car->in_reverse)
    {
        car->in_reverse = 0;
    }
    else
    {
        car->in_reverse = 1;
    }
}