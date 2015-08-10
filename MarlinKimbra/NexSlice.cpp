/**
 * @file NexSlice.cpp
 *
 * API of NexSlice. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/7/10
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include "NexSlice.h"

/**
 * Constructor,inherited NexTouch's constructor function.
 *
 */
NexSlice::NexSlice(NexPid pid, NexCid cid, char *name, NexTouchEventCb pop, void *pop_ptr)
    :NexTouch(pid, cid, name, pop, pop_ptr)
{
}

/*
 * Get the number of picture. 
 *
 * @param number - an output parameter to save the number of picture. 
 * 
 * @retval true - success. 
 * @retval false - failed. 
 */
bool NexSlice::getPic(uint32_t *number)
{
    String cmd = String("get ");
    cmd += getObjName();
    cmd += ".picc";
    sendCommand(cmd.c_str());
    return recvRetNumber(number);
}

/*
 * Set the number of picture. 
 *
 * @param number - the number of picture. 
 * 
 * @retval true - success. 
 * @retval false - failed. 
 */
bool NexSlice::setPic(uint32_t number)
{
    char buf[10] = {0};
    String cmd;
    
    utoa(number, buf, 10);
    cmd += getObjName();
    cmd += ".picc=";
    cmd += buf;

    sendCommand(cmd.c_str());
    return recvRetCommandFinished();
}

/**
 * Register slice pop callback function. 
 *
 * @param pop - the pointer to slice pop callback function.
 * @param ptr - the parameter to be transmitted to slice pop callback function. 
 */
void NexSlice::attachPop(NexTouchEventCb pop, void *ptr)
{
    NexTouch::attachPop(pop, ptr);
}

/**
 * Unload slice pop callback function.
 *
 */
void NexSlice::detachPop(void)
{
    NexTouch::detachPop();
}
 
