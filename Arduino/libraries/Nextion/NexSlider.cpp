/**
 * @file NexSlider.cpp
 *
 * The implementation of class NexSlider. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/13
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
#include "NexSlider.h"

NexSlider::NexSlider(uint8_t pid, uint8_t cid, const char *name)
    :NexTouch(pid, cid, name)
{
}

bool NexSlider::getValue(uint32_t *number)
{
    String cmd = String("get ");
    cmd += getObjName();
    cmd += ".val";
    sendCommand(cmd.c_str());
    return recvRetNumber(number);
}

bool NexSlider::setValue(uint32_t number)
{
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += getObjName();
    cmd += ".val=";
    cmd += buf;

    sendCommand(cmd.c_str());
    return recvRetCommandFinished();
}

bool NexSlider::setMaxValue(uint32_t number)
{
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += getObjName();
    cmd += ".maxval=";
    cmd += buf;

    sendCommand(cmd.c_str());
    return recvRetCommandFinished();
}

bool NexSlider::setHigValue(uint32_t number)
{
    char buf[10] = {0};
    String cmd;

    utoa(number, buf, 10);
    cmd += getObjName();
    cmd += ".hig=";
    cmd += buf;

    sendCommand(cmd.c_str());
    return recvRetCommandFinished();
}
