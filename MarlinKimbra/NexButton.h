/**
 * @file NexButton.h
 *
 * API of NexButton. 
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

#ifndef __NEXBUTTON_H__
#define __NEXBUTTON_H__
#ifdef __cplusplus
#include "NexTouch.h"

/**
 * NexButton,subclass of NexTouch,provides simple methods to control button component. 
 *
 */
class NexButton: public NexTouch
{
public: /* methods */
    NexButton(NexPid pid, NexCid cid, char *name, NexTouchEventCb pop = NULL, void *pop_ptr = NULL);
    
    void attachPop(NexTouchEventCb pop, void *ptr = NULL);
    void detachPop(void);

    uint16_t getText(char *buffer, uint16_t len);
    bool setText(const char *buffer);
};

#endif /* #ifdef __cplusplus */
#endif /* #ifndef __NEXBUTTON_H__ */
