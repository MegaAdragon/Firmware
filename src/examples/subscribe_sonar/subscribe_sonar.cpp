/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file subscribe_sonar.cpp
 *
 * @author Dominik Zipperle
 */

#include "subscribe_sonar.hpp"
#include <drivers/drv_tone_alarm.h>
#include <drivers/drv_rgbled.h>
#include <drivers/drv_hrt.h>

#include "DevMgr.hpp"

using namespace DriverFramework;
using namespace px4;

static DevHandle h_rgbleds;
static DevHandle h_buzzer;

static const long tone_length = 900000;
static hrt_abstime tune_end = 0;

static bool reset_LED = false;

void adc_sonar_callback_function(const px4_adc_sonar &msg)
{
    PX4_INFO("I heard: [%" PRIu64 "] + [%5.2f]", msg.data().timestamp, double(msg.data().distance));
    
    /* TODO: Do something with the sonar value */
}

void collision_callback_function(const px4_collision &msg)
{
    PX4_INFO("COL: [%" PRIu64 "] + [%5.2f]", msg.data().timestamp, double(msg.data().front));
    
    /* Playing Alarm Signals when front Sonar is getting close to obstacles */
    if(double(msg.data().front) <= 1.0 && (hrt_absolute_time() > tune_end))
    {
        tune_end = hrt_absolute_time() + tone_length;
        
        /* buzzer patterns are ordered by
         * priority, with a higher-priority pattern interrupting any
         * lower-priority pattern that might be playing.
         */
        h_buzzer.ioctl(TONE_SET_ALARM, TONE_NOTIFY_NEGATIVE_TUNE);
        h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_RED);
        h_rgbleds.ioctl(RGBLED_SET_PATTERN, RGBLED_MODE_BLINK_FAST);
        reset_LED = true;
    }
    /* reset LED to default mode */
    else if (reset_LED && (hrt_absolute_time() > tune_end)) {
        // TODO: reset to previous LED color and pattern
        h_rgbleds.ioctl(RGBLED_SET_COLOR, RGBLED_COLOR_BLUE);
        h_rgbleds.ioctl(RGBLED_SET_PATTERN, RGBLED_MODE_BREATHE);
        reset_LED = false;
    }
}

SubscribeSonar::SubscribeSonar() :
_n(_appState)
{
    DevMgr::getHandle(TONEALARM0_DEVICE_PATH, h_buzzer);
    
    if (!h_buzzer.isValid()) {
        PX4_WARN("Buzzer: px4_open fail\n");
    }
    
    DevMgr::getHandle(RGBLED0_DEVICE_PATH, h_rgbleds);
    
    if (!h_rgbleds.isValid()) {
        PX4_WARN("No RGB LED found at " RGBLED0_DEVICE_PATH);
    }
    
    /* Do some subscriptions */
    //TODO: check interval value
    _n.subscribe<px4_adc_sonar>(adc_sonar_callback_function, 100);
    _n.subscribe<px4_collision>(collision_callback_function, 100);
    
    PX4_INFO("subscribed");
}
