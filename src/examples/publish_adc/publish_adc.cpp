
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
 * @file publish_adc.cpp
 * read values from adc and publish them
 *
 * @author Dominik Zipperle
 */

#include "publish_adc.hpp"
#include <drivers/drv_adc.h>

#include "DevMgr.hpp"

using namespace DriverFramework;

struct adc_msg_s {
    uint8_t      am_channel;               /* The 8-bit ADC Channel */
    int32_t      am_data;                  /* ADC convert result (4 bytes) */
} __attribute__((packed));

static DevHandle h_adc;

using namespace px4;


ADCPublisher::ADCPublisher() :
SuperBlock(NULL, "SNR"),
_n(appState),
_adc_sonar_pub(_n.advertise<px4_adc_sonar>()),
_collision_pub(_n.advertise<px4_collision>()),
_sonar_stddev(this, "DEV"),
_filter_length_param(this, "FIL_LEN"),
_sonar_min_distance(this, "MIN_DIS"),
_sonar_max_distance(this, "MAX_DIS"),
_adc_channel(this, "ADC_CH"),
_filter_length(_filter_length_param.get()),
_sonarStats(this, ""),
_timeStamp(hrt_absolute_time()),
_time_last_sonar(0),
_sonarInitialized(false),
_maCount(0),
_est_count(0),
_est_distance(0)
{
    // fill buffer for the filter with zeros
    int i;
    for(i = 0; i<_filter_length; i++) {
        _maList[i] = 0;
    }
    
    for(i = 0; i<10; i++) {
        _est_buf[i] = 0;
    }
}

px4::AppState ADCPublisher::appState;

int ADCPublisher::main()
{
    /* make space for a maximum of twelve channels (to ensure reading all channels at once) */
    struct adc_msg_s buf_adc[12];
    
    DevMgr::getHandle(ADC0_DEVICE_PATH, h_adc);
    
    if (!h_adc.isValid()) {
        PX4_WARN("ADC Open failed");
    }
    
    // 10 samples per second
    px4::Rate loop_rate(10);
    
    while (!appState.exitRequested()) {
        loop_rate.sleep();
        _n.spinOnce();
        
        int ret = h_adc.read(&buf_adc, sizeof(buf_adc));
        
        px4_adc_sonar adc_sonar_msg;
        
        if (ret >= (int)sizeof(buf_adc[0])) {
            
            for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {
                
                /*
                 PX4_INFO("%d: channel: %d value: %d\n",
                 i, sample[i].am_channel, sample[i].am_data);
                 */
                
                if(buf_adc[i].am_channel == _adc_channel.get())
                {
                    
                    //PX4_INFO("%d: channel: %d value: %d\n", i, sample[i].am_channel, sample[i].am_data);
                    
                    adc_sonar_msg.data().id = 1;
                    adc_sonar_msg.data().raw_value = buf_adc[i].am_data;
                    adc_sonar_msg.data().distance = float(((buf_adc[i].am_data/6.4) * 2.54)/100);
                    
                    // filter for sonar
                    if (!_sonarInitialized) {
                        sonarInit(float(((buf_adc[i].am_data/6.4) * 2.54)/100));
                        
                    } else {
                        sonarCorrect(float(((buf_adc[i].am_data/6.4) * 2.54)/100));
                    }
                    
                    // publish data
                    adc_sonar_msg.data().timestamp = px4::get_time_micros();
                    _adc_sonar_pub->publish(adc_sonar_msg);
                    
                }
            }
        }
    }
    
    return 0;
}


