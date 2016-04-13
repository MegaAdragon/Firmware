
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

struct adc_msg_s {
    uint8_t      am_channel;               /* The 8-bit ADC Channel */
    int32_t      am_data;                  /* ADC convert result (4 bytes) */
} __attribute__((packed));

using namespace px4;


ADCPublisher::ADCPublisher() :
    SuperBlock(NULL, "SONAR"),
	_n(appState),
	_adc_sonar_pub(_n.advertise<px4_adc_sonar>()),
    _collision_pub(_n.advertise<px4_collision>()),
    _sonar_z_stddev(this, "SNR_Z"),
    _sonar_z_offset(this, "SNR_OFF_Z"),
    _sonarStats(this, ""),
    _timeStamp(hrt_absolute_time()),
    _time_last_sonar(0),
    _sonarInitialized(false),
    _maCount(0),
    _est_distance(0)
{
    int i;
    for(i = 0; i<5; i++) {
        _maList[i] = 0;
    }
}

px4::AppState ADCPublisher::appState;

int ADCPublisher::main()
{
    size_t readsize;
    ssize_t nbytes;
    struct adc_msg_s sample[10];
    
    int fd = open("/dev/adc0", O_RDONLY);
    if (fd < 0)
    {
        PX4_ERR("ADC Open failed");
    }
    
	px4::Rate loop_rate(10);

	while (!appState.exitRequested()) {
		loop_rate.sleep();
		_n.spinOnce();
        
        readsize = 10 * sizeof(struct adc_msg_s);
        nbytes = read(fd, sample, readsize);
        

		/* Publish example message */
		px4_adc_sonar adc_sonar_msg;
		adc_sonar_msg.data().timestamp = px4::get_time_micros();
        
        if(nbytes > 0) {
            int nsamples = nbytes / sizeof(struct adc_msg_s);
            if (nsamples * sizeof(struct adc_msg_s) != nbytes)
            {
                PX4_INFO("adc_main: read size=%d is not a multiple of sample size=%d, Ignoring\n",
                         nbytes, sizeof(struct adc_msg_s));
            }
            else
            {
                for (int i = 0; i < nsamples ; i++)
                {
                    /*
                     PX4_INFO("%d: channel: %d value: %d\n",
                     i, sample[i].am_channel, sample[i].am_data);
                     */
                    
                    if(sample[i].am_channel == 14)
                    {
                        
                        //PX4_INFO("%d: channel: %d value: %d\n", i, sample[i].am_channel, sample[i].am_data);
                        
                        adc_sonar_msg.data().id = 1;
                        adc_sonar_msg.data().raw_value = sample[i].am_data;
                        adc_sonar_msg.data().distance = float(((sample[i].am_data/6.4) * 2.54)/100);
                        
                        if (!_sonarInitialized) {
                            sonarInit(float(((sample[i].am_data/6.4) * 2.54)/100));
                            
                        } else {
                            sonarCorrect(float(((sample[i].am_data/6.4) * 2.54)/100));
                        }
                        
                    }
                }
            }
        }
        else {
            PX4_ERR("ADC read failed");
        }
        
		//PX4_INFO("adc sonar: %" PRIu64, adc_sonar_msg.data().timestamp);
		_adc_sonar_pub->publish(adc_sonar_msg);

	}

	return 0;
}
