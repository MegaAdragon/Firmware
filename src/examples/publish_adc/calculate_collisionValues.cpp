
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
 * @file calculate_collisonValues.cpp
 * read values from adc and publish them
 *
 * @author Dominik Zipperle
 */

#include "publish_adc.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>
#include <cmath>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
//static const int 		REQ_SONAR_INIT_COUNT = FILTER_LENGTH;
static const uint32_t 	SONAR_TIMEOUT =   1000000; // 1.0 s

void ADCPublisher::sonarInit(float current_distance)
{
    int i;
    
    if(_sonarStats.getCount() <= 1) {
        _est_distance = current_distance;
    }
    
    // measure
    float distance = sonarMeasure(current_distance);
    
    // if distance is not valid -> reset init
    if (distance < 0) {
        _sonarStats.reset();
        return;
    }
    
    // fill buffer with values
    _est_buf[_est_count] = distance;
    _est_count ++;
    
    if(_est_count > 9){
        _est_count = 0;
    }
    
    // if init is finished
    if (_sonarStats.getCount() > 10) {
        
        mavlink_and_console_log_info(&mavlink_log_pub, "[collision] sonar init "
                                     "mean %d cm std %d cm",
                                     int(100 * _sonarStats.getMean()(0)),
                                     int(100 * _sonarStats.getStdDev()(0)));
        
        
        float sum = 0;
        for(i=0; i<10; i++){
            sum += float(_est_buf[i]);
        }
        
        if(sum > 0) {
            _est_distance = sum / 10;
        }
        else { return; }
        
        _sonarInitialized = true;
        return;
    }
    
}

float ADCPublisher::sonarMeasure(float current_distance)
{
    // measure
    float d = current_distance;
    float eps = 0.01f;
    /*
     float min_dist = _sub_sonar->get().min_distance + eps;
     float max_dist = _sub_sonar->get().max_distance - eps;
     */
    
    float min_dist = _sonar_min_distance.get() + eps;
    float max_dist = _sonar_max_distance.get() - eps;
    
    // check for bad data
    if (d > max_dist || d < min_dist) {
        //PX4_INFO("out of range");
        return -1;
    }
    
    // update stats
    _sonarStats.update(Scalarf(d));
    _time_last_sonar = _timeStamp;
    
    // use fault detection only if sonar is initialized
    
    if (_sonarInitialized)
    {
        // calculate differenz between current value and estimated value
        double beta = fabs(d - _est_distance);
        
        //float cov = _sub_sonar->get().covariance;
        //TODO: use parameter here
        
        /*
         normal stddev for sonar is 0.05
         TODO: does this have to change over time?
         
         with stddev 0.05 -> fault detection at 0.148m
         with stddev 0.1 -> fault detection at 0.297m
         */
        double stddev = _sonar_stddev.get();
        
        // if fault is detected -> return invalid
        if (beta > (sqrt(BETA_TABLE[n_y_sonar]) * stddev)) {
            //PX4_INFO("sonar fault %5.2f | %5.2f | %5.2f | %5.2f", double(d), double(_est_distance), beta, (sqrt(BETA_TABLE[n_y_sonar]) * stddev));
            return -1;
        }
    }
    
    // return valid distance
    return d;
}

static int error_count = 0;

void ADCPublisher::sonarCorrect(float current_distance)
{
    int i;
    
    // measure
    float distance = sonarMeasure(current_distance);
    
    _est_buf[_est_count] = current_distance;
    _est_count ++;
    
    if(_est_count > 4){
        _est_count = 0;
    }
    
    // if distance is not valid -> return
    if (distance < 0) {
        
        if(error_count > 2)
        {
            float sum = 0;
            for(i=0; i<5; i++){
                sum += float(_est_buf[i]);
            }
            
            if(sum > 0) {
                _est_distance = sum / 5;
            }
            else { return; }
        }
        
        error_count++;
        return;
        
    }
    
    error_count = 0;
    
    // use moving average
    _maList[_maCount] = distance;
    _maCount ++;
    
    if(_maCount > _filter_length - 1){
        _maCount = 0;
    }
    
    float sum = 0;
    for(i=0; i<_filter_length; i++){
        sum += float(_maList[i]);
    }
    
    if(sum > 0) {
        distance = sum / _filter_length;
    }
    else { return; }
    
    // publish value
    px4::px4_collision collision_msg;
    collision_msg.data().timestamp = px4::get_time_micros();
    collision_msg.data().front = distance;
    
    _est_distance = distance;
    //PX4_INFO("publish %5.2f", double(_x(X_tz)));
    //PX4_INFO("publish2 %5.2f",double(y(0)));
    
    _collision_pub->publish(collision_msg);
    
}

void ADCPublisher::sonarCheckTimeout()
{
    if (_timeStamp - _time_last_sonar > SONAR_TIMEOUT) {
        if (_sonarInitialized) {
            _sonarInitialized = false;
            _sonarStats.reset();
            mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar timeout ");
        }
    }
}