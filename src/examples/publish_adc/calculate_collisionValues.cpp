
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

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const int 		REQ_SONAR_INIT_COUNT = 10;
static const uint32_t 	SONAR_TIMEOUT =   1000000; // 1.0 s

void ADCPublisher::sonarInit()
{
    // measure
    Vector<float, n_y_sonar> y;
    
    if (sonarMeasure(y, 0) != OK) {
        _sonarStats.reset();
        return;
    }
    
    // if finished
    if (_sonarStats.getCount() > REQ_SONAR_INIT_COUNT) {
        /*
        mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar init "
                                     "mean %d cm std %d cm",
                                     int(100 * _sonarStats.getMean()(0)),
                                     int(100 * _sonarStats.getStdDev()(0)));
         */
        _sonarInitialized = true;

        _sonarFault = FAULT_NONE;
    }
}

int ADCPublisher::sonarMeasure(Vector<float, n_y_sonar> &y, float current_distance)
{
    // measure
    float d = current_distance;
    float eps = 0.01f;
    /*
    float min_dist = _sub_sonar->get().min_distance + eps;
    float max_dist = _sub_sonar->get().max_distance - eps;
     */
    
    // TODO: Fix this
    float min_dist = float(0.3) + eps;
    float max_dist = float(7.0) - eps;
    
    // check for bad data
    if (d > max_dist || d < min_dist) {
        return -1;
    }
    
    // update stats
    _sonarStats.update(Scalarf(d));
    _time_last_sonar = _timeStamp;
    y.setZero();
    
    //TODO: WHY??
    /*
    y(0) = d *
    cosf(_sub_att.get().roll) *
    cosf(_sub_att.get().pitch);
     */
    return OK;
}

void ADCPublisher::sonarCorrect(float current_distance)
{
    // measure
    Vector<float, n_y_sonar> y;
    
    if (sonarMeasure(y, current_distance) != OK) { return; }
    
    // do not use sonar if lidar is active
    //if (_lidarInitialized && (_lidarFault < fault_lvl_disable)) { return; }
    
    // calculate covariance
    //float cov = _sub_sonar->get().covariance;
    //TODO: Fix this
    float cov = 0.5;
    
    if (cov < 1.0e-3f) {
        // use sensor value if reasoanble
        cov = _sonar_z_stddev.get() * _sonar_z_stddev.get();
    }
    
    // sonar measurement matrix and noise matrix
    Matrix<float, n_y_sonar, n_x> C;
    C.setZero();
    // y = -(z - tz)
    // TODO could add trig to make this an EKF correction
    C(Y_sonar_z, X_z) = -1; // measured altitude, negative down dir.
    C(Y_sonar_z, X_tz) = 1; // measured altitude, negative down dir.
    
    // covariance matrix
    Matrix<float, n_y_sonar, n_y_sonar> R;
    R.setZero();
    R(0, 0) = cov;
    
    // residual
    Vector<float, n_y_sonar> r = y - C * _x;
    
    // residual covariance, (inverse)
    Matrix<float, n_y_sonar, n_y_sonar> S_I =
    inv<float, n_y_sonar>(C * __P * C.transpose() + R);
    
    // fault detection
    float beta = (r.transpose()  * (S_I * r))(0, 0);
    
    if (beta > BETA_TABLE[n_y_sonar]) {
        if (_sonarFault < FAULT_MINOR) {
            _sonarFault = FAULT_MINOR;
            //mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar fault,  beta %5.2f", double(beta));
        }
        
        // abort correction
        return;
        
    } else if (_sonarFault) {
        _sonarFault = FAULT_NONE;
        //mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] sonar OK");
    }
    
    // kalman filter correction if no fault
    if (_sonarFault < fault_lvl_disable) {
        Matrix<float, n_x, n_y_sonar> K =
        __P * C.transpose() * S_I;
        Vector<float, n_x> dx = K * r;
        
        // TODO: do I need this?
        /*
        if (!_canEstimateXY) {
            dx(X_x) = 0;
            dx(X_y) = 0;
            dx(X_vx) = 0;
            dx(X_vy) = 0;
        }
         */
        
        _x += dx;
        __P -= K * C * __P;
        
        px4::px4_collision collision_msg;
        collision_msg.data().timestamp = px4::get_time_micros();
        collision_msg.data().front = current_distance;
        
        _collision_pub->publish(collision_msg);
    }
    
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