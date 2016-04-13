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
 * @file publish_adc.h
 * header file for publish_adc
 *
 * @author Dominik Zipperle
 */
#pragma once
#include <px4.h>
#include <px4_app.h>

#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>
#include <matrix/Matrix.hpp>

using namespace matrix;
using namespace control;



// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
    8.82050518214,
    12.094592431,
    13.9876612368,
    16.0875642296,
    17.8797700658,
    19.6465647819,
};

int pub_main(int argc, char **argv);

class ADCPublisher : public control::SuperBlock
{
public:
    enum {Y_sonar_z = 0, n_y_sonar};
	ADCPublisher();

	~ADCPublisher() {};

	int main();

	static px4::AppState appState;
protected:
	px4::NodeHandle _n;
    px4::Publisher<px4::px4_adc_sonar> *_adc_sonar_pub;
    px4::Publisher<px4::px4_collision> *_collision_pub;
    
private:
    
    // sonar
    float sonarMeasure(float current_distance);
    void sonarCorrect(float current_distance);
    void sonarInit(float current_distance);
    void sonarCheckTimeout();
    
    // sonar parameters
    BlockParamFloat  _sonar_z_stddev;
    BlockParamFloat  _sonar_z_offset;
    
    BlockStats<float, n_y_sonar> _sonarStats;
    
    uint64_t _timeStamp;
    uint64_t _time_last_sonar;
    
    bool _sonarInitialized;
    
    static const int FILTER_LENGTH = 10;
    
    int _maCount;
    float _maList[FILTER_LENGTH];
    
    float _est_distance;
    
};
