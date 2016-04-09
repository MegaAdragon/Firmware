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
 * @file subscriber_example.cpp
 * Example subscriber for ros and px4
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "subscriber_params.h"
#include "subscriber_example.h"

using namespace px4;

void adc_sonar_callback_function(const px4_adc_sonar &msg)
{
	PX4_INFO("I heard: [%" PRIu64 "] + [%f]", msg.data().timestamp, double(msg.data().distance));
    
    /* TODO: Do something with the sonar value */
}

SubscriberExample::SubscriberExample() :
	_n(_appState),
	_p_sub_interv("SUB_INTERV", PARAM_SUB_INTERV_DEFAULT),
	_p_test_float("SUB_TESTF", PARAM_SUB_TESTF_DEFAULT)
{
	/* Read the parameter back as example */
	_p_sub_interv.update();
	_p_test_float.update();
	PX4_INFO("Param SUB_INTERV = %d", _p_sub_interv.get());
	PX4_INFO("Param SUB_TESTF = %.3f", (double)_p_test_float.get());

	/* Do some subscriptions */
	/* Function */
	_n.subscribe<px4_adc_sonar>(adc_sonar_callback_function, _p_sub_interv.get());

	PX4_INFO("subscribed");
}
