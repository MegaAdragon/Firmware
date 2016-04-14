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
 * @file subscribe_sonar_start_nuttx.cpp
 *
 * @author Dominik Zipperle
 */
#include <string.h>
#include <cstdlib>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include "subscribe_sonar.hpp"

static volatile bool thread_should_exit = false;     /**< Deamon exit flag */
static volatile bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int subscribe_sonar_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int subscribe_sonar_thread_main(int argc, char *argv[]);

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int subscribe_sonar_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: subscribe_sonar {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;

		deamon_task = px4_task_spawn_cmd("subscribe_sonar",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 subscribe_sonar_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);

        return 0;
	}
    
    if (!strcmp(argv[1], "stop")) {
        if (thread_running) {
            warnx("stop");
            thread_should_exit = true;
            
        } else {
            warnx("not started");
        }
        
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("is running");
            
        } else {
            warnx("not started");
        }
        
        return 0;
    }

	warnx("unrecognized command");
	return 1;
}

int subscribe_sonar_thread_main(int argc, char **argv)
{
    px4::init(argc, argv, "subscribe_sonar");
    
    PX4_INFO("starting");
    SubscribeSonar s;
    thread_running = true;
    
    // keeps calling callbacks for incoming messages, returns when module is terminated
    s.spin();
    
    //TODO: thread stop has no effect
    PX4_INFO("exiting.");
    
    thread_running = false;
    
    return 0;
}
