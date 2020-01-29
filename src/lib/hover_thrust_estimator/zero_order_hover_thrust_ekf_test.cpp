/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * Test code for the Zero Order Hover Thrust Estimator
 * Run this test only using make tests TESTFILTER=zero_order_hover_thrust_ekf
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <random>

#include "zero_order_hover_thrust_ekf.hpp"

using namespace matrix;

class ZeroOrderHoverThrustEkfTest : public ::testing::Test
{
public:
	float computeAccelFromThrustAndHoverThrust(float thrust, float hover_thrust);
	ZeroOrderHoverThrustEkf::status runEkf(float accel, float thrust, float time);

	std::normal_distribution<float> standard_normal_distribution_;
	std::default_random_engine random_generator_;

private:
	ZeroOrderHoverThrustEkf _ekf{};
	static constexpr float _dt = 0.02f;
};

float ZeroOrderHoverThrustEkfTest::computeAccelFromThrustAndHoverThrust(float thrust, float hover_thrust)
{
	return CONSTANTS_ONE_G * thrust / hover_thrust - CONSTANTS_ONE_G;
}

ZeroOrderHoverThrustEkf::status ZeroOrderHoverThrustEkfTest::runEkf(float accel, float thrust, float time)
{
	ZeroOrderHoverThrustEkf::status status{};
	for (float t = 0.0; t <= time; t += _dt) {
		_ekf.predict(_dt);
		status = _ekf.fuseAccZ(accel, thrust);
	}
	return status;
}

TEST_F(ZeroOrderHoverThrustEkfTest, testStaticCase)
{
	// GIVEN: a vehicle at hover, (the estimator starting at the true value)
	const float thrust = 0.5f;
	const float hover_thrust_true = 0.5f;
	const float accel_meas = 0.f;

	// WHEN: we input noiseless data and run the filter
	ZeroOrderHoverThrustEkf::status status = runEkf(accel_meas, thrust, 1.f);

	// THEN: The estimate should not move and its variance decrease quickly
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 1e-4);
	EXPECT_NEAR(status.hover_thrust_var, 0.0, 1e-3);
	EXPECT_NEAR(status.accel_noise_var, 0.0, 1.0); // The noise learning is slow and takes more than 1s to go to zero
}

TEST_F(ZeroOrderHoverThrustEkfTest, testStaticConvergence)
{
	// GIVEN: a vehicle at hover, (the estimator starting at the true value)
	const float thrust = 0.72f;
	const float hover_thrust_true = 0.72f;
	const float accel_meas = computeAccelFromThrustAndHoverThrust(thrust, hover_thrust_true);

	// WHEN: we input noiseless data and run the filter
	ZeroOrderHoverThrustEkf::status status = runEkf(accel_meas, thrust, 2.f);
	//
	// THEN: The X and Y trajectories should have the same total time (= time sunchronized)
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 1e-4);
	EXPECT_NEAR(status.hover_thrust_var, 0.0, 1e-3);
	EXPECT_NEAR(status.accel_noise_var, 0.0, 1.0); // The noise learning is slow and takes more than 1s to go to zero
}

TEST_F(ZeroOrderHoverThrustEkfTest, testStaticConvergenceWithNoise)
{
	// GIVEN: a vehicle at hover, (the estimator starting at the true value)
	const float sigma_noise = 0.2f;
	const float thrust = 0.72f + sigma_noise * standard_normal_distribution_(random_generator_);
	const float hover_thrust_true = 0.72f;
	const float accel_meas = computeAccelFromThrustAndHoverThrust(thrust, hover_thrust_true);

	// WHEN: we input noiseless data and run the filter
	ZeroOrderHoverThrustEkf::status status = runEkf(accel_meas, thrust, 2.f);

	// THEN: The X and Y trajectories should have the same total time (= time sunchronized)
	EXPECT_NEAR(status.hover_thrust, hover_thrust_true, 1e-4);
	EXPECT_NEAR(status.hover_thrust_var, 0.0, 1e-3);
	EXPECT_NEAR(status.accel_noise_var, 0.0, 1.0); // The noise learning is slow and takes more than 1s to go to zero
}
