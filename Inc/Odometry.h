/*
 * Odometry.h
 *
 *  Created on: Feb 14, 2018
 *      Author: yusaku
 */
#pragma once

#include <ICM20602.h>
#include <cmath>

#define SPI_ICM20602  SPI2
#define GPIO_ICM20602 GPIOB
#define PIN_ICM20602  GPIO_PIN_12

class Odometry
{
private:
	float x;
	float y;
	float yaw;

	ICM20602 *icm20602 = nullptr;

	// diameter of wheels in metre
	static constexpr float WheelDiameter = 0.0508; //タイヤの直径によって変更
	// pulse/rev
	static constexpr float PulsePerRevolution = 500.0 * 4; //パルス
	/// Kpd = 2_pi_r[mm/rev] / Kp[pulse/rev]
	static constexpr float MPerPulse = M_PI * WheelDiameter / PulsePerRevolution;

	// milli degree per sec // milli G per sec //Gxyz,Axyz
//	int movavg[6];
//	int biased[6];

	void GetBias(float * const avg, float * const stdev) const;
	bool InitGyro(void);

	void ReadEncoder(void);
	void ReadAccGyro(void);

public:
	Odometry(void);

	bool Initialize(void);

	int raw[6]; //後で戻す
	int movavg[6];
	int biased[6];

	void Sample(void);
	void SetPose(float x, float y, float yaw);
	void GetPose(float *x, float *y, float *yaw);

	static constexpr int32_t SamplingFrequency = 1000; //TIM2の割り込み周波数と一致
	//正直madgwickfilterがそんなに周波数出るかわからん
};



















