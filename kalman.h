/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

/*
 *  modify by Regis for PIC24,2013/11/24
 *
 */

#ifndef _Kalman_h
#define _Kalman_h

#define PI          3.14159265
#define HALF_PI     1.57079
#define TWO_PI      6.283185
#define DEG_TO_RAD  0.01745329
#define RAD_TO_DEG  57.2957786

void Kalman_Init(int x);
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double Kalman_getAngle(int x, double newAngle, double newRate, double dt);
// Used to set angle, this should be set as the starting angle
void Kalman_setAngle(int x, double newAngle);
double Kalman_getRate(int x);
/* These are used to tune the Kalman filter */
void Kalman_setQangle(int x, double newQ_angle);
void Kalman_setQbias(int x, double newQ_bias);
void Kalman_setRmeasure(int x, double newR_measure);
double Kalman_getQangle(int x);
double Kalman_getQbias(int x);
double Kalman_getRmeasure(int x);

#endif
