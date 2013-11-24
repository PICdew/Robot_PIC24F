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
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "led.h"

#include "i2c.h"
#include "i2c_func.h"
#include "lcd_i2c_1602.h"
#include "kalman.h"

// X, Y axis
#define KALMAN_AXIS 2

/* Kalman filter variables */
static double Q_angle[KALMAN_AXIS]; // Process noise variance for the accelerometer
static double Q_bias[KALMAN_AXIS]; // Process noise variance for the gyro bias
static double R_measure[KALMAN_AXIS]; // Measurement noise variance - this is actually the variance of the measurement noise

static double angle[KALMAN_AXIS]; // The angle calculated by the Kalman filter - part of the 2x1 state vector
static double bias[KALMAN_AXIS]; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
static double rate[KALMAN_AXIS]; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

static double P[KALMAN_AXIS][2][2]; // Error covariance matrix - This is a 2x2 matrix
static double K[KALMAN_AXIS][2]; // Kalman gain - This is a 2x1 vector
static double y[KALMAN_AXIS]; // Angle difference
static double S[KALMAN_AXIS]; // Estimate error

void Kalman_Init(int x)
{
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle[x] = 0.001;
    Q_bias[x] = 0.003;
    R_measure[x] = 0.03;

    angle[x] = 0; // Reset the angle
    bias[x] = 0; // Reset bias

    P[x][0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[x][0][1] = 0;
    P[x][1][0] = 0;
    P[x][1][1] = 0;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double Kalman_getAngle(int x, double newAngle, double newRate, double dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate[x] = newRate - bias[x];
    angle[x] += dt * rate[x];

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[x][0][0] += dt * (dt*P[x][1][1] - P[x][0][1] - P[x][1][0] + Q_angle[x]);
    P[x][0][1] -= dt * P[x][1][1];
    P[x][1][0] -= dt * P[x][1][1];
    P[x][1][1] += Q_bias[x] * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    S[x] = P[x][0][0] + R_measure[x];
    /* Step 5 */
    K[x][0] = P[x][0][0] / S[x];
    K[x][1] = P[x][1][0] / S[x];
    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    y[x] = newAngle - angle[x];
    /* Step 6 */
    angle[x] += K[x][0] * y[x];
    bias[x] += K[x][1] * y[x];

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    P[x][0][0] -= K[x][0] * P[x][0][0];
    P[x][0][1] -= K[x][0] * P[x][0][1];
    P[x][1][0] -= K[x][1] * P[x][0][0];
    P[x][1][1] -= K[x][1] * P[x][0][1];

    return angle[x];
}

void Kalman_setAngle(int x, double newAngle)
{
    angle[x] = newAngle;
}; // Used to set angle, this should be set as the starting angle

double Kalman_getRate(int x)
{
    return rate[x];
}; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman_setQangle(int x, double newQ_angle)
{
    Q_angle[x] = newQ_angle;
}

void Kalman_setQbias(int x, double newQ_bias)
{
    Q_bias[x] = newQ_bias;
}

void Kalman_setRmeasure(int x, double newR_measure)
{
    R_measure[x] = newR_measure;
}

double Kalman_getQangle(int x)
{
    return Q_angle[x];
}

double Kalman_getQbias(int x)
{
    return Q_bias[x];
}

double Kalman_getRmeasure(int x)
{
    return R_measure[x];
}


#if 0
#ifndef _Kalman_h
#define _Kalman_h

class Kalman {
public:
    Kalman() {
        /* We will set the variables like so, these can also be tuned by the user */
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;

        angle = 0; // Reset the angle
        bias = 0; // Reset bias

        P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 0;
    };
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    double getAngle(double newAngle, double newRate, double dt) {
        // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        // Modified by Kristian Lauszus
        // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        /* Step 1 */
        rate = newRate - bias;
        angle += dt * rate;

        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P[0][0] + R_measure;
        /* Step 5 */
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;
        /* Step 6 */
        angle += K[0] * y;
        bias += K[1] * y;

        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P[0][0] -= K[0] * P[0][0];
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];

        return angle;
    };
    void setAngle(double newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    double getRate() { return rate; }; // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(double newR_measure) { R_measure = newR_measure; };

    double getQangle() { return Q_angle; };
    double getQbias() { return Q_bias; };
    double getRmeasure() { return R_measure; };

private:
    /* Kalman filter variables */
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 vector
    double y; // Angle difference
    double S; // Estimate error
};

#endif
#endif
