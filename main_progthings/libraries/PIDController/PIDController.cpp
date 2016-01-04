/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "PIDController.h"

PIDController::PIDController(float p, float i, float d)
{
    m_P = p;
    m_I = i;
    m_D = d;

    sumError = 0;
    lastError = 0;
    lastTime = 0;
        
    proportionalTerm = 0;
    integralTerm = 0;
    derivativeTerm = 0;
}

float PIDController::calculate(float error)
{
    float output;

    /* compute and add the proportional term */
    proportionalTerm = m_P * error;
    output = proportionalTerm;

    /* compute and add the derivative term */
    unsigned long currTime = micros();
    float dt = (currTime - lastTime) / 1000.0f;
    derivativeTerm = m_D * ((error - lastError) / dt);
    output += derivativeTerm;
        
    /* compute and add the integral term */
    if ((error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
        sumError = 0; /* auto-zero past integration sum */
    }
    sumError += error * dt;
    integralTerm = sumError * m_I;
    output += integralTerm;

    /* update the previous values */
    lastError = error;
    lastTime = currTime;
        
    return (output);
}

float PIDController::getPContribution()
{
    return proportionalTerm; 
}

float PIDController::getIContribution()
{
    return integralTerm;
}

float PIDController::getDContribution()
{
    return derivativeTerm;
}
