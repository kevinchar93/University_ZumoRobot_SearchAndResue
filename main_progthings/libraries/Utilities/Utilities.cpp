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

#include "Utilities.h"
#include <math.h>

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

#define NUM_ERRORVALS 10

Utilities::Utilities()
{
}

float Utilities::wrapAngle(float angle)
{
    angle = fmod(angle + 180, 360);
    if (angle < 0) {
        angle += 360;
    }
    return (angle - 180);
}

float Utilities::wrapAngle360(float angle)
{
    angle = wrapAngle(angle);
    if (angle < 0) {
        angle += 360;
    }
    return angle;
}

float Utilities::saturate(float value, float min, float max)
{
    if (value < min) {
        value = min;
    }
    else if (value > max) {
        value = max;
    }
    return value;
}

bool Utilities::inRange(float value, float lowerBound, float upperBound)
{
    return ((value > lowerBound) && (value < upperBound));
}

int Utilities::clip(int a)
{
    if (a > 400) {
        a = 400;
    }
    if (a < -400) {
        a = -400;
    }
    return a;
}

float Utilities::clip(float a)
{
    if (a > 400) {
        a = 400;
    }
    if (a < -400) {
        a = -400;
    }
    return a;
}

int Utilities::min(int a, int b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

int Utilities::max(int a, int b)
{
    if (a > b) {
        return a;
    }
    else {
        return b;
    }
}

float Utilities::toDegrees(float angle)
{
    return float ((angle * 180.0f) / M_PI);
}
