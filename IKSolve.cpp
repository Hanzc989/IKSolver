/*
 *     IKSolve.cpp
 *
 *     This file implements the IK solver.
 *
 *     Copyright (C) 2013-2014  Kevin Balke
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; either version 2 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License along
 *     with this program; if not, write to the Free Software Foundation, Inc.,
 *     51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "IKSolve.h"
#include <iostream>

using namespace std;

//int16_t sin_lookup_table[] = {
//    +0,  -1607,  -3211,  -4808,  -6392,  -7961,  -9512, -11039,
//-12539, -14010, -15446, -16846, -18204, -19519, -20787, -22005,
//-23170, -24279, -25330, -26319, -27245, -28106, -28898, -29621,
//-30273, -30852, -31357, -31785, -32138, -32413, -32610, -32728,
//-32768, -32728, -32610, -32413, -32138, -31785, -31357, -30852,
//-30273, -29621, -28898, -28106, -27245, -26319, -25330, -24279,
//-23170, -22005, -20787, -19519, -18204, -16846, -15446, -14010,
//-12539, -11039,  -9512,  -7961,  -6392,  -4808,  -3211,  -1607,
//    +0,  +1607,  +3211,  +4808,  +6392,  +7961,  +9512, +11039,
//+12539, +14010, +15446, +16846, +18204, +19519, +20787, +22005,
//+23170, +24279, +25330, +26319, +27245, +28106, +28898, +29621,
//+30273, +30852, +31357, +31785, +32138, +32413, +32610, +32728,
//+32767, +32728, +32610, +32413, +32138, +31785, +31357, +30852,
//+30273, +29621, +28898, +28106, +27245, +26319, +25330, +24279,
//+23170, +22005, +20787, +19519, +18204, +16846, +15446, +14010,
//+12539, +11039,  +9512,  +7961,  +6392,  +4808,  +3211, 1607
//};

#define LERP_STEP 0.001f
#define TIMEOUT 5000

int solve(float* angles, const float ranges[][2], const float* radii, const int& nJoints, Vector2d (*forwardSolve)(float*), const Vector2d& target, const float& reqError)
{
    int solve_counter = 0;
    Vector2d startPos = forwardSolve(angles);
    Vector2d curPos, step, testPos;
    int i = 0;
    float dTheta = 0;
    float error = 0;
    float prevAngle = 0;
    for(int j = 0; j < nJoints; j++)
    {
        angles[j] = 0.0f;//(ranges[i][0] + ranges[i][1])/2.0f;
    }
    for(float lerp_pos = LERP_STEP; lerp_pos <= 1.0f; lerp_pos+=LERP_STEP)
    {
        do
        {
            if(solve_counter >= TIMEOUT)
            {
                //cout << "<" << target.x << ", " << target.y << ">\t";
                break;
            }
            // Forward solve for the current end effector position
            curPos = forwardSolve(angles);
            solve_counter++;
            // Calculate the next small step to the target position
            step = startPos.lerp(target, lerp_pos);
            // Compute the error (distance between current end effector position and
            // the step position)
            error = (curPos-step).magnitude();

            // Compute the dtheta
            dTheta = error/radii[i];

            // Test dtheta in the positive direction
            prevAngle = angles[i];

            angles[i] = prevAngle + dTheta;
            if(angles[i] > ranges[i][1])
            {
                angles[i] = ranges[i][1];
            }

            // Forward solve again
            testPos = forwardSolve(angles);
            solve_counter++;
            // Did the +dtheta result in a position closer to the target?
            if((testPos-step).magnitude() < error)
            {
                // Yes, proceed to next loop iteration
                continue;
            }
            // No, try the other direction
            else
            {
                // Test -dtheta
                angles[i] = prevAngle - dTheta;
                if(angles[i] < ranges[i][0])
                {
                    angles[i] = ranges[i][0];
                }
                // Forward solve again
                testPos = forwardSolve(angles);
                solve_counter++;
                if((testPos-step).magnitude() < error)
                {
                    continue;
                }
            }
            angles[i] = prevAngle;


            i++;
            if(i == nJoints)
            {
                i = 0;
            }


        }
        while(error > reqError);
        if(solve_counter >= TIMEOUT)
        {
            break;

        }
    }
    return (solve_counter >= TIMEOUT)?(-1):(solve_counter);
}

//#define I_LERP_STEPS        (8)
//#define I_LERP_T_MAX        (128)
//#define I_LERP_STEP         (I_LERP_T_MAX/I_LERP_STEPS)
//
//template<class A, class B, class T>
//inline int_fast32_t i_lerp(A a, B b, T t)
//{
//    //return ((b-a)*((8<<sizeof(t))-1))/t;
//    return a + (((b-a)*t)/I_LERP_T_MAX);
//}
//
//
//
//int_fast32_t solve(int_fast32_t* angles,
//          const int_fast32_t* radii,
//          const int_fast8_t& nJoints,
//          void (*forwardSolve)(int_fast32_t*, vector_int_2d_t&),
//          vector_int_2d_t target,
//          const int_fast32_t& reqError)
//{
//    int_fast32_t solve_counter = 0;
//    vector_int_2d_t startPos, step, testPos;
//    forwardSolve(angles, startPos);
//
//    // Premultiply to match magnitude returned by forwardSolve
//    target.x *= TrigLUTMagnitude;
//    target.y *= TrigLUTMagnitude;
//
//    int i = 0;
//    int_fast32_t error, prevAngle;
//    // Fix for loop breaking out after lerp_pos = 112
//    for(int8_t lerp_pos = I_LERP_STEP; lerp_pos <= I_LERP_T_MAX; lerp_pos+=I_LERP_STEP)
//    {
//        // Calculate the next small step to the target position
//        step.x = i_lerp<int_fast32_t, int_fast32_t, int8_t>(startPos.x, target.x, lerp_pos);
//        step.y = i_lerp<int_fast32_t, int_fast32_t, int8_t>(startPos.y, target.y, lerp_pos);
//        do
//        {
//            // Forward solve for the current end effector position
//            forwardSolve(angles, testPos);
//            solve_counter++;
//
//            // Compute the error (distance between current end effector position and
//            // the step position)
//            error = abs(step.x-testPos.x) + abs(step.y-testPos.y);
//
//            // Compute the dtheta
//            //dTheta = error/radii[i];
//
//            // Test dtheta in the positive direction
//            prevAngle = angles[i];
//            angles[i] = (prevAngle*radii[i]*TrigLUTMagnitude + (error*TrigLUTTwoPiRatio))/(radii[i]*TrigLUTMagnitude);
//
//            // Forward solve again
//            forwardSolve(angles, testPos);
//            solve_counter++;
//            // Did the +dtheta result in a position closer to the target?
//            if((abs(step.x-testPos.x) + abs(step.y-testPos.y)) < error)
//            {
//                // Yes, proceed to next loop iteration
//                continue;
//            }
//            // No, try the other direction
//            else
//            {
//                // Test -dtheta (need to subtract 2*dtheta to cancel the +dtheta from before)
//                angles[i] = (prevAngle*radii[i]*TrigLUTMagnitude - (error*TrigLUTTwoPiRatio))/(radii[i]*TrigLUTMagnitude);
//                // Forward solve again
//                forwardSolve(angles, testPos);
//                solve_counter++;
//                if((abs(step.x-testPos.x) + abs(step.y-testPos.y)) < error)
//                {
//                    continue;
//                }
//            }
//            angles[i] = prevAngle;
//
//
//            i++;
//            if(i == nJoints)
//            {
//                i = 0;
//            }
//        }
//        while(error > reqError);
//    }
//    return solve_counter;
//}
