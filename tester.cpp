/*
 *     tester.cpp
 *
 *     This file runs the IKSolver on a simple two-joint parallel linkage leg.
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
#include "Vector.h"

#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>

using namespace std;

// Forward declarations
Vector2d forwardSolve(float * angles);
Vector2d targPos(10,-40);

//void i_forwardSolve(int_fast32_t * angles, vector_int_2d_t& ret);
//vector_int_2d_t i_targPos = {1000, -500};

float * IK_LUT;

//#define X_LOWER_BOUND -15.0f
//#define X_UPPER_BOUND 51.7f
//#define Y_LOWER_BOUND -82.2f
//#define Y_UPPER_BOUND -27.1f
//#define X_STEP 1.0f
//#define Y_STEP 1.0f

#define X_LOWER_BOUND (-15.0f)
#define X_UPPER_BOUND (55.0f)
#define Y_LOWER_BOUND (-85.0f)
#define Y_UPPER_BOUND (-25.0f)
#define X_STEP (2.0f)
#define Y_STEP (2.0f)

#define X_STEPS (int)(abs((X_UPPER_BOUND-X_LOWER_BOUND)/X_STEP))
#define Y_STEPS (int)(abs((Y_UPPER_BOUND-Y_LOWER_BOUND)/Y_STEP))

// Leg parameters
float angles[2] = {0};
const float radii[2] = {30, 30};
const float offsets[2] = {-2.747, -24.96};

// Upper joint: +25.54deg to -62.51deg
// Lower joint: -62.51deg to +58.35deg
const float ranges[2][2] = {{-60.0f/180.0f*PI, 25.0f/180.0f*PI},{-55.0f/180.0f*PI, 55.0f/180.0f*PI}};

//int_fast32_t i_radii[2] = {1000, 1000};
//int_fast32_t i_angles[2] = {0};

const float PROGRESS_STEP = 0.05f;

int main()
{
    // Solve the angles
    cout<<"Lookup table generation progress: ";

    IK_LUT = new float[X_STEPS*Y_STEPS*2];

    cout << X_STEPS << " " << Y_STEPS << endl;

    int solve_res;
    float prev_progress = 0.0f;
    float progress = 0.0f;

    for(int j = 0; j < Y_STEPS; j++)
    {
        for(int i = 0; i < X_STEPS; i++)
        {
            //printf("The angles for E.E. position <%f, %f> ", X_LOWER_BOUND+(i*X_STEP), Y_LOWER_BOUND+(j*Y_STEP));
            solve_res = solve(angles, ranges, radii, 2, forwardSolve, Vector2d(X_LOWER_BOUND+(i*X_STEP), Y_LOWER_BOUND+(j*Y_STEP)), 1.0f);
            if(solve_res != -1)
            {
                cout<<solve_res<<"\t";
                //printf("are (%f, %f)\r\n", RAD2DEG(angles[0]), RAD2DEG(angles[1]));
                IK_LUT[j*X_STEPS+i] = angles[0];
                IK_LUT[j*X_STEPS+i+1] = angles[1];
            }
            else
            {
                //printf("cannot be computed.");
                IK_LUT[j*X_STEPS+i] = 2.215f;
                IK_LUT[j*X_STEPS+i+1] = 2.215f;
            }
            progress = (((float)(j*X_STEPS+i))/((float)(Y_STEPS*X_STEPS)));
            if(progress > (prev_progress+PROGRESS_STEP))
            {
                cout<<(int)(progress*100)<<"%"<<endl;
                prev_progress = progress;
            }

        }
    }
    ofstream LUTFile;
    LUTFile.open("LegLUT.h");
    LUTFile << "#ifndef _LEG_LUT_H_"<<endl
    <<"#define _LEG_LUT_H_"<<endl
    <<"extern const int16_t legPosLUT ["<<Y_STEPS<<"]["<<X_STEPS<<"]["<<2<<"];"<<endl
    <<"#endif"<<endl;
    LUTFile.close();
    LUTFile.open("LegLUT.c");

    LUTFile<<"#include \"LegLUT.h\""<<endl<<"const int16_t legPosLUT ["<<2<<"]["<<X_STEPS<<"]["<<Y_STEPS<<"] = {"<<endl;
    for(int j = 0; j < Y_STEPS; j++)
    {
        LUTFile<<"{";
        for(int i = 0; i < X_STEPS; i++)
        {
            LUTFile<<"{"<<(int16_t)(IK_LUT[j*X_STEPS+i]*180/PI)<<", "<<(int16_t)(IK_LUT[j*X_STEPS+i+1]*180/PI)<<"}";
            if(i != X_STEPS-1)
            {
                LUTFile<<", ";
            }
        }
        LUTFile<<"}";
        if(j != Y_STEPS-1)
        {
            LUTFile<<", ";
        }
        LUTFile<<endl;
    }
    LUTFile<<"};"<<endl;

    LUTFile.close();

    LUTFile.open("graph.txt");

    for(int j = Y_STEPS-1; j >= 0; j--)
    {
        for(int i = 0; i < X_STEPS; i++)
        {
            if(((int16_t)(IK_LUT[j*X_STEPS+i]*180/PI) == 126) ||
            ((int16_t)(IK_LUT[j*X_STEPS+i+1]*180/PI) == 126))
            {
                LUTFile<<"  ";
            }else
            {
                LUTFile<<"XX";
            }
        }
        LUTFile<<endl;
    }

    LUTFile.close();

    delete IK_LUT;

    // Solve using integers
    //cout << "Solve using integer arithmetic:" << endl;
    //cout << solve(i_angles, i_radii, 2, i_forwardSolve, i_targPos, 100) << endl;

    //cout << i_angles[0] << ", " << i_angles[1] << endl;
    //vector_int_2d_t i_result;
    //i_forwardSolve(i_angles, i_result);
    //cout << '<' << i_result.x << ", " << i_result.y << ">" << endl;
    return 0;
}

// Function for computing the end effector position from the angles of the joints
Vector2d forwardSolve(float * angles)
{
    Vector2d ret;
    ret.x = (radii[0]*cos(angles[0]))+(radii[1]*sin(angles[1]))+offsets[0];
    ret.y = (radii[0]*sin(angles[0]))-(radii[1]*cos(angles[1]))+offsets[1];
    return ret;
}

//void i_forwardSolve(int_fast32_t * angles, vector_int_2d_t& ret)
//{
//    ret.x = i_radii[0]*cosLUT(angles[0])+i_radii[1]*sinLUT(angles[1]);
//    ret.y = -i_radii[0]*sinLUT(angles[0])-i_radii[1]*cosLUT(angles[1]);
//}
