/*********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-11 14:55:07#$ $Rev:: 63067   $
**********************************************************************/
#pragma once

#pragma pack(push,1)
typedef struct
{
    int count_right;
    int dir_right;
    int count_left;
    int dir_left;
}tWheelDataSimple;
#pragma pack(pop)

#pragma pack(push,1)
/*! A battery data struct for holding the values */
typedef struct 
{
    int actuator_overall;
    int actuator_cell1  ;
    int actuator_cell2  ;
    int sensors_overall ;
    int sensors_cell1   ;
    int sensors_cell2   ;
    int sensors_cell3   ;
    int sensors_cell4   ;
    int sensors_cell5   ;
    int sensors_cell6   ;
}tVoltageStructSimple;
#pragma pack(pop)

#pragma pack(push,1)
/*! A data struct for ultrasonic sensor values */
typedef struct 
{
    int us_side_left;
    int us_side_right;
    int us_rear_left;
    int us_rear_center;
    int us_rear_right;
}tUltrasonicStructSimple;
#pragma pack(pop)