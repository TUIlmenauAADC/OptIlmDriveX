/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once

#include "stdafx.h"
#include "aadc_structs.h"
#include "TU_Ilmenau_SOP.h"

//*************************************************************************************************
#define CID_OPTILM_AUTONOMOUS_DRIVING_DATA_TRIGGERED_FILTER "Optilm_MapVisualization.filter.user.aadc.cid"


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

typedef struct _POINT_STRUCT
{
    X_Y_POINT from;
    X_Y_POINT to;

}POINT_STRUCT;

typedef struct _DATA_STRUCT
{
    POINT_STRUCT down;
    POINT_STRUCT center;
    POINT_STRUCT top;

}DATA_STRUCT;

typedef struct _KURVE_DATA_STRUCT
{
    X_Y_POINT center;
    int radius;
    int star_angle;
    int end_angle;

}KURVE_DATA_STRUCT;

typedef struct _KURVE_STRUCT
{
    KURVE_DATA_STRUCT down;
    KURVE_DATA_STRUCT center;
    KURVE_DATA_STRUCT top;

}KURVE_STRUCT;

typedef struct _ROAD_SIGN
{
    int id;
    int direction;
    X_Y_POINT position;
}ROAD_SIGN;

/*! the main class of the open cv template. */
class cOptilm_MapVisualization : public cTriggerFunction
{
private:




    //Pins
    /*! Writer to an OutPin. Video*/
    cPinWriter m_oVideo_Output;

    //Stream Formats
        /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;


    /*! Media Descriptions. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;


    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;



    /*! Reader of an InPin. Info */
    cPinReader m_oInput_Position;
    cPinReader m_oInput_Position2;




    tTimeStamp m_lastSampleTime;
    tFloat64 f64SampleTime;


public:

    /*! Default constructor. */
    cOptilm_MapVisualization();


    /*! Destructor. */
    virtual ~cOptilm_MapVisualization() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;


    //Get pin Data
    tResult GetPosition();
    tResult GetPosition2();
    tResult GetCarSpeed();


    //Send pin data
    tResult TransmitDataFusionVideo(void);

    tResult SystemInitial();


public:
 
    float position_data[5];
    float position_data2[5];
    bool position_initial_flag;
    bool position_initial_flag2;






};


//*************************************************************************************************
