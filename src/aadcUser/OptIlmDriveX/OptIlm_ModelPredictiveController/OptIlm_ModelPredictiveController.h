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
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "OptIlm_ModelPredictiveController.filter.user.aadc.cid"



using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;





class cOptIlm_ModelPredictiveController : public cTriggerFunction
{
private:


    /*! Media Descriptions. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;


    /*! A ddl laser scanner data identifier. */
    struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;
    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;


    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;


    /*! The reader position */
    cPinReader m_oReaderPos;
    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    /*! A ddl laser scanner data identifier. */
    struct
    {
        tSize pointArray;
    } m_ddlTrajectoryPointDataId;
    adtf::mediadescription::cSampleCodecFactory m_TrajectoryPointSampleFactory;



    /*! Reader of an InPin. */
    cPinReader m_oDrivingModel;
    cPinReader m_oVehicleSpeed;
    cPinReader m_oInputLaserScanner;
    cPinReader m_oInput_trajectory_point;


    /*! Writer to an OutPin. */
    cPinWriter m_oOutputSpeed;
    cPinWriter m_oOutputSteering;
    cPinWriter m_oOutputEKFPosition;


    tSignalValue EmergencyBreakFlag;

//    FILE *log_file; // debug file

public:

    /*! Default constructor. */
    cOptIlm_ModelPredictiveController();

    /*! Destructor. */
    virtual ~cOptIlm_ModelPredictiveController() = default;

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



    /*!
     * Gets signal value from reader.
     *
     * \param [in,out]  reader      The reader.
     * \param [in,out]  signalValue The signal value.
     *
     * \return  Standard Result Code.
     */
    tResult GetSignalValueFromReader(sample_reader<size_limited_sample_reader_queue<1>>& reader, tFloat32& signalValue) const;

    tResult SendControlSignal(float speed, float steering);
    tResult TransmitSpeed(tSignalValue speed_signal);
    tResult TransmitSteering(tSignalValue steering_signal);
    tResult TransmitEKFPosition();
    bool Initial(bool flag);
    void FreeMem(bool flag);

    tTimeStamp m_lastSampleTime;
    tFloat64 f64SampleTime;
    tTimeStamp GetTime();

    //get data
    tResult GetDrivingModel(void);
    tResult GetVehicleSpeed(void);
    tResult GetPosition(void);
    tResult GetTrajectoryPoint(void);

    /*! The mutex */
    std::mutex m_oMutex;

};


//*************************************************************************************************
