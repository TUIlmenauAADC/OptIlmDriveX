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
#include "aadc_jury.h"
#include <a_utils/core/a_utils_core.h>

//*************************************************************************************************
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "OptIlm_DriverModule.filter.user.aadc.cid"




using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace aadc::jury;



class cOptIlm_DriverModule : public cTriggerFunction
{
private:

    //properties
    property_variable<tInt>    m_propTCPPort = 1234;
    property_variable<tBool> m_propEnableConsoleOutput = tFalse;




    //Media Descriptions
    struct tJuryStructId
    {
        tSize actionId;
        tSize maneuverEntry;
    } m_ddlJuryStructId;

    /*! The jury structure sample factory */
    cSampleCodecFactory m_juryStructSampleFactory;

    struct tDriverStructId
    {
        tSize stateId;
        tSize maneuverEntry;
    } m_ddlDriverStructId;

    /*! The driver structure sample factory */
    cSampleCodecFactory m_driverStructSampleFactory;

    /*! The maneuver file string */
    cString     m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    aadc::jury::maneuverList m_sectorList;

    /*! The mutex */
    std::mutex m_oMutex;
    /*! The server socket */
    cServerSocket m_serverSocket;
    /*! The stream socket */
    cStreamSocket m_streamSocket;
    /*! The client connection established */
    tBool m_clientConnectionEstablished;


    /*! A ddl Maneuver Listr data identifier. */
    struct
    {
        tSize maneuver;
    } m_ddlManeuverListDataId;
    adtf::mediadescription::cSampleCodecFactory m_ManeuverListSampleFactory;


    /*! Reader of an InPin. */
    cPinReader m_oDriverToJuly;

    /*! Writer to an OutPin. */
    cPinWriter m_oStateFromJuly;
    cPinWriter m_oSend_maneuver_listr;



public:

    /*! Default constructor. */
    cOptIlm_DriverModule();

    /*! Destructor. */
    virtual ~cOptIlm_DriverModule() = default;

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


    tTimeStamp m_lastSampleTime;
    tFloat64 f64SampleTime;
    MANEUVER_LIST ManeuverList;

    tResult SystemInitial();
    tResult ReceiveTCPData(std::vector<tChar>& data);
    tResult GetJulyOnTimer();
    tResult LoadManeuverList();
    tResult ConnectToClient();
    tResult OnSendState(int stateID, tInt16 i16ManeuverEntry);
    tResult TransmitManeuverListr();
    tResult TransmitJulyState(tInt16 i8ActionID , tInt16 i16entry);
    tResult GetAutonomusFilterJulyState();


};


//*************************************************************************************************
