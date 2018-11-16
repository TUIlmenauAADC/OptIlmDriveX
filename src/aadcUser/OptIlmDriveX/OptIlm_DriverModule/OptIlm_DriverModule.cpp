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


#include "OptIlm_DriverModule.h"

bool system_initial = false;


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "OptIlm July Driver Module",
    cOptIlm_DriverModule,
    adtf::filter::thread_trigger(true))


cOptIlm_DriverModule::cOptIlm_DriverModule()
{
    SetName("July Driver Module");
    RegisterPropertyVariable("Jury Module Port number", m_propTCPPort);
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    object_ptr<IStreamType> pTypeJuryStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tJuryStruct", pTypeJuryStruct, m_juryStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_ERROR("No mediadescription for tJuryStruct found!");
    }
    Register(m_oStateFromJuly, "StateFromJuly" , pTypeJuryStruct);


    object_ptr<IStreamType> pTypeDriverStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct", pTypeDriverStruct, m_driverStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_ERROR("No mediadescription for tJuryStruct found!");
    }
    Register(m_oDriverToJuly, "StateToJuly" , pTypeDriverStruct);

    //get the media description
    object_ptr<IStreamType> pTypeManeuverListr;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tManeuverList", pTypeManeuverListr, m_ManeuverListSampleFactory))
    {
        (adtf_ddl::access_element::find_array_index(m_ManeuverListSampleFactory, "i16Maneuver",   m_ddlManeuverListDataId.maneuver));
    }
    else
    {
         LOG_ERROR("Could not load media description for output pin Maneuver Listr");
    }

    //register output pose pin
    Register(m_oSend_maneuver_listr, "ManeuverListrFromJuly", pTypeManeuverListr);


    m_lastSampleTime = 0;
}


//implement the Configure function to read ALL Properties
tResult cOptIlm_DriverModule::Configure()
{

    RETURN_IF_FAILED(cTriggerFunction::Configure());

    RETURN_NOERROR;
}

tResult cOptIlm_DriverModule::Process(tTimeStamp tmTimeOfTrigger)
{
    tTimeStamp m_NowSampleTime =  tmTimeOfTrigger;
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;

    if(f64SampleTime > 0.5)
    {
        m_lastSampleTime = m_NowSampleTime;
//        LOG_INFO(cString::Format("Driver Module Sample Time = %.3f S", f64SampleTime));

        if(!system_initial)
        {
            SystemInitial();
        }
        else
        {
            if(m_clientConnectionEstablished == tFalse)
                ConnectToClient();
            else
            {
                GetJulyOnTimer();
                GetAutonomusFilterJulyState();
            }
        }
    }
    
    RETURN_NOERROR;
}

tResult cOptIlm_DriverModule::SystemInitial()
{
    RETURN_IF_FAILED_DESC(m_serverSocket.Open(m_propTCPPort, cServerSocket::SS_Exclusive),
        cString::Format("Could not open server socket with port %d", static_cast<tInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket was opened with port %d", static_cast<tInt>(m_propTCPPort)));
    RETURN_IF_FAILED_DESC(m_serverSocket.Listen(),
        cString::Format("Could not listen to port %d",static_cast<tInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket now listens on port %d", static_cast<tInt>(m_propTCPPort)));
    if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(5e5)))
    {
        RETURN_IF_FAILED_DESC(m_serverSocket.Accept(m_streamSocket),"Could not access Server socket");
        m_clientConnectionEstablished = tTrue;
        LOG_INFO("TCP Connection was established");
    }
    else
    {
        LOG_ERROR(cString::Format("No client is connected on Port %d", static_cast<tInt>(m_propTCPPort)));
        m_clientConnectionEstablished = tFalse;
    }

   system_initial = true;
   RETURN_NOERROR;
}


tResult cOptIlm_DriverModule::ConnectToClient()
{
    // no stream connected yet
    if (!m_clientConnectionEstablished)
    {
        // try to connect to client
        if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(2e2)))
        {
            RETURN_IF_FAILED(m_serverSocket.Accept(m_streamSocket));
            LOG_INFO("TCP Connection was established");
            m_clientConnectionEstablished = tTrue;
            OnSendState(statecar_startup, 0);
        }
    }

    RETURN_NOERROR;
}

tResult cOptIlm_DriverModule::ReceiveTCPData(std::vector<tChar>& data)
{
    // no stream connected yet
    if (!m_clientConnectionEstablished)
    {
        // try to connect to client
        if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(2e2)))
        {
            RETURN_IF_FAILED(m_serverSocket.Accept(m_streamSocket));
            LOG_INFO("TCP Connection was established");
            m_clientConnectionEstablished = tTrue;
        }
    }
    else
    {
        if (m_streamSocket.DataAvailable())
        {
            cString strBuffer;
            const tSize bufferSize = 65536;
            tInt bytesRead = 0;
            //make some space for data
            strBuffer.SetBuffer(bufferSize);
            // if read ok
            tResult res = m_streamSocket.Read((void*) strBuffer.GetPtr(), bufferSize, &bytesRead);
            if (IS_OK(res))
            {
                LOG_INFO(cString::Format("Received from client: %s", strBuffer.GetPtr()));
                data.clear();
                data.resize(bytesRead);
                memcpy(data.data(), strBuffer.GetPtr(), bytesRead);
            }
            else
            {
                LOG_INFO("TCP Connection was disconnected");
                m_clientConnectionEstablished = tFalse;
            }
        }
        else
        {
            RETURN_ERROR(ERR_NOT_READY);
        }
    }
    RETURN_NOERROR;
}

tResult cOptIlm_DriverModule::GetJulyOnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    std::vector<tChar> data;
    RETURN_IF_FAILED(ReceiveTCPData(data));
    const tSize sizeOfJuryStruct = sizeof(tJuryStruct);
    if (data.size() == sizeOfJuryStruct)
    { //jurysruct

        tJuryStruct* juryStruct = (tJuryStruct*) data.data();
        tInt16 i8ActionID = juryStruct->i16ActionID;
        tInt16 i16entry = juryStruct->i16ManeuverEntry;


        switch (aadc::jury::juryAction(i8ActionID))
        {
            case action_getready:
                LOG_INFO(cString::Format("Driver Module: Received Request Ready with maneuver ID %d", i16entry));
                break;
            case action_start:
                LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d", i16entry));
                break;
            case action_stop:
                LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d", i16entry));
                break;
        }
        TransmitJulyState(i8ActionID , i16entry);

    }
    else if (data.size() > 0)
    {//maneuverlist
        m_strManeuverFileString.Set(data.data(),data.size());
        LoadManeuverList();
        TransmitManeuverListr();
    }


    RETURN_NOERROR;
}

tResult cOptIlm_DriverModule::LoadManeuverList()
{
    m_sectorList.clear();
    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    ManeuverList.number_of_id = 0;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for (cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action =  maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    sector.sector.push_back(man);
                    m_sectorList.push_back(sector);

                    switch (man.action)
                    {
                        case maneuver_left:
                            ManeuverList.action[ManeuverList.number_of_id][0] = TURN_LEFT;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            break;
                        case maneuver_right:
                            ManeuverList.action[ManeuverList.number_of_id][0] = TURN_RIGHT;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            break;
                        case maneuver_straight:
                            ManeuverList.action[ManeuverList.number_of_id][0] = STRAIGHT;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            break;
                        case maneuver_pull_out_left:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PULL_OUT_LEFT;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            break;
                        case maneuver_pull_out_right:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PULL_OUT_RIGHT;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            break;
                        case maneuver_merge_left:
                            ManeuverList.action[ManeuverList.number_of_id][0] = MERGE_LEFT;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            break;
                        case maneuver_cross_parking_1:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 1;
                            break;
                        case maneuver_cross_parking_2:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 2;
                            break;
                        case maneuver_cross_parking_3:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 3;
                            break;
                        case maneuver_cross_parking_4:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 4;
                            break;
                        case maneuver_cross_parking_5:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 5;
                            break;
                        case maneuver_cross_parking_6:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 6;
                            break;
                        case maneuver_cross_parking_7:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 7;
                            break;
                        case maneuver_cross_parking_8:
                            ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 8;
                            break;
                        default:
                            ManeuverList.action[ManeuverList.number_of_id][0] = CAR_STOP;
                            ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                            LOG_ERROR(cString::Format("Maneuver List %d Error!!!", ManeuverList.number_of_id));
                            break;
                    }
                    ManeuverList.number_of_id++;
                }
            }
        }
    }


    if (ManeuverList.number_of_id > 0)
    {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
//        for(int index = 0; index < ManeuverList.number_of_id; index++)
//        {
//            switch (ManeuverList.action[index][0])
//            {
//                case CAR_STOP:
//                    LOG_ERROR(adtf_util::cString::Format("Maneuver id: %d ERROR!!!!!", index));
//                    break;
//                case PULL_OUT_LEFT:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d pull_out_left", index));
//                    break;
//                case PULL_OUT_RIGHT:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d pull_out_right", index));
//                    break;
//                case TURN_LEFT:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d left", index));
//                    break;
//                case TURN_RIGHT:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d right", index));
//                    break;
//                case STRAIGHT:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d straight", index));
//                    break;
//                case MERGE_LEFT:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d merge_left", index));
//                    break;
//                case PARKING:
//                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d cross_parking %d", index, ManeuverList.action[index][1]));
//                    break;
//            }
//        }
//        LOG_INFO(cString::Format("Maneuver End ID: %d action: Car Stop", ManeuverList.number_of_id));

    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
    }

    RETURN_NOERROR;
}

tResult cOptIlm_DriverModule::GetAutonomusFilterJulyState()
{
    tDriverStruct driverStruct;

    object_ptr<const ISample> pReadSample;
    while (IS_OK(m_oDriverToJuly.GetNextSample(pReadSample)))
    {
        auto oDecoder = m_driverStructSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.stateId, &driverStruct.i16StateID));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.maneuverEntry, &driverStruct.i16ManeuverEntry));

        OnSendState(driverStruct.i16StateID, driverStruct.i16ManeuverEntry);
    }

    RETURN_NOERROR;
}


tResult cOptIlm_DriverModule::OnSendState(int stateID, tInt16 i16ManeuverEntry)
{

    tDriverStruct driverStruct;
    driverStruct.i16StateID = stateID;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;

    if (m_propEnableConsoleOutput)
    {
        switch (stateID)
        {
            case statecar_ready:
                LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_running:
                LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_complete:
                LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_error:
                LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d", i16ManeuverEntry));
                break;
            case statecar_startup:
                LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d", i16ManeuverEntry));
                break;
        }
    }
    if (m_clientConnectionEstablished)
    {
        RETURN_IF_FAILED(m_streamSocket.Write(&driverStruct, sizeof(tDriverStruct)));
    }
    RETURN_NOERROR;
}



tResult cOptIlm_DriverModule::TransmitJulyState(tInt16 i8ActionID , tInt16 i16entry)
{
    object_ptr<ISample> pWriteSample;
//LOG_INFO(cString::Format("Driver Module:  %d", i8ActionID));
    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_juryStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlJuryStructId.actionId, i8ActionID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlJuryStructId.maneuverEntry, i16entry));

    }
    m_oStateFromJuly << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}



tResult cOptIlm_DriverModule::TransmitManeuverListr()
{
    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        {
            auto oCodec = m_ManeuverListSampleFactory.MakeCodecFor(pWriteSample);

            tManeuverListStruct *ptManeuverList = reinterpret_cast<tManeuverListStruct*>(oCodec.GetElementAddress(m_ddlManeuverListDataId.maneuver));


            for (int index = 0; index < 150; index++)
            {
                if(index < ManeuverList.number_of_id)
                {
                    ptManeuverList[index].i16Action  = ManeuverList.action[index][0];
                    ptManeuverList[index].i16Action2 = ManeuverList.action[index][1];
                }
                else
                {
                    ptManeuverList[index].i16Action  = 999;
                    ptManeuverList[index].i16Action2 = 999;
                }
            }

        }
        m_oSend_maneuver_listr << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

