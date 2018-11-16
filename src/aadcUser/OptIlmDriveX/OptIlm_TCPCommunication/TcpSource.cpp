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
#include "TcpSource.h"
#include <aadc_structs.h>
#include <aadc_jury.h>


#define SOCKET_TIMEOUT_MICROSECONDS 50000

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

using namespace aadc::jury;


int space [10][2] = {0};
int space_id =  0;
int space_id_counter = 0;




cTcpSource::cTcpSource()
{
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);
    RegisterPropertyVariable("port", m_propTCPPort);

    //m_oReadBuffer.resize(INT32_MAX);
    m_oReadBuffer.assign(MaxPacketCount * 1500 /*the max ethernet packet size */, 0);
}

tResult cTcpSource::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());
    //Get Media Descriptions
    object_ptr<IStreamType> pTypeJuryStruct;
    cString structName = "tJuryStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeJuryStruct, m_juryStructSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    RETURN_IF_FAILED(create_pin(*this, m_oOutputJuryStruct, "jury_struct", pTypeJuryStruct));


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
    RETURN_IF_FAILED(create_pin(*this, m_oSend_maneuver_listr, "Demo_ManeuverList", pTypeManeuverListr));



//    object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_anonymous());
//    //the file outputs
//    RETURN_IF_FAILED(create_pin(*this, m_oOutputManeuverList, "maneuver_list", pType));
//    RETURN_IF_FAILED(create_pin(*this, m_oOutputOpenDriveMap, "open_drive_map", pType));
//    RETURN_IF_FAILED(create_pin(*this, m_oOutputTrafficSignMap, "road_sign_map", pType));

    m_pSocket = make_object_ptr<cSocketWrapper>();
    // create an interface binding server, so that the sink can access our socket instance.
    RETURN_IF_FAILED(create_server<ISocket>(*this, "socket", ucom_object_ptr_cast<ISocket>(m_pSocket)));

    RETURN_NOERROR;
}

tResult cTcpSource::StartStreaming()
{
    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    m_oReadThread = kernel_thread_looper(get_named_graph_object_full_name(*this) + "::recieve_thread",
                                         &cTcpSource::ReadThread,
                                         this);

    if (!m_oReadThread.Joinable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create read kernel thread");
    }

    RETURN_NOERROR;
}

tResult cTcpSource::Init()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Init());

//    RETURN_IF_FAILED_DESC(m_serverSocket.Open(static_cast<tUInt>(m_propTCPPort), cServerSocket::SS_Exclusive),
    RETURN_IF_FAILED_DESC(m_serverSocket.Open(static_cast<tUInt>(m_propTCPPort)),
                          cString::Format("Could not open server socket with port %d", static_cast<tUInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket was opened with port %d", static_cast<tUInt>(m_propTCPPort)));
    RETURN_IF_FAILED_DESC(m_serverSocket.Listen(),
                          cString::Format("Could not listen to port %d", static_cast<tUInt>(m_propTCPPort)));
    LOG_INFO(cString::Format("Server Socket now listens on port %d", static_cast<tUInt>(m_propTCPPort)));
    if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(5e5)))
    {
        RETURN_IF_FAILED_DESC(m_serverSocket.Accept(*m_pSocket), "Could not access Server socket");
        m_clientConnectionEstablished = tTrue;
        LOG_INFO("TCP Connection was established");
    }
    else
    {
        LOG_WARNING(cString::Format("No client is connected on Port %d", static_cast<tUInt>(m_propTCPPort)));
        m_clientConnectionEstablished = tFalse;
    }

    RETURN_NOERROR;
}

tResult cTcpSource::Shutdown()
{
    //closes the connections and the server
    m_pSocket->Close();
    m_serverSocket.Close();
    m_clientConnectionEstablished = tFalse;
    return cSampleStreamingSource::Shutdown();
}

tResult cTcpSource::StopStreaming()
{
    m_oReadThread = kernel_thread_looper();
    return cSampleStreamingSource::StopStreaming();
}

tVoid cTcpSource::ReadThread()
{
    // no stream connected yet
    if (!m_clientConnectionEstablished)
    {
        // try to connect to client
        if (m_serverSocket.IsConnected(static_cast<tTimeStamp>(2e2)))
        {
            if IS_FAILED(m_serverSocket.Accept(*m_pSocket))
            {
                return;
            }
            LOG_INFO("TCP Connection was established");
            m_clientConnectionEstablished = tTrue;
        }
    }
    else
    {
        if (m_pSocket->DataAvailable())
        {
            tResult nResult = ReadAndForwardPacket();
            if (IS_FAILED(nResult))
            {
                m_oOutputManeuverList.SetStreamError(nResult);
            }
        }

    }
}

tResult cTcpSource::ProcessJuryContainer()
{
    tJuryContainer container;
    if (deserializeContainer(std::vector<char>(m_oReadBuffer.begin(), m_oReadBuffer.begin() + m_totalBytesRead), container))
    {

        switch (container.id)
        {
            case container_id_juryStruct:
            {
                tJuryStruct* pReceivedData = reinterpret_cast<tJuryStruct*>(container.data);
                RETURN_IF_FAILED(TransmitJuryStruct(*pReceivedData));
                break;
            }
            case container_id_maneuverlist:
            {
                LOG_INFO(container.data);

                RETURN_IF_FAILED(TransmitFileData(m_oOutputManeuverList, container.data, container.dataSize));
                break;
            }
//            case container_id_opendrive_map:
//            {
//                RETURN_IF_FAILED(TransmitFileData(m_oOutputOpenDriveMap, container.data, container.dataSize));
//                break;
//            }
//            case container_id_traffic_sign_map:
//            {
//                LOG_INFO(container.data);
//                RETURN_IF_FAILED(TransmitFileData(m_oOutputTrafficSignMap, container.data, container.dataSize));
//                break;
//            }
            case container_id_unkown:
            default:
                break;

        }
    }
    else
    {
        RETURN_ERROR_DESC(ERR_INVALID_TYPE, "could not interpret data");
    }
    RETURN_NOERROR;
}

tResult cTcpSource::ReadAndForwardPacket()
{
    tInt m_nReadBytes = -1;
    tResult result = ERR_NOERROR;
    char data_array[100];
    int underscore_counter = 0;

    if (m_pSocket->DataAvailable())
    {
        result = (m_pSocket->Read(m_oReadBuffer.data() + m_totalBytesRead, m_oReadBuffer.size(), &m_nReadBytes));
        if (IS_FAILED(result) && result != ERR_DEVICE_IO)
        {   //connection was lost, ERR_DEVICE_IO == reading finished
            LOG_WARNING("TCP Connection was disconnected");
            m_clientConnectionEstablished = tFalse;
            RETURN_ERROR(ERR_NOT_CONNECTED);
        }
        m_totalBytesRead += m_nReadBytes;
        m_packetCnt++;




        std::string temp(m_oReadBuffer.begin(),m_oReadBuffer.end());
        str2=temp.data();
        str2=str2.erase(0,str1.length());
        memcpy(data_array, str2.data(), m_nReadBytes);
        str1=temp.data();


        tJuryStruct newStruct;

//        LOG_INFO(str2.data());

        if(m_nReadBytes == 3)
        {
            int state = (data_array[0] - 0x30);
            int ManeuverEntry = 0;

            switch (data_array[2])
            {
                case 'H':
                    for(int index = 0; index < space_id; index++)
                    {
                        if(space[index][0] == HOME)
                            ManeuverEntry = space[index][1];
                    }
                    break;
                case 'O':
                    for(int index = 0; index < space_id; index++)
                    {
                        if(space[index][0] == OFFICE)
                            ManeuverEntry = space[index][1];
                    }
                    break;
                case 'R':
                    for(int index = 0; index < space_id; index++)
                    {
                        if(space[index][0] == RESTAURANT)
                            ManeuverEntry = space[index][1];
                    }
                    break;
                case 'P':
                    for(int index = 0; index < space_id; index++)
                    {
                        if(space[index][0] == POST)
                            ManeuverEntry = space[index][1];
                    }
                    break;
                case 'S':
                    for(int index = 0; index < space_id; index++)
                    {
                        if(space[index][0] == SUPERMARKET)
                            ManeuverEntry = space[index][1];
                    }
                    break;
                case 'G':
                    for(int index = 0; index < space_id; index++)
                    {
                        if(space[index][0] == GARAGE)
                            ManeuverEntry = space[index][1];
                    }
                    break;

                case 'D':
                    ManeuverEntry =  space[space_id_counter][1];
                    if(state == 1)
                        space_id_counter++;
                    break;

                default:
                    break;
            }

            if(state == 0)
            {
                LOG_INFO(cString::Format("Get Ready (%d) ManeuverEntry %d  ReadBytes %d", state, ManeuverEntry, m_nReadBytes));

            }
            else if(state == 1)
            {
                LOG_INFO(cString::Format("Get Start (%d) ManeuverEntry %d  ReadBytes %d", state, ManeuverEntry, m_nReadBytes));
            }
            else
                LOG_INFO(cString::Format("Get Error (%d) ReadBytes %d", state, m_nReadBytes));

            newStruct.i16ActionID = state;
            newStruct.i16ManeuverEntry = ManeuverEntry;
            RETURN_IF_FAILED(TransmitJuryStruct(newStruct));
        }
        else if(m_nReadBytes == 2)
        {
            LOG_INFO(cString::Format("Get Stop (%d)  ReadBytes %d", atoi(str2.data()), m_nReadBytes));
            newStruct.i16ActionID = atoi(str2.data());
            newStruct.i16ManeuverEntry = 0;
            RETURN_IF_FAILED(TransmitJuryStruct(newStruct));

        }
        else
        {
            LOG_INFO(cString::Format("Get Maneuver List (%s)  ReadBytes %d", str2.data(), m_nReadBytes));
            ManeuverList.number_of_id = 0;
            space_id = 0;
            for(int index = 0; index < m_nReadBytes; index++)
            {

//                LOG_INFO(data_array[index]);
                switch (data_array[index])
                {
                    case '1':
                        ManeuverList.action[ManeuverList.number_of_id][0] = TURN_LEFT;
                        ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                        ManeuverList.number_of_id++;
                        break;
                    case '2':
                        ManeuverList.action[ManeuverList.number_of_id][0] = TURN_RIGHT;
                        ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                        ManeuverList.number_of_id++;
                        break;
                    case '0':
                        ManeuverList.action[ManeuverList.number_of_id][0] = STRAIGHT;
                        ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                        ManeuverList.number_of_id++;
                        break;
                    case '3':
                        ManeuverList.action[ManeuverList.number_of_id][0] = PULL_OUT_LEFT;
                        ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                        ManeuverList.number_of_id++;
                        break;
                    case '4':
                        ManeuverList.action[ManeuverList.number_of_id][0] = PULL_OUT_RIGHT;
                        ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                        ManeuverList.number_of_id++;
                        break;
                    case '5':
                        ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                        ManeuverList.action[ManeuverList.number_of_id][1] = 6;
                        ManeuverList.number_of_id++;
                        break;

                    case 'H':
                        space[space_id][0] = HOME;
                        space_id++;
                        break;
                    case 'O':
                        space[space_id][0] = OFFICE;
                        space_id++;
                        break;
                    case 'R':
                        space[space_id][0] = RESTAURANT;
                        space_id++;
                        break;
                    case 'P':
                        space[space_id][0] = POST;
                        space_id++;
                        break;
                    case 'S':
                        space[space_id][0] = SUPERMARKET;
                        space_id++;
                        break;

                    case 'G':
                        space[space_id][0] = GARAGE;
                        space_id++;
                        break;

                    case '_':
                        space[underscore_counter][1] = ManeuverList.number_of_id;
                        underscore_counter++;
                        break;

                    default:
                        break;
                }
            }
            ManeuverList.action[ManeuverList.number_of_id][0] = CAR_STOP;
            ManeuverList.action[ManeuverList.number_of_id][1] = 0;


            if (ManeuverList.number_of_id > 0)
            {
                for(int index = 0; index < space_id; index++)
                {
                    LOG_INFO(cString::Format("Demo  %d. Ort: %d  Maneuver ID: %d", index + 1, space[index][0], space[index][1]));
                }

                for(int index = 0; index <= ManeuverList.number_of_id; index++)
                {
                    switch (ManeuverList.action[index][0])
                    {
                    case CAR_STOP:
                        LOG_WARNING(cString::Format("Demo Maneuver End ID: %d action: Car Stop", ManeuverList.number_of_id));
                        break;
                    case PULL_OUT_LEFT:
                        LOG_INFO(adtf_util::cString::Format("Demo Maneuver id: %d  pull_out_left", index));
                        break;
                    case PULL_OUT_RIGHT:
                        LOG_INFO(adtf_util::cString::Format("Demo Maneuver id: %d  pull_out_right", index));
                        break;
                    case TURN_LEFT:
                        LOG_INFO(adtf_util::cString::Format("Demo Maneuver id: %d  left", index));
                        break;
                    case TURN_RIGHT:
                        LOG_INFO(adtf_util::cString::Format("Demo Maneuver id: %d  right", index));
                        break;
                    case STRAIGHT:
                        LOG_INFO(adtf_util::cString::Format("Demo Maneuver id: %d  straight", index));
                        break;
                    case PARKING:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  cross_parking %d", index, ManeuverList.action[index][1]));
                        break;
                    }

                }


                TransmitManeuverListr();
                space_id_counter = 0;
            }
            else
            {
                LOG_ERROR("no valid Maneuver Data found!");
            }


        }

//        m_totalBytesRead = 0;
//        m_packetCnt = 0;

    }
    RETURN_NOERROR;
}

tResult cTcpSource::TransmitFileData(adtf::streaming::cSampleWriter& output, void* data, tSize length)
{
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample));
    pSample->SetTime(m_pClock->GetStreamTime());
    {
        object_ptr_locked<ISampleBuffer> pBuffer;
        RETURN_IF_FAILED(pSample->WriteLock(pBuffer, length));
        RETURN_IF_FAILED(pBuffer->Write(adtf_memory_buffer<const tVoid>(data, length)));
    }
    output << pSample << trigger;
    RETURN_NOERROR;
}

tResult cTcpSource::TransmitJuryStruct(tJuryStruct& juryStruct)
{
    object_ptr<ISample> pWriteSample;
    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_juryStructSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlJuryStructId.actionId, juryStruct.i16ActionID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlJuryStructId.maneuverEntry, juryStruct.i16ManeuverEntry));

    }
    m_oOutputJuryStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cTcpSource::TransmitManeuverListr()
{
    object_ptr<ISample> pWriteSample;


    if (IS_OK(alloc_sample(pWriteSample)))
    {
        {
            auto oCodec = m_ManeuverListSampleFactory.MakeCodecFor(pWriteSample);

            tManeuverListStruct *ptManeuverList = reinterpret_cast<tManeuverListStruct*>(oCodec.GetElementAddress(m_ddlManeuverListDataId.maneuver));

            int space_counter = 0;

            for (int index = 0; index < 150; index++)
            {
                if(index < ManeuverList.number_of_id)
                {
                    ptManeuverList[index].i16Action  = ManeuverList.action[index][0];
                    ptManeuverList[index].i16Action2 = ManeuverList.action[index][1];
                }
                else if(index >= ManeuverList.number_of_id && index < (ManeuverList.number_of_id + space_id))
                {
                    ptManeuverList[index].i16Action  = 800 + space[space_counter][1];
                    ptManeuverList[index].i16Action2 = space[space_counter][0];
                    space_counter++;
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
