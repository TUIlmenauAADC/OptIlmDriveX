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


#include <adtf_systemsdk.h>

using namespace adtf::util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::system;

#include "OptIlm_SignalValueGenerator.h"



ADTF_PLUGIN(LABEL_SIGNALVALUEGENERATOR_STREAMING_SOURCE, OptIlm_SignalValueGenerator);

OptIlm_SignalValueGenerator::OptIlm_SignalValueGenerator()
{
    RegisterPropertyVariable("Test Index", m_value);
    RegisterPropertyVariable("Timer Interval [Hz]", m_timerInterval);
}

tResult OptIlm_SignalValueGenerator::Construct()
{
    RETURN_IF_FAILED(cSampleStreamingSource::Construct());

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_signalValueSampleFactory))
    {
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_signalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp));
        RETURN_IF_FAILED(adtf_ddl::access_element::find_index(m_signalValueSampleFactory, "f32Value", m_ddlSignalValueId.value));

    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    //Register Output Pin
    RETURN_IF_FAILED(create_pin(*this, m_SteeringOut, "Steering_output", pTypeSignalValue));
    RETURN_IF_FAILED(create_pin(*this, m_SpeedOut, "Speed_output", pTypeSignalValue));

    RETURN_NOERROR;
}

tResult OptIlm_SignalValueGenerator::StartStreaming()
{

    timer_array = (float*)calloc(m_value,sizeof(float));
    steering_array = (float*)calloc(m_value,sizeof(float));
    speed_array = (float*)calloc(m_value,sizeof(float));
    timer_index = 0;
    timer_counter = 0;


    int time = 0;
    float steering = 0, speed = 0;
    int index = 0,status = 0;
    log_file = fopen("/home/aadc/AADC/src/aadcUser/OptIlmDriveX/OptIlm_SignalValueGenerator/Test.txt","r");

    max_index = m_value;
    for(index = 0; index < m_value; index++)
    {
        status = fscanf(log_file,"%d %f %f",&time, &steering, &speed);

        if(status == -1)
        {
            max_index = index;
            steering_array[index] = 0;
            speed_array[index] = 0;
            break;
        }

        timer_array[index] = (float)time;
        steering_array[index] = steering;
        speed_array[index] = speed;

//        LOG_INFO(cString::Format("%d, %g %g %g %d", (index + 1),time, steering, speed, status));
        LOG_INFO(cString::Format("%d, %g %g %g          %d", (index + 1),timer_array[index], steering_array[index], speed_array[index], status));
    }
    fclose(log_file);


    RETURN_IF_FAILED(cSampleStreamingSource::StartStreaming());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    m_oTimer = kernel_timer(cString(get_named_graph_object_full_name(*this) + "::frame_timer"), 1000/*m_timerInterval*/, 0,
        &OptIlm_SignalValueGenerator::TimerFunc,
        this);
    
    if (!m_oTimer.Stoppable())
    {
        RETURN_ERROR_DESC(ERR_UNEXPECTED, "Unable to create kernel timer");
    }

    RETURN_NOERROR;
}

tResult OptIlm_SignalValueGenerator::StopStreaming()
{
    m_oTimer.Stop();

    return cSampleStreamingSource::StopStreaming();
}

tVoid OptIlm_SignalValueGenerator::TimerFunc()
{


    if(timer_counter >= timer_array[timer_index] && timer_index < max_index)
    {
        LOG_INFO(cString::Format("%d, %g %g %g", (timer_index + 1),timer_array[timer_index], steering_array[timer_index], speed_array[timer_index]));
        TransmitNewSample(steering_array[timer_index], speed_array[timer_index]);
        timer_index++;
    }

    if(timer_index < max_index)
        timer_counter++;
    else
    {
        TransmitNewSample(0, 0);
    }



}

tResult OptIlm_SignalValueGenerator::TransmitNewSample(float steering, float speed)
{

//    transmitSignalValue(m_SteeringOut, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, steering);
//    transmitSignalValue(m_SpeedOut, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, speed);


//    object_ptr<ISample> pSample;
//    if IS_OK(alloc_sample(pSample, m_pClock->GetStreamTime()))
//    {
//        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pSample);
//        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, &steering));

//    }
//    if (pSample)
//    {
//        m_SteeringOut << pSample << trigger;
//    }

//    object_ptr<ISample> pSample_speed;
//    if IS_OK(alloc_sample(pSample_speed, m_pClock->GetStreamTime()))
//    {
//        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pSample_speed);
//        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, &speed));

//    }
//    if (pSample)
//    {
//        m_SpeedOut << pSample_speed << trigger;
//    }




    object_ptr<ISample> pWriteSample;
    tUInt32 timeStamp = 0;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, timeStamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, steering));
    }

    m_SteeringOut << pWriteSample << flush << trigger;


    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_signalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, timeStamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, speed));
    }

    m_SpeedOut << pWriteSample << flush << trigger;



    RETURN_NOERROR;

}
