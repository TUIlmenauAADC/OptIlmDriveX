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


#include "OptIlm_WheelsAngleConverter.h"
#include "ADTF3_helper.h"


#define MAX_POSITIVE_STEERING_ANGLE           31//28.2  //Degree//20
#define MAX_NEGATIVE_STEERING_ANGLE           35//252.5.8 //25.8//19//26//25
#define POSITIVE_STEERING_ANGLE_TO_PERCENT    (88.0 / MAX_POSITIVE_STEERING_ANGLE)
#define NEGATIVE_STEERING_ANGLE_TO_PERCENT    (90.0 / MAX_NEGATIVE_STEERING_ANGLE)

#define POSITIVE_STEERING_ANGLE_a           0.014  //
#define POSITIVE_STEERING_ANGLE_b           -0.5754
#define POSITIVE_STEERING_ANGLE_c           9.5548
#define POSITIVE_STEERING_ANGLE_d           20.2373

#define NEGATIVE_STEERING_ANGLE_a           -0.0196  //
#define NEGATIVE_STEERING_ANGLE_b           0.7212
#define NEGATIVE_STEERING_ANGLE_c           -11.0728
#define NEGATIVE_STEERING_ANGLE_d           -17.53

float temp_output_steering[4];


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "OptIlm WheelsAngleConverter",
    cOptIlm_WheelsAngleConverter,
//    adtf::filter::pin_trigger({ "Wheels_Angle_Input" }))
    adtf::filter::thread_trigger(true))


cOptIlm_WheelsAngleConverter::cOptIlm_WheelsAngleConverter()
{
    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_ERROR("No mediadescription for tSignalValue found!");
    }


    Register(m_oReader, "Wheels_Angle_Input" , pTypeSignalValue);
    Register(m_oWriter, "OutputServoPercent", pTypeSignalValue);

    m_lastSampleTime = 0;

    for (int i = 0;i<4;i++)
    {
        temp_output_steering[i]=0;
    }
}


//implement the Configure function to read ALL Properties
tResult cOptIlm_WheelsAngleConverter::Configure()
{
    RETURN_NOERROR;
}

tResult cOptIlm_WheelsAngleConverter::Process(tTimeStamp tmTimeOfTrigger)
{
    tTimeStamp m_NowSampleTime =  tmTimeOfTrigger;
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;

    if(f64SampleTime > 0.010)
    {
        m_lastSampleTime = m_NowSampleTime;
//        LOG_INFO(cString::Format("WheelsAngleConverter Sample Time = %.3f S", f64SampleTime));



        tSignalValue input_wheels_angle;

        object_ptr<const ISample> pReadSample;
        if (IS_OK(m_oReader.GetLastSample(pReadSample)))
        {
            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            // retrieve the values (using convenience methods that return a variant)
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &input_wheels_angle.f32Value));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &input_wheels_angle.ui32ArduinoTimestamp));


        }


        // Do the Processing
        tFloat32 output_servo_percent = 0;

        if(input_wheels_angle.f32Value == 0)
            output_servo_percent = 0;
        else if(input_wheels_angle.f32Value > 0)
        {
            output_servo_percent = input_wheels_angle.f32Value * POSITIVE_STEERING_ANGLE_TO_PERCENT;
            //                output_servo_percent = input_wheels_angle.f32Value * input_wheels_angle.f32Value * input_wheels_angle.f32Value * POSITIVE_STEERING_ANGLE_a +
            //                                       input_wheels_angle.f32Value * input_wheels_angle.f32Value * POSITIVE_STEERING_ANGLE_b +
            //                                       input_wheels_angle.f32Value * POSITIVE_STEERING_ANGLE_c + POSITIVE_STEERING_ANGLE_d;
            //                if (input_wheels_angle.f32Value<1.5)
            //                {
            //                    output_servo_percent = input_wheels_angle.f32Value /1.5 * 30;
            //                }
        }

        else if(input_wheels_angle.f32Value < 0)
        {
            //                input_wheels_angle.f32Value = input_wheels_angle.f32Value  * -1;
            output_servo_percent = input_wheels_angle.f32Value * NEGATIVE_STEERING_ANGLE_TO_PERCENT;
            //                output_servo_percent = input_wheels_angle.f32Value * input_wheels_angle.f32Value * input_wheels_angle.f32Value * NEGATIVE_STEERING_ANGLE_a +
            //                                       input_wheels_angle.f32Value * input_wheels_angle.f32Value * NEGATIVE_STEERING_ANGLE_b +
            //                                       input_wheels_angle.f32Value * NEGATIVE_STEERING_ANGLE_c + NEGATIVE_STEERING_ANGLE_d;
            //                if (input_wheels_angle.f32Value < 1.2)
            //                {
            //                    output_servo_percent = - input_wheels_angle.f32Value /1.2 * 30;
            //                }
        }
        //debug
        //    LOG_INFO(cString::Format("------- Steering percentage = %.3f S", output_servo_percent));

//        if (fabs(temp_output_steering[0] - output_servo_percent)>30&&fabs(temp_output_steering[0])>50)
//        {
//             LOG_INFO(cString::Format("-------oldoutput= %.3f S", output_servo_percent));
//            output_servo_percent = (0.5*output_servo_percent + 2*temp_output_steering[0]+3*temp_output_steering[1] +1.5*temp_output_steering[2])/7; // + temp_output_steering[1] + temp_output_steering[2] + temp_output_steering[3])/4.5;
//            //debug
//                    LOG_INFO(cString::Format("-------Smoothing the Steering output = %.3f S", output_servo_percent));
//        }


        if(output_servo_percent > 90)
            output_servo_percent = 90;
        else if(output_servo_percent < -90)
            output_servo_percent = -90;

//        temp_output_steering[3] = temp_output_steering[2];
//        temp_output_steering[2] = temp_output_steering[1];
//        temp_output_steering[1] = temp_output_steering[0];
//        temp_output_steering[0] = output_servo_percent;


        object_ptr<ISample> pWriteSample;

        if (IS_OK(alloc_sample(pWriteSample)))
        {

            auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, output_servo_percent));
            RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, input_wheels_angle.ui32ArduinoTimestamp));

        }
        m_oWriter << pWriteSample << flush << trigger;
    }
    
    RETURN_NOERROR;
}
