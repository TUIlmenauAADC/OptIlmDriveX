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


#include "OptIlm_EmergencyBreak.h"


short emergency_break_counter = 0;


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "OptIlm_EmergencyBreak",
    OptIlm_EmergencyBreak,
    adtf::filter::pin_trigger({"VehicleSpeed"}));


OptIlm_EmergencyBreak::OptIlm_EmergencyBreak()
{
    SetName("Emergency Break");

    //register properties
    RegisterPropertyVariable("1.Front Minimum break time(S)", m_front_min_break_time);
    RegisterPropertyVariable("2.Rear Minimum break time(s)", m_rear_min_break_time);
    RegisterPropertyVariable("3.Minimum break distance(cm)", m_min_break_distance);



    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    Register(m_oInputSpeed, "ControlSpeed_pct" , pTypeSignalValue);
    Register(m_oVehicleSpeed, "VehicleSpeed" , pTypeSignalValue);
    Register(m_oOutputSpeed, "OutputSpeed_pct", pTypeSignalValue);
    Register(m_oEmergencyBreakFlag, "EmergencyBreakFlag", pTypeSignalValue);



    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method.
        LOG_INFO("Found mediadescription for tLaserScannerData!");
        //get all the member indices
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }
    else
    {
        LOG_INFO("No mediadescription for tLaserScannerData found!");
    }
    Register(m_oInputLaserScanner, "laser_scanner" , pTypeLSData);



    //the us struct
    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pTypeUSData, m_USDataSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.SideRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.SideRight.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearCenter.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearCenter.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".ui32ArduinoTimestamp"), m_ddlUltrasonicStructIndex.RearRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") + cString(".f32Value"), m_ddlUltrasonicStructIndex.RearRight.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    Register(m_oInputUltrasonicUnit, "ultrasonic_struct" , pTypeUSData);

    EmergencyBreakFlag.f32Value = 0;
    EmergencyBreakFlag.ui32ArduinoTimestamp = 0;
}


//implement the Configure function to read ALL Properties
tResult OptIlm_EmergencyBreak::Configure()
{

    RETURN_IF_FAILED(cTriggerFunction::Configure());

    RETURN_NOERROR;
}

tResult OptIlm_EmergencyBreak::Process(tTimeStamp tmTimeOfTrigger)
{

    object_ptr<const ISample> pReadSample;

    tSignalValue input_control_speed_pct;

    if (IS_OK(m_oInputSpeed.GetLastSample(pReadSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &input_control_speed_pct.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &input_control_speed_pct.ui32ArduinoTimestamp));

    }



    object_ptr<const ISample> pSampleFromVcc;

    tSignalValue vehicle_speed;

    if (IS_OK(m_oVehicleSpeed.GetNextSample(pSampleFromVcc)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromVcc);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &vehicle_speed.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &vehicle_speed.ui32ArduinoTimestamp));

    }


    object_ptr<const ISample> pSampleFromLS;

    float x = 0, y = 0;
    float angel_RAD = 0;
    float ObstacleToClose = 0;
    float min_break_distance = 0;

    if(vehicle_speed.f32Value > 0 || input_control_speed_pct.f32Value < 0)
    {
        min_break_distance = (vehicle_speed.f32Value * m_front_min_break_time) * 100;
        if(min_break_distance < m_min_break_distance)
            min_break_distance = m_min_break_distance;


        if (IS_OK(m_oInputLaserScanner.GetLastSample(pSampleFromLS)))
        {
            //Angle is Degree
            //radius is mm

            auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pSampleFromLS);

            RETURN_IF_FAILED(oDecoder.IsValid());
            tSize numOfScanPoints = 0;
            tResult res = oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints);

            const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

            for (tSize i = 0; i < numOfScanPoints; ++i)
            {
                angel_RAD = pCoordinates[i].f32Angle * DEGREES_TO_RADIAN;
                x = (pCoordinates[i].f32Radius * cos(angel_RAD)) * 0.1;  //mm to Cm
                y = (pCoordinates[i].f32Radius * sin(angel_RAD)) * 0.1;

//                LOG_INFO(cString::Format("%d Angle = %.2f   x , y =  %.2f , %.2f", i,pCoordinates[i].f32Angle, x ,y));

                if(x != 0.0 && x < 20 && (y < 25 && y > -25))
                {
                    ObstacleToClose = 1;
                    emergency_break_counter = 30;
                }


                //            LOG_INFO(cString::Format("%d Angle %g Radius %g", i,pCoordinates[i].f32Angle ,pCoordinates[i].f32Radius));
            }
//            LOG_INFO("----------Ende------------------------------------");
        }
    }
    else if(vehicle_speed.f32Value < 0 || input_control_speed_pct.f32Value > 0)
    {
        min_break_distance = (vehicle_speed.f32Value * m_rear_min_break_time) * 100;
        if(min_break_distance > -m_min_break_distance)
            min_break_distance = -m_min_break_distance;


        object_ptr<const ISample> pSampleFromUS;

        if (IS_OK(m_oInputUltrasonicUnit.GetLastSample(pSampleFromUS)))
        {
            auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

            RETURN_IF_FAILED(oDecoderUS.IsValid());


            // retrieve the values (using convenience methods that return a variant)
            //IMU
            tUltrasonicStruct US_data;

            //we do not need the timestamps here
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &US_data.tSideLeft.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &US_data.tSideRight.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &US_data.tRearLeft.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &US_data.tRearCenter.f32Value));
            RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &US_data.tRearRight.f32Value));


            if((US_data.tRearLeft.f32Value < -min_break_distance) ||
               (US_data.tRearCenter.f32Value < -min_break_distance) ||
               (US_data.tRearRight.f32Value < -min_break_distance))
            {
                ObstacleToClose = 1;
                emergency_break_counter = 30;
            }
        }



//        radian = 210 * DEGREES_TO_RADIAN;
//        ult_world_coord[R_LEFT][X] = (ult_value[14] * sin(radian)) - 30; //X is
//        ult_world_coord[R_LEFT][Y] = (ult_value[14] * cos(radian)) - 10; //Y is ->

//        ult_world_coord[R_CENTER][X] = -ult_value[16] - 30; //X is
//        ult_world_coord[R_CENTER][Y] = 0; //Y is ->

//        radian = 330 * DEGREES_TO_RADIAN;
//        ult_world_coord[R_RIGHT][X] = (ult_value[18] * sin(radian)) - 30; //X is
//        ult_world_coord[R_RIGHT][Y] = (ult_value[18] * cos(radian)) + 10; //Y is ->



    }


//ObstacleToClose =0;
//emergency_break_counter =0;
    if(ObstacleToClose == 1)
    {

        input_control_speed_pct.f32Value = 0;
        TransmitSpeed(input_control_speed_pct);

        EmergencyBreakFlag.f32Value = 1;
        TransmitEmergencyBreakFlag(EmergencyBreakFlag);


    }
    else if(ObstacleToClose == 0 && emergency_break_counter != 0)
    {
        input_control_speed_pct.f32Value = 0;
        TransmitSpeed(input_control_speed_pct);
        emergency_break_counter--;
        if(emergency_break_counter <= 0)
            emergency_break_counter = 0;
    }
    else if(ObstacleToClose == 0 && emergency_break_counter == 0)
    {
        TransmitSpeed(input_control_speed_pct);

        EmergencyBreakFlag.f32Value = 0;
        TransmitEmergencyBreakFlag(EmergencyBreakFlag);
    }
    
    RETURN_NOERROR;
}

tResult OptIlm_EmergencyBreak::TransmitSpeed(tSignalValue speed_signal)
{
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, speed_signal.f32Value));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, speed_signal.ui32ArduinoTimestamp));

    }
    m_oOutputSpeed << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult OptIlm_EmergencyBreak::TransmitEmergencyBreakFlag(tSignalValue Flag)
{
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, Flag.f32Value));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, Flag.ui32ArduinoTimestamp));

    }
    m_oEmergencyBreakFlag << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

