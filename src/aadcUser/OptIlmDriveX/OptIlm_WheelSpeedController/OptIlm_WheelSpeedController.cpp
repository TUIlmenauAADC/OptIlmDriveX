/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS ?AS IS? AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hart#$  $Date:: 2017-05-19 08:12:10#$ $Rev:: 63515   $
**********************************************************************/
#include <adtf3.h>
#include <stdlib.h>
#include "OptIlm_WheelSpeedController.h"
#include "ADTF3_helper.h"


/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_WHEELSPEEDCONTROLLER_DATA_TRIGGERED_FILTER,
                                    "OptIlm_WheelSpeedController",
                                    cOptIlm_WheelSpeedController,
                                    //                                    adtf::filter::pin_trigger({"measured_vehicle_speed"}));
                                    adtf::filter::thread_trigger(true))

////adtf::filter::thread_trigger(tTrue));




#define MOTOR_F_MAX			-50		//% Gas
#define MOTOR_F_MIN			-7		//% Gas//-5
#define MOTOR_F_GAS_LIMIT	-50//-50	//30	//% Gas
#define MOTOR_F_BRAKE_LIMIT	20		//Linit der Bremse

#define MOTOR_B_MAX   		50		//445 = 15% Gas
#define MOTOR_B_MIN			7		//339 = 100% Gas//9
#define MOTOR_B_GAS_LIMIT	50//50		//445 = 40% Gas//30
#define MOTOR_B_BRAKE_LIMIT	-20		//Linit der Bremse

#define MOTOR_NULL_POINT	0 	//stop
#define MS_TO_PRC_F		11.57//7.76	//	45(Max - Min)/ 5.8(Max SPEED)
#define MS_TO_PRC_B		13.84//8.91	//	41(%)/ 4.6(KMH)

#define MOTOR_START_F_PCT 8//7
#define MOTOR_START_B_PCT 8



//CTOR of the TriggerFuntion
//This is to initialize the Trigger
cOptIlm_WheelSpeedController::cOptIlm_WheelSpeedController()
{
    // SetName("Timer_trigger");

    //Register Properties
    RegisterPropertyVariable("01.controller type", m_i32ControllerMode                              );
    RegisterPropertyVariable("02.proportional factor for PID Controller ", m_f64PIDKp               );
    RegisterPropertyVariable("03.integral factor for PID Controller", m_f64PIDKi                    );
    RegisterPropertyVariable("04.differential factor for PID Controller", m_f64PIDKd                );
    RegisterPropertyVariable("05.sampletime for the pid controller [S]", m_f64PIDSampleTime        );
    RegisterPropertyVariable("06.input factor for PT1", m_f64PT1OutputFactor                        );
    RegisterPropertyVariable("07.time constant for pt1 controller", m_f64PT1TimeConstant            );
    RegisterPropertyVariable("08.set point is multiplied with this factor", m_f64PT1CorrectionFactor);
    RegisterPropertyVariable("09.gain factor for PT1 controller", m_f64PT1Gain                      );
    RegisterPropertyVariable("10.the minimum output value for the controller [%]", m_f64PIDMinimumOutput);
    RegisterPropertyVariable("11.the maximum output value for the controller [%]", m_f64PIDMaximumOutput);
    RegisterPropertyVariable("12.show debug output", m_bShowDebug                                   );






    //Get Media Descriptions
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
    Register(m_oInputMeasWheelSpeed, "measured_vehicle_speed", pTypeSignalValue);
    Register(m_oInputSetWheelSpeed,  "desired_vehicle_speed", pTypeSignalValue);
    Register(m_oInputSteering, "steering", pTypeSignalValue);//XH

    Register(m_oEmergencyBreakFlag,  "Emergency_Break_Flag", pTypeSignalValue);

    Register(m_oOutputActuator,      "actuator_output", pTypeSignalValue);

    star_flag = 1;


}


//implement the Configure function to read ALL Properties
tResult cOptIlm_WheelSpeedController::Configure()
{
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64accumulatedVariable = 0;


    EmergencyBreakFlag = 0;
    last_steering_angle = 0;
    difference_steering_angle = 0;

    //    RETURN_IF_FAILED(cTriggerFunction::Configure());


    switch (m_i32ControllerMode)
    {
    case 1:
        LOG_INFO(adtf_util::cString::Format("P controller"));
        LOG_INFO(adtf_util::cString::Format("Kp: %.4f",(float)m_f64PIDKp));
        break;
    case 2:
        LOG_INFO(adtf_util::cString::Format("PI controller"));
        LOG_INFO(adtf_util::cString::Format("Kp: %g; Ki: %g",(float)m_f64PIDKp, (float)m_f64PIDKi));
        break;
    case 3:
        LOG_INFO(adtf_util::cString::Format("PID controller"));
        LOG_INFO(adtf_util::cString::Format("Kp: %.4f; Ki: %.4f; Kd: %.4f",(float)m_f64PIDKp, (float)m_f64PIDKi, (float)m_f64PIDKd));
        break;
    case 4:
        LOG_INFO(adtf_util::cString::Format("PT1 controller"));
        LOG_INFO(adtf_util::cString::Format("input factor: %.4f; time constant: %.4f; set point is multiplied with this factor: %.4f; gain factor: %.4f",(float)m_f64PT1OutputFactor, (float)m_f64PT1TimeConstant, (float)m_f64PT1CorrectionFactor, (float)m_f64PT1Gain));
        break;
    default:
        break;
    }


    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}



///this funtion will be executed each time a trigger occured
///Due to the change of the data receive events it can be possible that more than one sample was pushed to the
/// Readers queue. So make sure the execution of this funtion will read ALL Samples of ALL Readers until the queues are empty.
tResult cOptIlm_WheelSpeedController::Process(tTimeStamp tmTimeOfTrigger)
{
    // avoid that the method triggers itself due to the feedback loop
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    tTimeStamp m_NowSampleTime = tmTimeOfTrigger;// GetTime();
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;


//    // Setpoint value speed
//    object_ptr<const ISample> pSetWheelSpeedSample;
//    while (IS_OK(m_oInputSetWheelSpeed.GetNextSample(pSetWheelSpeedSample)))
//    {
//        //write values with zero
//        tFloat32 f32Value = 0;
//        tUInt32  Ui32TimeStamp = 0;

//        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSetWheelSpeedSample);

//        RETURN_IF_FAILED(oDecoder.IsValid());

//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

//        // write to member variable
//        m_f64SetPoint = static_cast<tFloat64>(f32Value);
//        if (m_i32ControllerMode == 4)
//            m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;
//    }


//    // Read Output speed percent value because of Emergency Break
//    object_ptr<const ISample> pEmergencyBreakFlaSample;
//    while (IS_OK(m_oEmergencyBreakFlag.GetNextSample(pEmergencyBreakFlaSample)))
//    {
//        //write values with zero
//        tFloat32 f32Value = 0;
//        tUInt32  Ui32TimeStamp = 0;

//        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pEmergencyBreakFlaSample);

//        RETURN_IF_FAILED(oDecoder.IsValid());

//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

//        EmergencyBreakFlag = f32Value;
//    }

//    // Measured Wheel Speed
//    object_ptr<const ISample> pWheelSpeedSample;
//    while (IS_OK(m_oInputMeasWheelSpeed.GetNextSample(pWheelSpeedSample)))
//    {
//        //write values with zero
//        tFloat32 f32Value = 0;
//        tUInt32  Ui32TimeStamp = 0;

//        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pWheelSpeedSample);

//        RETURN_IF_FAILED(oDecoder.IsValid());

//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

//        // write to member variable
//        m_f64MeasuredVariable = f32Value;
//    }

//    object_ptr<const ISample> pSampleFromSteering;
//    while (IS_OK(m_oInputSteering.GetNextSample(pSampleFromSteering)))
//    {
//        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromSteering);

//        RETURN_IF_FAILED(oDecoder.IsValid());

//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &steering_angle.f32Value));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &steering_angle.ui32ArduinoTimestamp));

//        difference_steering_angle = fabs(last_steering_angle) - fabs(steering_angle.f32Value);
//        last_steering_angle = steering_angle.f32Value;
//        //debug
//        //            LOG_INFO(cString::Format("Steering Angle: %g ", m_f64WheelAngle));

//    }


    if(f64SampleTime > m_f64PIDSampleTime)
    {


        m_lastSampleTime = m_NowSampleTime;



        object_ptr<const ISample> pSetWheelSpeedSample;
        if (IS_OK(m_oInputSetWheelSpeed.GetLastSample(pSetWheelSpeedSample)))
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32  Ui32TimeStamp = 0;

            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSetWheelSpeedSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            // write to member variable
            m_f64SetPoint = static_cast<tFloat64>(f32Value);
            if (m_i32ControllerMode == 4)
                m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;
        }


        // Read Output speed percent value because of Emergency Break
        object_ptr<const ISample> pEmergencyBreakFlaSample;
        if (IS_OK(m_oEmergencyBreakFlag.GetLastSample(pEmergencyBreakFlaSample)))
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32  Ui32TimeStamp = 0;

            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pEmergencyBreakFlaSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            EmergencyBreakFlag = f32Value;
        }

        // Measured Wheel Speed
        object_ptr<const ISample> pWheelSpeedSample;
        if (IS_OK(m_oInputMeasWheelSpeed.GetLastSample(pWheelSpeedSample)))
        {
            //write values with zero
            tFloat32 f32Value = 0;
            tUInt32  Ui32TimeStamp = 0;

            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pWheelSpeedSample);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &Ui32TimeStamp));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value));

            // write to member variable
            m_f64MeasuredVariable = f32Value;
        }

        object_ptr<const ISample> pSampleFromSteering;
        if (IS_OK(m_oInputSteering.GetLastSample(pSampleFromSteering)))
        {
            auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromSteering);

            RETURN_IF_FAILED(oDecoder.IsValid());

            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &steering_angle.f32Value));
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &steering_angle.ui32ArduinoTimestamp));

            difference_steering_angle = fabs(last_steering_angle) - fabs(steering_angle.f32Value);
            last_steering_angle = steering_angle.f32Value;
            //debug
            //            LOG_INFO(cString::Format("Steering Angle: %g ", m_f64WheelAngle));

        }




        //calculation
        // if speed = 0 is requested output is immediately set to zero
        if (m_f64SetPoint == 0 || EmergencyBreakFlag == 1 || (m_f64MeasuredVariable == 0 && star_flag == 0))
        {
            m_f64LastOutput = 0;
            m_f64accumulatedVariable = 0;
            m_f64LastMeasuredError = 0;

//            if(m_f64MeasuredVariable == 0)
                star_flag = 1;
        }
        //soft starter
        else if (m_f64SetPoint != 0 && EmergencyBreakFlag == 0 && star_flag == 1)
        {
            m_f64accumulatedVariable = 0;
            m_f64LastMeasuredError = 0;

#ifdef AUTO_A
            if(m_f64SetPoint > 0)
            {
                if(fabs(last_steering_angle) > 50 && fabs(last_steering_angle) <= 60)
                    m_f64LastOutput = -10;
                else if(fabs(last_steering_angle) > 60 && fabs(last_steering_angle) <= 70)
                    m_f64LastOutput = -12;
                else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
                    m_f64LastOutput = -14;
               else if(fabs(last_steering_angle) > 80)
                    m_f64LastOutput = -15;
                else
                    m_f64LastOutput = -MOTOR_START_F_PCT;
            }
            if(m_f64SetPoint < 0)
            {
                if(fabs(last_steering_angle) > 50 && fabs(last_steering_angle) <= 60)
                    m_f64LastOutput = 11;
                else if(fabs(last_steering_angle) > 60 && fabs(last_steering_angle) <= 70)
                    m_f64LastOutput = 12;
                else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
                    m_f64LastOutput = 14;
               else if(fabs(last_steering_angle) > 80)
                    m_f64LastOutput = 16;
                else
                    m_f64LastOutput = MOTOR_START_B_PCT+1;
            }

            if((fabs(m_f64MeasuredVariable) > 0.3 && m_f64SetPoint > 0) || (fabs(m_f64MeasuredVariable) > 0.3 && m_f64SetPoint < 0))
                star_flag = 0;
#else
            if(m_f64SetPoint > 0)
            {
                if(fabs(last_steering_angle) > 50 && fabs(last_steering_angle) <= 60)
                    m_f64LastOutput = -9;
                else if(fabs(last_steering_angle) > 60 && fabs(last_steering_angle) <= 70)
                    m_f64LastOutput = -11;
                else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
                    m_f64LastOutput = -13;
               else if(fabs(last_steering_angle) > 80)
                    m_f64LastOutput = -14;
                else
                    m_f64LastOutput = -MOTOR_START_F_PCT;
            }
            if(m_f64SetPoint < 0)
            {
                if(fabs(last_steering_angle) > 50 && fabs(last_steering_angle) <= 60)
                    m_f64LastOutput = 11;
                else if(fabs(last_steering_angle) > 60 && fabs(last_steering_angle) <= 70)
                    m_f64LastOutput = 12;
                else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
                    m_f64LastOutput = 14;
               else if(fabs(last_steering_angle) > 80)
                    m_f64LastOutput = 16;
                else
                    m_f64LastOutput = MOTOR_START_B_PCT+1;
            }

            if((fabs(m_f64MeasuredVariable) > 0.3 && m_f64SetPoint > 0) || (fabs(m_f64MeasuredVariable) > 0.3 && m_f64SetPoint < 0))
                star_flag = 0;
#endif

        }
        else
        {

            if (m_i32ControllerMode == 4)
                m_f64LastOutput = getControllerValue(m_f64MeasuredVariable)*m_f64PT1OutputFactor;
            else
                m_f64LastOutput = getControllerValue(m_f64MeasuredVariable);

            m_f64LastOutput = ConvertSpeedToPercent(m_f64LastOutput);


        }

        // speed limit
        if(fabs(m_f64MeasuredVariable) > 2.0)
            m_f64LastOutput = 0;

        outputValue = static_cast<tFloat32>(m_f64LastOutput);
        transmitSignalValue(m_oOutputActuator, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, outputValue);

        //for debug
        //        LOG_INFO(cString::Format("Sample Time= %.3f  SetPoint= %.2f  Speed= %.2f  Emergency Break= %g  Output= %g", f64SampleTime, m_f64SetPoint, m_f64MeasuredVariable, EmergencyBreakFlag, outputValue));
        //        LOG_INFO(cString::Format("Sample Time= %.3f  SetPoint= %.2f  Speed= %.2f  Steering angle= %g  Output= %g", f64SampleTime, m_f64SetPoint, m_f64MeasuredVariable, m_f64WheelAngle, outputValue));

    }


    //if (m_bShowDebug)
    //{
    //    SendSignalData();
    //}

    RETURN_NOERROR;
}


//tResult cOptIlm_WheelSpeedController::SendSignalData()
//{
//    __synchronized_kernel(m_oLock);
//    tTimeStamp tsStreamTime = _clock->GetStreamTime();
//
//    for (tActiveSignals::iterator oSignal = m_oActive.begin();
//         oSignal != m_oActive.end();
//         ++oSignal)
//    {
//        if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_MEASVALUE)
//        {
//            tSignalValue sValue;
//            sValue.nRawValue = 0;
//            sValue.nTimeStamp = tsStreamTime;
//            sValue.strTextValue = 0;
//            sValue.f64Value = m_f64MeasuredVariable;
//            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
//        }
//        else if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_SETVALUE)
//        {
//            tSignalValue sValue;
//            sValue.nRawValue = 0;
//            sValue.nTimeStamp = tsStreamTime;
//            sValue.strTextValue = 0;
//            sValue.f64Value = m_f64SetPoint;
//
//            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
//        }
//
//        else if (*oSignal == WSC_SIGREG_ID_WHEELSPEED_OUTPUTVALUE)
//        {
//            tSignalValue sValue;
//            sValue.nRawValue = 0;
//            sValue.nTimeStamp = tsStreamTime;
//            sValue.strTextValue = 0;
//            sValue.f64Value = m_f64LastOutput;
//
//            m_pISignalRegistry->UpdateSignal(this, *oSignal, sValue);
//        }
//    }
//    RETURN_NOERROR;
//}

tFloat64 cOptIlm_WheelSpeedController::
getControllerValue(tFloat64 i_f64MeasuredValue)
{

    //i_f64MeasuredValue = (i_f64MeasuredValue +  m_f64LastSpeedValue) /2.0;

    //m_f64LastSpeedValue = i_f64MeasuredValue;

    tFloat f64Result = 0;




    //the three controller algorithms
    if (m_i32ControllerMode == 1)
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //y = Kp * e
        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);

        f64Result = m_f64PIDKp * f64Error;
    }
    else if (m_i32ControllerMode == 2) //PI- Regler
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:

        tFloat64 temp_m_f64accumulatedVariable = 0;;

        //        if(fabs(steering_angle.f32Value) < 40)
        temp_m_f64accumulatedVariable = (f64Error*f64SampleTime);
        //        else if(fabs(steering_angle.f32Value) > 40 && fabs(steering_angle.f32Value) < 65)
        //            temp_m_f64accumulatedVariable = (f64Error*f64SampleTime) * 0.8;
        //        else if(fabs(steering_angle.f32Value) > 65)
        //            temp_m_f64accumulatedVariable = (f64Error*f64SampleTime) * 0.7;


        //        m_f64accumulatedVariable += (f64Error*m_f64PIDSampleTime);
        if(m_f64SetPoint > 0)
        {
            if(outputValue < MOTOR_B_GAS_LIMIT && outputValue > MOTOR_F_GAS_LIMIT)
                m_f64accumulatedVariable += temp_m_f64accumulatedVariable;
            else
                m_f64accumulatedVariable = m_f64accumulatedVariable;
        }
        else
        {
            if(outputValue < MOTOR_B_GAS_LIMIT && outputValue > MOTOR_F_GAS_LIMIT)
                m_f64accumulatedVariable += temp_m_f64accumulatedVariable;
            else
                m_f64accumulatedVariable = m_f64accumulatedVariable;

        }



        //        if(difference_steering_angle > 20 && difference_steering_angle < 30)
        //            temp_m_f64accumulatedVariable = temp_m_f64accumulatedVariable * 0.7;
        //        else if(difference_steering_angle > 30)
        //            temp_m_f64accumulatedVariable = temp_m_f64accumulatedVariable * 0.6;




        f64Result = m_f64PIDKp * f64Error + (m_f64PIDKi*m_f64accumulatedVariable);

    }
    else if (m_i32ControllerMode == 3)
    {
        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e ? ealt)/Ta
        //ealt = e

        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:
        //        m_f64accumulatedVariable += f64Error * m_f64PIDSampleTime;

        if(outputValue < MOTOR_B_GAS_LIMIT && outputValue > MOTOR_F_GAS_LIMIT)
            m_f64accumulatedVariable += (f64Error * f64SampleTime);

        f64Result = m_f64PIDKp * f64Error
                + (m_f64PIDKi*m_f64accumulatedVariable)
                + m_f64PIDKd * (f64Error - m_f64LastMeasuredError) / f64SampleTime;

        m_f64LastMeasuredError = f64Error;
    }
    else if (m_i32ControllerMode == 4)
    {
        /*********************************
        * PT 1 discrete algorithm
        *
        *               Tau
        *       In +    ---  * LastOut
        *             Tsample
        * Out = ---------------------
        *               Tau
        *       1 +     ---
        *             Tsample
        *
        *                           Tau
        * here with     PT1Gain =   ---
        *                         Tsample
        *
        *                  T
        * y(k) = y(k-1) + --- ( v * e(k)  - y(k-1))
        *                  T1
        *
        * e(k):  input
        * y(k-1) : last output
        * v : gain
        * T/T1: time constant
        ********************************************/
        f64Result = m_f64LastOutput + m_f64PT1TimeConstant * (m_f64PT1Gain *
                                                              (m_f64SetPoint - i_f64MeasuredValue) - m_f64LastOutput);

    }

    m_f64LastOutput = f64Result;

    return f64Result;
}

tTimeStamp cOptIlm_WheelSpeedController::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

tFloat64 cOptIlm_WheelSpeedController::ConvertSpeedToPercent(tFloat64 speed)
{
    tFloat64 output_percent = 0;


    if(speed == 0)
    {
        output_percent = 0;
    }
    else if(speed > 0)
    {
        //        if(fabs(last_steering_angle) > 40 && fabs(last_steering_angle) <= 75)
          //            output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + (fabs(last_steering_angle) * 0.02)));
          //        else if(fabs(last_steering_angle) > 75 && fabs(last_steering_angle) <= 85)
          //            output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + (fabs(last_steering_angle) * 0.025)));
          //        else if(fabs(last_steering_angle) > 85)
          //            output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + (fabs(last_steering_angle) * 0.028)));
          //        else
          if (last_steering_angle >= 0)
          {
              //            if(fabs(last_steering_angle) > 40 && fabs(last_steering_angle) <= 70)
              //                output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + 1));
              //            else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
              //                output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + 2));
              //            else if(fabs(last_steering_angle) > 80)
              //                output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + 3));
              if(fabs(last_steering_angle) > 45 && fabs(last_steering_angle) <= 70)
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + (fabs(last_steering_angle) * 0.02)));
              else if(fabs(last_steering_angle) > 70 && fabs(last_steering_angle) <= 80)
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + (fabs(last_steering_angle) * 0.024)));
              else if(fabs(last_steering_angle) > 80)
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + (fabs(last_steering_angle) * 0.028)));
              else
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F));
          }
          else /*if (last_steering_angle < 0)*/
          {
              if(fabs(last_steering_angle) > 60 && fabs(last_steering_angle) <= 80)
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + 1));
              else if(fabs(last_steering_angle) > 80 /*&& fabs(last_steering_angle) <= 85*/)
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + 2));
//              else if(fabs(last_steering_angle) > 85)
//                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F + 2));
              else
                  output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F));
          }
          //        else
          //            output_percent = speed + (m_f64SetPoint * (MS_TO_PRC_F));
          //LOG_INFO(cString::Format("Steering Angle: %g ", last_steering_angle));
          output_percent = Limit_Set(-output_percent, MOTOR_F_BRAKE_LIMIT, MOTOR_F_GAS_LIMIT);  }
    else if(speed < 0)
    {
        output_percent = speed + (m_f64SetPoint * MS_TO_PRC_B);
        output_percent = Limit_Set(-output_percent, MOTOR_B_GAS_LIMIT, MOTOR_B_BRAKE_LIMIT);

    }


    //    // checking for minimum and maximum limits
    //    if (f64Result > m_f64PIDMaximumOutput)
    //    {
    //        f64Result = m_f64PIDMaximumOutput;
    //    }
    //    else if (f64Result < m_f64PIDMinimumOutput)
    //    {
    //        f64Result = m_f64PIDMinimumOutput;
    //    }

    return output_percent;
}


tFloat64 cOptIlm_WheelSpeedController::Limit_Set(tFloat64 value, tFloat64 max, tFloat64 min)
{
    if(value >= max)
        return max;
    else if(value <= min)
        return min;

    return value;
}
