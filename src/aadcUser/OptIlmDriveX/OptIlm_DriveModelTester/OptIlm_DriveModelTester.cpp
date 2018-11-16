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

#include "OptIlm_DriveModelTester.h"
#include "ADTF3_helper.h"
//*************************************************************************************************


ADTF_PLUGIN("OptIlm_DriveModelTester Plugin", cOptIlm_DriveModelTester)


cOptIlm_DriveModelTester::cOptIlm_DriveModelTester() : m_pUiFileWidget(nullptr)
{
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    create_pin(*this, m_oOutputDriveModel, "Drive_model", pTypeSignalValue);




    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue")              , m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    create_pin(*this, m_oOutputHeadLight   , "head_light"        , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputTurnLeft    , "turn_signal_left"  , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputTurnRight   , "turn_signal_right" , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputBrakeLight  , "brake_light"       , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputHazard      , "hazard_light"      , pTypeBoolSignalValue);
    create_pin(*this, m_oOutputReverseLight, "reverse_light"     , pTypeBoolSignalValue);

}


cOptIlm_DriveModelTester::~cOptIlm_DriveModelTester()
{

}

QWidget* cOptIlm_DriveModelTester::CreateView()
{
    // use single UI File in background
    m_pUiFileWidget = new cCarControllerWidget(nullptr);

    connect(m_pUiFileWidget->getLightButtonGroup(), SIGNAL(buttonClicked(int))     , this, SLOT(ToggleLights(int)));
    connect(m_pUiFileWidget                       , SIGNAL(buttonClicked(int))     , this, SLOT(ToggleLights(int)));

    ToggleLights(6);
    return m_pUiFileWidget;
}

tVoid cOptIlm_DriveModelTester::ReleaseView()
{
    delete m_pUiFileWidget;
    m_pUiFileWidget = nullptr;
}

tResult cOptIlm_DriveModelTester::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cOptIlm_DriveModelTester::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);
    
    RETURN_NOERROR;
}

tResult cOptIlm_DriveModelTester::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult cOptIlm_DriveModelTester::SendDriveModel(int value)
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    transmitSignalValue(m_oOutputDriveModel, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);

    RETURN_NOERROR;
}


tResult cOptIlm_DriveModelTester::ToggleLights(int buttonId)
{
    static bool headToggle;
    static bool reverseToggle;
    static bool brakeToggle;
    static bool turnRightToggle;
    static bool turnLeftToggle;
    static bool hazzardLightToggle;

//    LOG_INFO(cString::Format("--------------------------buttonId: %d", buttonId));
    switch (buttonId)
    {
        case 0: // Head
            transmitBoolSignalValue(m_oOutputHeadLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !headToggle);
            headToggle = !headToggle;
            LOG_INFO(cString::Format("Heads toggled: %d", headToggle));
            break;
        case 1: // Brake
            transmitBoolSignalValue(m_oOutputBrakeLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !brakeToggle);
            brakeToggle = !brakeToggle;
            LOG_INFO(cString::Format("Brake toggled: %d", brakeToggle));
            break;
        case 2: // Reverse
            transmitBoolSignalValue(m_oOutputReverseLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !reverseToggle);
            reverseToggle = !reverseToggle;
            LOG_INFO(cString::Format("Reverse toggled: %d", reverseToggle));
            break;
        case 3: // Hazard
            transmitBoolSignalValue(m_oOutputHazard, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !hazzardLightToggle);
            hazzardLightToggle = !hazzardLightToggle;
            LOG_INFO(cString::Format("Hazard toggled: %d", hazzardLightToggle));
            break;
        case 4: // Left
            transmitBoolSignalValue(m_oOutputTurnLeft, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !turnLeftToggle);
            turnLeftToggle = !turnLeftToggle;
            LOG_INFO(cString::Format("Turn Left toggled: %d", turnLeftToggle));
            break;
        case 5: // Right
            transmitBoolSignalValue(m_oOutputTurnRight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, !turnRightToggle);
            turnRightToggle = !turnRightToggle;
            LOG_INFO(cString::Format("Turn right toggled: %d", turnRightToggle));
            break;


        // Drive Model
        case 6: //Stop
            SendDriveModel(CAR_STOP);
            LOG_INFO(cString::Format("Drive Model: Stop   %d", CAR_STOP));
            break;
        case 7: //Lane Keeping
            SendDriveModel(LANE_KEEPING);
            LOG_INFO(cString::Format("Drive Model: Lane Keeping   %d", LANE_KEEPING));
            break;
        case 8: //TurnLeft
            SendDriveModel(TURN_LEFT);
            LOG_INFO(cString::Format("Drive Model: Turn left   %d", TURN_LEFT));
            break;
        case 9: //TurnRight
            SendDriveModel(TURN_RIGHT);
            LOG_INFO(cString::Format("Drive Model: turn right   %d", TURN_RIGHT));
            break;
        case 10: //Straight
            SendDriveModel(STRAIGHT);
            LOG_INFO(cString::Format("Drive Model: straight   %d", STRAIGHT));
            break;
        case 11: //Parking
            SendDriveModel(PARKING);
            LOG_INFO(cString::Format("Drive Model: parking   %d", PARKING));
            break;
        case 12: //Pull out left
            SendDriveModel(PULL_OUT_LEFT);
            LOG_INFO(cString::Format("Drive Model: Pull out left    %d", PULL_OUT_LEFT));
            break;
        case 13: //Pull out right
            SendDriveModel(PULL_OUT_RIGHT);
            LOG_INFO(cString::Format("Drive Model: pull out right   %d", PULL_OUT_RIGHT));
            break;
        case 14: //Merge left
            SendDriveModel(MERGE_LEFT);
            LOG_INFO(cString::Format("Drive Model: merge left   %d", MERGE_LEFT));
            break;

        default:
            break;
    }

    RETURN_NOERROR;
}
