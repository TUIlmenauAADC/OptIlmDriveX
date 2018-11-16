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

#include "Optilm_AutonomousDriving.h"
#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"
#include<pthread.h>







ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_OPTILM_AUTONOMOUS_DRIVING_DATA_TRIGGERED_FILTER,
                                    "Optilm AutonomousDriving",
                                    cOptilm_AutonomousDriving,
                                    adtf::filter::thread_trigger(true))



cOptilm_AutonomousDriving::cOptilm_AutonomousDriving()
{
    SetName("Autonomous Driving");

    //register properties

    RegisterPropertyVariable("1.DrivingModelTest", m_Driving_Model_Test);
    RegisterPropertyVariable("1.Demo Model", m_Demo_Model);
    RegisterPropertyVariable("2.EmergencyBreak: a.Front Minimum break time(S)", m_front_min_break_time);
    RegisterPropertyVariable("2.EmergencyBreak: b.Rear Minimum break time(s)", m_rear_min_break_time);
    RegisterPropertyVariable("2.EmergencyBreak: c.Minimum break distance(cm)", m_min_break_distance);
    RegisterPropertyVariable("3.VehicleSpeed: a.Minimum speed", m_vehicle_min_speed);
    RegisterPropertyVariable("3.VehicleSpeed: b.Maximum speed", m_vehicle_max_speed);
    RegisterPropertyVariable("4.Parking: a.Road sign distance(cm)", parking_road_sign_distance);
    RegisterPropertyVariable("4.Parking: b.Slot distance 1 (cm)", slot_distance[0]);
    RegisterPropertyVariable("4.Parking: c.Slot distance 2 (cm)", slot_distance[1]);
    RegisterPropertyVariable("4.Parking: d.Slot distance 3 (cm)", slot_distance[2]);
    RegisterPropertyVariable("4.Parking: e.Slot distance 4 (cm)", slot_distance[3]);
    RegisterPropertyVariable("5.Crossing: a.Road sign distance(cm)", crossing_road_sign_distance);
    RegisterPropertyVariable("5.Crossing: b.Stop line distance(cm)", crossing_stop_line_distance);
    RegisterPropertyVariable("5.Crossing: c.TurnLeft stop distance(cm)", stop_crossing_left);
    RegisterPropertyVariable("5.Crossing: d.TurnLeft no stop distance(cm)", nostop_crossing_left);
    RegisterPropertyVariable("5.Crossing: e.TurnRight stop distance(cm)", stop_crossing_right);
    RegisterPropertyVariable("5.Crossing: f.TurnRight no stop distance(cm)", nostop_crossing_right);
    RegisterPropertyVariable("5.Crossing: g.TurnStraight stop distance(cm)", stop_crossing_strgiht);
    RegisterPropertyVariable("5.Crossing: h.TurnStraight no stop distance(cm)", nostop_crossing_strgiht);
    RegisterPropertyVariable("6.Avoidance: a.Obstacle detection distance(cm)", obstacle_detection_distance);
    RegisterPropertyVariable("6.Avoidance: b.Avoidance start distance(cm)", avoidance_start_distance);
    RegisterPropertyVariable("6.Avoidance: c.Side detection distance(cm)", avoidance_side_distance);
    RegisterPropertyVariable("6.Avoidance: d.Come back counter(cm)", avoidance_comeBack_counter);
    RegisterPropertyVariable("7.Pedestrian: a.Road sign distance(cm)", pedestrian_road_sign_distance);
    RegisterPropertyVariable("7.Pedestrian: b.Overall distance(cm)", pedestrian_low_speed_distance);



    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register output pin
    Register(m_oVideo_Output, "Video_output", pType);



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
    //Input
    Register(m_oInput_VehicleSpeed, "VehicleSpeed" , pTypeSignalValue);
    Register(m_oInput_SteeringAngle, "SteeringAngle" , pTypeSignalValue);
    Register(m_oInput_TestDriveingModel, "DrivingModelFromTester", pTypeSignalValue);
    Register(m_oInput_DistanceOverall, "DistanceOverall", pTypeSignalValue);

    //Output
    Register(m_oOutput_DriveingModel, "DrivingModel", pTypeSignalValue);
    Register(m_oOutput_EmergencyBreakFlag, "EmergencyBreakFlag", pTypeSignalValue);
    Register(m_oOutput_Image_control_mode, "ImageControlMode" , pTypeSignalValue);


    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory, cString("bValue")              , m_ddlBoolSignalValueId.bValue));
    }
    else
    {
        LOG_INFO("No mediadescription for tBool found!");
    }

    Register(m_oOutputHeadLight,    "head_light" , pTypeBoolSignalValue);
    Register(m_oOutputTurnLeft,     "turn_signal_left" , pTypeBoolSignalValue);
    Register(m_oOutputTurnRight,    "turn_signal_right" , pTypeBoolSignalValue);
    Register(m_oOutputBrakeLight,   "brake_light" , pTypeBoolSignalValue);
    Register(m_oOutputHazard,       "hazard_light" , pTypeBoolSignalValue);
    Register(m_oOutputReverseLight, "reverse_light" , pTypeBoolSignalValue);


    //Add position pin from mediadescription
    object_ptr<IStreamType> pTypeLaneData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaneCurveData", pTypeLaneData, m_LaneDataStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32LaneDetectMode"), m_ddlLaneDataId.laneDetectMode));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32LandModel_k"), m_ddlLaneDataId.parameter_k));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32LandModel_m"), m_ddlLaneDataId.parameter_m));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32LandModel_b"), m_ddlLaneDataId.parameter_b));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32LaneWidth"), m_ddlLaneDataId.laneWidth));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32L_SL_LorR"), m_ddlLaneDataId.leftOrRightLine));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32Adult_flag"), m_ddlLaneDataId.adult_flag));
        (adtf_ddl::access_element::find_index(m_LaneDataStructSampleFactory, cString("f32Child_flag"), m_ddlLaneDataId.child_flag));
    }
    else
    {
        LOG_ERROR("No mediadescription for tLaneCurveData found!");
    }
    //    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;
    //Create Pin
    Register(m_oInput_LaneData, "LaneData" , pTypeLaneData);


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
        LOG_ERROR("No mediadescription for tLaserScannerData found!");
    }
    Register(m_oInput_LaserScanner, "LaserScanner" , pTypeLSData);



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
        LOG_ERROR("No mediadescription for tUltrasonicStruct found!");
    }
    Register(m_oInput_UltrasonicUnit, "UltrasonicStruct" , pTypeUSData);


    //Add position pin from mediadescription
    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32x"), m_ddlPositionIndex.x));
        (adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32y"), m_ddlPositionIndex.y));
        (adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32radius"), m_ddlPositionIndex.radius));
        (adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32speed"), m_ddlPositionIndex.speed));
        (adtf_ddl::access_element::find_index(m_PositionSampleFactory, cString("f32heading"), m_ddlPositionIndex.heading));
    }
    else
    {
        LOG_ERROR("No mediadescription for tPosition found!");
    }
    //    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;
    //Create Pin
    Register(m_oInput_Position, "Position" , pTypePositionData);


    //the road sign data struct
    object_ptr<IStreamType> pTypeRoadSignData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRoadSignData", pTypeRoadSignData, m_RoadSignDataSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_RoadSignDataSampleFactory, "i16Identifier", m_ddlRoadSignDataIndex.id);
        adtf_ddl::access_element::find_index(m_RoadSignDataSampleFactory, "f32Distance", m_ddlRoadSignDataIndex.distance);
        adtf_ddl::access_element::find_index(m_RoadSignDataSampleFactory, "f32Lateral", m_ddlRoadSignDataIndex.Lateral);
        adtf_ddl::access_element::find_index(m_RoadSignDataSampleFactory, "f32Direction", m_ddlRoadSignDataIndex.direction);
        adtf_ddl::access_element::find_index(m_RoadSignDataSampleFactory, "f32RoadSign_X", m_ddlRoadSignDataIndex.RoadSign_X);
        adtf_ddl::access_element::find_index(m_RoadSignDataSampleFactory, "f32RoadSign_Y", m_ddlRoadSignDataIndex.RoadSign_Y);
    }
    else
    {
        LOG_WARNING("No mediadescription for tRoadSignData found!");
    }

    //register output pose pin
    Register(m_oInput_RoadSignData, "RoadSignData", pTypeRoadSignData);



    //get the media description
    object_ptr<IStreamType> pTypeTrajectoryPoint;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLTrajectoryPoint", pTypeTrajectoryPoint, m_TrajectoryPointSampleFactory))
    {
        (adtf_ddl::access_element::find_array_index(m_TrajectoryPointSampleFactory, "tPointArray",   m_ddlTrajectoryPointDataId.pointArray));
    }
    else
    {
        LOG_ERROR("Could not load media description for output pin Trajectory Point");
    }

    //register output pose pin
    Register(m_oOutput_trajectory_point, "TrajectoryPoint", pTypeTrajectoryPoint);


    //Get Media Descriptions
    object_ptr<IStreamType> pTypeJuryStruct;
    cString structName = "tJuryStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeJuryStruct, m_juryStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ActionID"), m_ddlJuryStructId.actionId));
        (adtf_ddl::access_element::find_index(m_juryStructSampleFactory, cString("i16ManeuverEntry"), m_ddlJuryStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    Register(m_oInputJuryStruct, "jury_struct", pTypeJuryStruct);

    object_ptr<IStreamType> pTypeDriverStruct;
    structName = "tDriverStruct";
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(structName.GetPtr(), pTypeDriverStruct, m_driverStructSampleFactory))
    {
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));
    }
    else
    {
        LOG_WARNING(cString::Format("No mediadescription for %s found!", structName.GetPtr()));
    }
    Register(m_oOutputDriverStruct, "driver_struct", pTypeDriverStruct);

    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    Register(m_oInputManeuverList, "maneuver_list", pTypeDefault);


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
    Register(m_oInput_Demo_maneuver_list, "Demo_ManeuverListr", pTypeManeuverListr);




    system_initial = false;
    m_lastSampleTime = 0;
    last_driving_model_flag = CAR_STOP;

}


tResult cOptilm_AutonomousDriving::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    
    RETURN_NOERROR;
}

float temp_last_driving_model_flag = CAR_STOP;
tResult cOptilm_AutonomousDriving::Process(tTimeStamp tmTimeOfTrigger)
{
    // avoid that the method triggers itself due to the feedback loop
    //    std::lock_guard<std::mutex> oGuard(m_oMutex);

    tTimeStamp m_NowSampleTime =  tmTimeOfTrigger;
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;


    //    //Get data from pin
    //    GetLaneData();
    //    GetPosition();
    //    GetCarSpeed();
    //    GetSteeringAngle();
    //    GetDistanceOverall();
    //    GetRoadSignData();
    //    GetUltrasonicUnit();
    //    GetLaserScanner();


    if(m_Driving_Model_Test == tFalse && system_initial == true)
    {
        if(m_Demo_Model == tTrue)
            GetDemoManeuverList();
        else
            GetManeuverList();


        GetJulyState();
    }


    if(f64SampleTime > 0.05)
    {
        m_lastSampleTime = m_NowSampleTime;

        //Filter sample time test for debug
        //        LOG_INFO(cString::Format("StateControl Sample Time = %.3f S", f64SampleTime));

        //Get data from pin
        if(system_initial == true)
        {
            GetLaneData();
            GetPosition();
            GetCarSpeed();
            GetSteeringAngle();
            GetDistanceOverall();
            GetRoadSignData();
            GetUltrasonicUnit();
            GetLaserScanner();
        }



        if(m_Driving_Model_Test)
        {
            GetDrivingModelFromTester();
            last_driving_model_flag = StateMachineFromTester(last_driving_model_flag);
        }
        else
        {
            if(ManeuverList.ready_flag == tTrue)
                last_driving_model_flag = StateMachine(last_driving_model_flag);
            else
                last_driving_model_flag = CAR_STOP;

        }


        if(system_initial == false)
        {
            SystemInitial();
        }
        else
        {
            //for Debug
            if(temp_last_driving_model_flag != last_driving_model_flag)
            {
                temp_last_driving_model_flag = last_driving_model_flag;
                LOG_INFO(cString::Format("last_driving_model_flag = %g", last_driving_model_flag));
            }

            //State Control function
            last_driving_model_flag = StateControl(last_driving_model_flag);
            TransmitDataFusionVideo();

//            if (log_file) fprintf(log_file,"%.3f   %.3f\n",vehicle_speed.f32Value, lane.kurve_parameter[B]);

        }



    }
    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::SystemInitial()
{      
    position_initial_flag = tFalse;

    ResetLights();
    ResetValue();


    ManeuverList.ready_flag = tFalse;
    ManeuverList.number_of_id = 0;
    ManeuverList.current_id = 0;
    ManeuverList.state = CAR_STOP;


    image_processing_function_switch = LANE_DETECTION;


//    SetDigitialMapRegion();
    //    SetDigitialMapRegionTestEvent();
        SetDigitialMapRegionFinal();
//    SetIntersectionGraph();
//    SetIntersectionGraphAugumented();
  if(m_Demo_Model == tTrue)
    SetIntersectionGraphDemo();
    else
     SetIntersectionGraphFinal();


    system_initial = true;

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::ResetValue()
{
    number_of_reference_point = 12;
    laser_scann.number_of_scan_point = 0;

    avoidance.comeback_flag = 0;
    avoidance.flag = false;
    avoidance_car_HeadingAngle = 0;

    find_obstacle = false;
    S_CurveDecisionflag = false;

    maneuverid_flag = false;

    memset(path_reference_point.X, 0, 20*sizeof(float));
    memset(path_reference_point.Y, 0, 20*sizeof(float));
    vehicle_target_speed = 0;

    crossing_flag = CROSSING_FLAG_OFF;
    crossing_stop_wait_counter = 0;
    nostop_crossing = 0;
    stop_crossing = 0;
    crossing_sign_found = false;

    pedestrian_flag = false;
    pedestrian_stop_counter = 0;

    T_crossing_current_index = 0;
    T_crossing_direction = -1;
    roundabout_flag = false;
    roundabout_in = false;

    find_side_direction = -1;


    emergency_vehicle_shift = 0;
    emergency_vehicle_counter = 0;
    emergency_vehicle_flag = false;
    in_emergency_vehicle_boundary_flag = false;


    ramp_state_flag = NO_RAMP;

    parking_Ready_flag = PARKING_READY_FLAG_OFF;

    road_sign_ID = NO_TRAFFIC_SIGN;
    road_sign_distance = 0;
    road_sign_lateral  = 0;

    path_reference_counter = 0;
    avoidance_permit_flag = 0;

    temp_road_sign_X = ROAD_SIGN_NOT_IN_XML;
    temp_road_sign_Y = ROAD_SIGN_NOT_IN_XML;
    temp_car_direction_heading = 0;


    parking_temp_road_sign_X = ROAD_SIGN_NOT_IN_XML;
    parking_temp_road_sign_Y = ROAD_SIGN_NOT_IN_XML;
    parking_sign_found = false;


    space_id = 0;
    space_id_counter = 0;

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::ResetLights()
{
    light_flag[HEAD   ] = tTrue;
    light_flag[BRAKE  ] = tTrue;
    light_flag[REVERSE] = tTrue;
    light_flag[HAZARD ] = tTrue;
    light_flag[LEFT   ] = tTrue;
    light_flag[RIGHT  ] = tTrue;
    ToggleLights(HEAD, tFalse);
    ToggleLights(BRAKE, tFalse);
    ToggleLights(REVERSE, tFalse);
    ToggleLights(HAZARD, tFalse);
    ToggleLights(LEFT, tFalse);
    ToggleLights(RIGHT, tFalse);
    ToggleLights(HEAD, tTrue);
    pull_out_light_counter = 0;

    RETURN_NOERROR;
}





//////////////////////////////////////////////////////////
/// Get Pin data
///
///
tResult cOptilm_AutonomousDriving::GetDrivingModelFromTester()
{
    float model_flag = 0;
    object_ptr<const ISample> pReadSample;
    while (IS_OK(m_oInput_TestDriveingModel.GetNextSample(pReadSample)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &model_flag));

        last_driving_model_flag = model_flag;
        LOG_INFO(cString::Format("last_driving_model_flag %g",last_driving_model_flag));
    }


    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetLaneData()
{
    object_ptr<const ISample> pSampleFromLaneData;


    if (IS_OK(m_oInput_LaneData.GetLastSample(pSampleFromLaneData)))
    {
        auto oDecoder = m_LaneDataStructSampleFactory.MakeDecoderFor(*pSampleFromLaneData);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.laneDetectMode, &lane.detect_mode));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.parameter_k, &lane.kurve_parameter[K]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.parameter_m, &lane.kurve_parameter[M]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.parameter_b, &lane.kurve_parameter[B]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.laneWidth, &lane.width));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.leftOrRightLine, &lane.left_or_right_line));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.adult_flag, &lane.adult_flag));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneDataId.child_flag, &lane.child_flag));

        //show lane dat pin data for debug
        //        LOG_INFO(cString::Format("mode %g k:%g m:%g b:%g width:%g ", lane.detect_mode, lane.kurve_parameter[K], lane.kurve_parameter[M], lane.kurve_parameter[B], lane.width));
        //         LOG_INFO(cString::Format("lane.left_or_right_line %g",lane.left_or_right_line));

    }

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetPosition()
{

    object_ptr<const ISample> pReadSampleFromPosition;
    if (IS_OK(m_oInput_Position.GetLastSample(pReadSampleFromPosition)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSampleFromPosition);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &position_data[X]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &position_data[Y]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &position_data[SPEED]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &position_data[HEADING]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &position_data[RADIUS]));

        position_data[X]   = cos(position_data[HEADING]) * 0.18 + position_data[X];
        position_data[Y]   = sin(position_data[HEADING]) * 0.18 + position_data[Y];

        position_initial_flag = tTrue;

        //show position pin data for debug
        //        LOG_INFO(cString::Format("position x:%g y:%g heading:%g speed:%g radius:g ", Nmpc->car_position.X_Position, Nmpc->car_position.Y_Position, Nmpc->car_position.HeadingAngle, Nmpc->car_position.speed, Nmpc->car_position.radius));
    }


    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetCarSpeed()
{

    object_ptr<const ISample> pSampleFromVcc;


    if (IS_OK(m_oInput_VehicleSpeed.GetLastSample(pSampleFromVcc)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromVcc);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &vehicle_speed.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &vehicle_speed.ui32ArduinoTimestamp));

    }

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetSteeringAngle()
{

    object_ptr<const ISample> pSampleFromSteering;


    if (IS_OK(m_oInput_SteeringAngle.GetLastSample(pSampleFromSteering)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromSteering);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &steering_angle.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &steering_angle.ui32ArduinoTimestamp));

    }
    CalculateVehiclePath(-steering_angle.f32Value);
    //for debug
    //    LOG_INFO(cString::Format("Steering Angle: %g ", steering_angle.f32Value));

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetDistanceOverall()
{

    object_ptr<const ISample> pSampleFromDistance;


    if (IS_OK(m_oInput_DistanceOverall.GetLastSample(pSampleFromDistance)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromDistance);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &distance_overall.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &distance_overall.ui32ArduinoTimestamp));

    }

    RETURN_NOERROR;
}


tResult cOptilm_AutonomousDriving::GetRoadSignData()
{

    object_ptr<const ISample> pSampleFromRoadSign;
    if (IS_OK(m_oInput_RoadSignData.GetLastSample(pSampleFromRoadSign)))
    {
        auto oDecoder = m_RoadSignDataSampleFactory.MakeDecoderFor(*pSampleFromRoadSign);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignDataIndex.id, &road_sign_ID));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignDataIndex.distance, &road_sign_distance));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignDataIndex.Lateral, &road_sign_lateral));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignDataIndex.direction, &road_sign_direction));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignDataIndex.RoadSign_X, &road_sign_X));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlRoadSignDataIndex.RoadSign_Y, &road_sign_Y));
    }


    //for debug
    //        LOG_INFO(cString::Format("Road Sign ID %d, Distance %.2f, Lateral %.2f, Sign position x %.2f, Sign position y %.2f", road_sign_ID, road_sign_distance, road_sign_lateral,road_sign_X,road_sign_Y));

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetUltrasonicUnit()
{
    object_ptr<const ISample> pSampleFromUS;

    if (IS_OK(m_oInput_UltrasonicUnit.GetLastSample(pSampleFromUS)))
    {
        auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

        RETURN_IF_FAILED(oDecoderUS.IsValid());


        // retrieve the values (using convenience methods that return a variant)
        //we do not need the timestamps here
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value, &US_data.tSideLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value, &US_data.tSideRight.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value, &US_data.tRearLeft.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value, &US_data.tRearCenter.f32Value));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value, &US_data.tRearRight.f32Value));

    }

    //debug
    //    LOG_INFO(cString::Format("Side left = %f, Side right = %f, Rear left = %f, Rear center = %f, Rear right = %f", US_data.tSideLeft.f32Value,US_data.tSideRight.f32Value,US_data.tRearLeft.f32Value,US_data.tRearCenter.f32Value,US_data.tRearRight.f32Value));


    if(US_data.tSideLeft.f32Value == -1)
        US_data.tSideLeft.f32Value = 400;
    if(US_data.tSideRight.f32Value == -1)
        US_data.tSideRight.f32Value = 400;
    if(US_data.tRearLeft.f32Value == -1)
        US_data.tRearLeft.f32Value = 400;
    if(US_data.tRearCenter.f32Value == -1)
        US_data.tRearCenter.f32Value = 400;
    if(US_data.tRearRight.f32Value == -1)
        US_data.tRearRight.f32Value = 400;


    CalculateUltrasonicWorldCoordinate();

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetLaserScanner()
{
    object_ptr<const ISample> pSampleFromLS;
    if (IS_OK(m_oInput_LaserScanner.GetLastSample(pSampleFromLS)))
    {
        //Angle is Degree
        //radius is mm

        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pSampleFromLS);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tResult res = oDecoder.GetElementValue(m_ddlLSDataId.size, &laser_scann.number_of_scan_point);

        const tPolarCoordiante* pCoordinates = reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));
        float radian_angel = 0;
        for (int index = 0; index < laser_scann.number_of_scan_point; ++index)
        {
            if(pCoordinates[index].f32Angle == 0 && pCoordinates[index].f32Radius == 0)
            {
                laser_scann.coordinate[index].X = 0;  //mm to Cm
                laser_scann.coordinate[index].Y = 0;
            }
            else
            {
                radian_angel = pCoordinates[index].f32Angle * DEGREES_TO_RADIAN;
                laser_scann.coordinate[index].X = (pCoordinates[index].f32Radius * cos(radian_angel)) * 0.1;  //mm to Cm
                laser_scann.coordinate[index].Y = (pCoordinates[index].f32Radius * sin(radian_angel)) * 0.1;
            }

            //for debug
            //            LOG_INFO(cString::Format("%d Angle = %.2f   x , y =  %.2f , %.2f", index,pCoordinates[index].f32Angle, laser_scann.coordinate[index].X ,laser_scann.coordinate[index].Y));

        }
        //for debug
        //        LOG_INFO(cString::Format("Laser Scanner Number Of Points = %d ", laser_scann.number_of_scan_point));

    }


    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetManeuverList(void)
{
    object_ptr<const ISample> pSampleAnonymous;
    while (IS_OK(m_oInputManeuverList.GetNextSample(pSampleAnonymous)))
    {

        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//maneuverlist
            m_strManeuverFileString.Set(data.data(), data.size());
            LoadManeuverList();
        }
    }

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::LoadManeuverList()
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
                    man.extra = (*itManeuverElem)->GetAttributeUInt32("extra");
                    sector.sector.push_back(man);

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
                    case maneuver_cross_parking:
                        ManeuverList.action[ManeuverList.number_of_id][0] = PARKING;
                        ManeuverList.action[ManeuverList.number_of_id][1] = man.extra;
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
            m_sectorList.push_back(sector);
        }
    }

    ManeuverList.action[ManeuverList.number_of_id][0] = CAR_STOP;
    ManeuverList.action[ManeuverList.number_of_id][1] = 0;

    if (ManeuverList.number_of_id > 0)
    {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
//        RoutePlanning();
//        RoutePlanningFinal();

        for(int index = 0; index < ManeuverList.number_of_id; index++)
        {
            switch (ManeuverList.action[index][0])
            {
            case CAR_STOP:
                LOG_ERROR(adtf_util::cString::Format("Maneuver id: %d ERROR!!!!!", index));
                break;
            case PULL_OUT_LEFT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d pull_out_left", index, intersection_list[index] + 1));
                break;
            case PULL_OUT_RIGHT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d pull_out_right", index, intersection_list[index] + 1));
                break;
            case TURN_LEFT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d left", index, intersection_list[index] + 1));
                break;
            case TURN_RIGHT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d right", index, intersection_list[index] + 1));
                break;
            case STRAIGHT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d straight", index, intersection_list[index] + 1));
                break;
            case MERGE_LEFT:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d merge_left", index, intersection_list[index] + 1));
                break;
            case PARKING:
                LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d cross_parking %d", index, intersection_list[index] + 1, ManeuverList.action[index][1]));
                break;
            }

//            LOG_INFO(adtf_util::cString::Format("Intersection id: %d", intersection_list[index]+1));

        }

        LOG_WARNING(cString::Format("Maneuver End ID: %d action: Car Stop", ManeuverList.number_of_id));

        TransmitDriverStruct(statecar_startup, 0);

    }
    else
    {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
    }

    RETURN_NOERROR;
}


tResult cOptilm_AutonomousDriving::GetDemoManeuverList(void)
{
    object_ptr<const ISample> pReadSample;
    while (IS_OK(m_oInput_Demo_maneuver_list.GetNextSample(pReadSample)))
    {


        auto oDecoder = m_ManeuverListSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        const tManeuverListStruct* ptManeuverList = reinterpret_cast<const tManeuverListStruct*>(oDecoder.GetElementAddress(m_ddlManeuverListDataId.maneuver));
        ManeuverList.number_of_id = 0;
        space_id =  0;
        for (int index = 0; index < 150; index++)
        {
            if(ptManeuverList[index].i16Action == 999 && ptManeuverList[index].i16Action2 == 999)
            {
                ManeuverList.action[ManeuverList.number_of_id][0] = CAR_STOP;
                ManeuverList.action[ManeuverList.number_of_id][1] = 0;
                break;
            }
            else if(ptManeuverList[index].i16Action >= 800 && ptManeuverList[index].i16Action <= 900 )
            {
                space [space_id][0] = ptManeuverList[index].i16Action2;

                switch (space [space_id][0])
                {
                    case HOME:
                        space[space_id][1] = HOME_ID;
                        space[space_id][2] = ptManeuverList[index].i16Action - 800;
                        break;
                    case OFFICE:
                        space[space_id][1] = OFFICE_ID;
                        space[space_id][2] = ptManeuverList[index].i16Action - 800;
                        break;
                    case RESTAURANT:
                        space[space_id][1] = RESTAURANT_ID;
                        space[space_id][2] = ptManeuverList[index].i16Action - 800;
                        break;
                    case POST:
                        space[space_id][1] = POST_ID;
                        space[space_id][2] = ptManeuverList[index].i16Action - 800;
                        break;
                    case SUPERMARKET:
                        space[space_id][1] = SUPERMARKET_ID;
                        space[space_id][2] = ptManeuverList[index].i16Action - 800;
                        break;
                    case GARAGE:
                        space[space_id][1] = 555;
                        space[space_id][2] = ptManeuverList[index].i16Action - 800;
                        break;

                    default:
                        break;
                }
                space_id++;
            }
            else
            {
                ManeuverList.action[index][0] = ptManeuverList[index].i16Action;
                ManeuverList.action[index][1] = ptManeuverList[index].i16Action2;
                ManeuverList.number_of_id++;
            }
        }


        if (ManeuverList.number_of_id > 0)
        {
            LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
//            RoutePlanning();
            RoutePlanningFinal();

            for(int index = 0; index < space_id; index++)
            {
                LOG_INFO(cString::Format("Demo  %d. Ort: %d (%d)  ManeuverList ID %d", index + 1, space[index][0], space[index][1], space[index][2]));
            }


            for(int index = 0; index < ManeuverList.number_of_id; index++)
            {
                switch (ManeuverList.action[index][0])
                {
                case CAR_STOP:
                    LOG_ERROR(adtf_util::cString::Format("Maneuver id: %d ERROR!!!!!", index));
                    break;
                case PULL_OUT_LEFT:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d pull_out_left", index, intersection_list[index] + 1));
                    break;
                case PULL_OUT_RIGHT:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d pull_out_right", index, intersection_list[index] + 1));
                    break;
                case TURN_LEFT:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d left", index, intersection_list[index] + 1));
                    break;
                case TURN_RIGHT:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d right", index, intersection_list[index] + 1));
                    break;
                case STRAIGHT:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d straight", index, intersection_list[index] + 1));
                    break;
                case MERGE_LEFT:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d merge_left", index, intersection_list[index] + 1));
                    break;
                case PARKING:
                    LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d cross_parking %d", index, intersection_list[index] + 1, ManeuverList.action[index][1]));
                    break;
                }

    //            LOG_INFO(adtf_util::cString::Format("Intersection id: %d", intersection_list[index]+1));

            }

            LOG_WARNING(cString::Format("Maneuver End ID: %d action: Car Stop", ManeuverList.number_of_id));

            TransmitDriverStruct(statecar_startup, 0);

            if(space[0][1] == HOME_ID)
                space_id_counter = 1;
            else
                space_id_counter = 0;

        }
        else
        {
            LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        }
    }




    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::GetJulyState()
{
    tDriverStruct driverStruct;
    object_ptr<const ISample> pReadSample;


    while (IS_OK(m_oInputJuryStruct.GetNextSample(pReadSample)))
    {
        auto oDecoder = m_juryStructSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tJuryStruct juryInput;
        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.maneuverEntry, &juryInput.i16ManeuverEntry));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlJuryStructId.actionId, &juryInput.i16ActionID));

        ManeuverList.state = (int)juryInput.i16ActionID;
        ManeuverList.current_id = juryInput.i16ManeuverEntry;
        driverStruct.i16ManeuverEntry = ManeuverList.current_id;

        //for debug
        //        LOG_INFO(cString::Format("Autonomous Driving: %d %d",ManeuverList.state, ManeuverList.current_id));

        switch (ManeuverList.state)
        {
        case action_getready:
            if(ManeuverList.number_of_id == 0)
            {
                LOG_ERROR("NO Maneuver List!!");
                driverStruct.i16StateID = statecar_error;
                ManeuverList.ready_flag = tFalse;
            }
            else if(position_initial_flag == tFalse)
            {
                LOG_ERROR("NO Position!!");
                driverStruct.i16StateID = statecar_error;
                ManeuverList.ready_flag = tFalse;
            }
            else
            {
                driverStruct.i16StateID = statecar_ready;
                ManeuverList.ready_flag = tTrue;


//                RoutePlanning();
                RoutePlanningFinal();


                for(int index = 0; index < ManeuverList.number_of_id; index++)
                {
                    switch (ManeuverList.action[index][0])
                    {
                    case CAR_STOP:
                        LOG_ERROR(adtf_util::cString::Format("Maneuver id: %d ERROR!!!!!", index));
                        break;
                    case PULL_OUT_LEFT:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d pull_out_left", index, intersection_list[index] + 1));
                        break;
                    case PULL_OUT_RIGHT:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d pull_out_right", index, intersection_list[index] + 1));
                        break;
                    case TURN_LEFT:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d left", index, intersection_list[index] + 1));
                        break;
                    case TURN_RIGHT:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d right", index, intersection_list[index] + 1));
                        break;
                    case STRAIGHT:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d straight", index, intersection_list[index] + 1));
                        break;
                    case MERGE_LEFT:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d merge_left", index, intersection_list[index] + 1));
                        break;
                    case PARKING:
                        LOG_INFO(adtf_util::cString::Format("Maneuver id: %d  Intersection id: %d cross_parking %d", index, intersection_list[index] + 1, ManeuverList.action[index][1]));
                        break;
                    }

        //            LOG_INFO(adtf_util::cString::Format("Intersection id: %d", intersection_list[index]+1));

                }

                LOG_WARNING(cString::Format("Maneuver End ID: %d action: Car Stop", ManeuverList.number_of_id));

                ResetLights();
                ResetValue();
                image_processing_function_switch = LANE_DETECTION;


                LOG_INFO(cString::Format("Autonomous Driving: Received Request Ready with maneuver ID %d", ManeuverList.current_id));
            }
            vehicle_target_speed = 0;
            last_driving_model_flag = CAR_STOP;



            for(int index = 0; index < ManeuverList.number_of_id; index++)
            {
                if(space[index][2] == ManeuverList.current_id)
                {
                    space_id_counter = index + 1;
                    LOG_INFO(adtf_util::cString::Format("space[space_id_counter][1] %d %d", space[space_id_counter][1], space_id_counter));
                    break;
                }
            }

            break;


        case action_start:
            if(ManeuverList.ready_flag == tTrue)
            {
                if(ManeuverList.action[ManeuverList.current_id][0] == PULL_OUT_LEFT || ManeuverList.action[ManeuverList.current_id][0] == PULL_OUT_RIGHT)
                {
                    pull_out_light_counter = 0;
                    last_driving_model_flag = ManeuverList.action[ManeuverList.current_id][0];
                }
                else
                {
                    image_processing_function_switch = LANE_DETECTION;
                    last_driving_model_flag = LANE_KEEPING;

//                    log_file = fopen("/home/aadc/Desktop/Test.txt","w");
                }


                LOG_INFO(adtf_util::cString::Format("space[space_id_counter][1] %d %d", space[space_id_counter][1], space_id_counter));

                driverStruct.i16StateID = statecar_running;
                LOG_INFO(cString::Format("Autonomous Driving: Received Run with maneuver ID %d  Driving Flag &g", ManeuverList.current_id, last_driving_model_flag));
            }
            else
            {
                vehicle_target_speed = 0;
                last_driving_model_flag = CAR_STOP;
                driverStruct.i16StateID = statecar_error;
                LOG_ERROR(cString::Format("Autonomous Driving: Received Run but Car not Ready maneuver ID %d  Driving Flag &g", ManeuverList.current_id, last_driving_model_flag));
            }



            break;

        case action_stop:
            ManeuverList.ready_flag = tFalse;
            vehicle_target_speed = 0;
            memset(path_reference_point.X, 0, 20*sizeof(float));
            memset(path_reference_point.Y, 0, 20*sizeof(float));
            last_driving_model_flag = CAR_STOP;
            TransmitTrajectoryPointAndSpeed(number_of_reference_point, vehicle_target_speed);

            driverStruct.i16StateID = statecar_error;
            LOG_INFO(cString::Format("Autonomous Driving: Received Stop with maneuver ID %d", ManeuverList.current_id));

//             fclose(log_file);

            break;
        }
        TransmitDriverStruct(driverStruct.i16StateID, driverStruct.i16ManeuverEntry);
    }

    RETURN_NOERROR;
}













/////////////////////////////////////////////////
/// Send Data
///
///
tResult cOptilm_AutonomousDriving::TransmitDriveModel(float value)
{
    transmitSignalValue(m_oOutput_DriveingModel, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, value);
    // for Debug
    //    LOG_INFO(cString::Format("Send Driving mode %g", value));
    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::TransmitEmergencyBreakFlag(float flag)
{
    transmitSignalValue(m_oOutput_EmergencyBreakFlag, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, flag);
    // for Debug
    //    LOG_INFO(cString::Format("Send Emergency Break Flag %g", flag));
    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::TransmitImageControl(float flag)
{
    transmitSignalValue(m_oOutput_Image_control_mode, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, 0, m_ddlSignalValueId.value, flag);
    // for Debug
    //    LOG_INFO(cString::Format("Send Image Control Flag %g", flag));
    RETURN_NOERROR;
}


tResult cOptilm_AutonomousDriving::TransmitDriverStruct(tInt16 StateID, tInt16 ManeuverEntry)
{
    tDriverStruct driverStruct;
    driverStruct.i16StateID = StateID;
    driverStruct.i16ManeuverEntry = ManeuverEntry;

    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample, m_pClock->GetStreamTime()));
    {
        auto oCodec = m_driverStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct.i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry, driverStruct.i16ManeuverEntry));
    }

    m_oOutputDriverStruct << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}


tResult cOptilm_AutonomousDriving::TransmitTrajectoryPointAndSpeed(int max_count, float speed)
{
    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {
        {
            auto oCodec = m_TrajectoryPointSampleFactory.MakeCodecFor(pWriteSample);

            tWorldCoordinate *pWoldCoordinates = reinterpret_cast<tWorldCoordinate*>(oCodec.GetElementAddress(m_ddlTrajectoryPointDataId.pointArray));


            for (int index = 0; index < 20; index++)
            {
                if(index <= max_count)
                {
                    pWoldCoordinates[index].world_X = path_reference_point.X[index];
                    pWoldCoordinates[index].world_Y = path_reference_point.Y[index];
                }
                else
                {
                    pWoldCoordinates[index].world_X = 999;
                    pWoldCoordinates[index].world_Y = 999;
                }
            }
            pWoldCoordinates[20].world_X = max_count;
            pWoldCoordinates[20].world_Y = speed;

        }
        m_oOutput_trajectory_point << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::TransmitDataFusionVideo(void)
{

#define IMAGE_HEIGHT 460
#define IMAGE_WIDTH  430
#define IMAGE_HALF_HEIGHT (IMAGE_HEIGHT/2.0)
#define IMAGE_HALF_WIDTH  (IMAGE_WIDTH/2.0)


    // new image for result
    cv::Mat outputImage;


    int row, col;
    int red = 0;
    int green = 0;
    int blue = 0;
    int index = 0;

    float half_lane_width = (lane.width * 0.5);
    float y_coordinate = 0;
    float car_y_coordinate = 0;
    float distance2 = IMAGE_HALF_HEIGHT - 8;
    float distance = IMAGE_HALF_HEIGHT - 8;


    outputImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);


    for(row = 0; row < IMAGE_HEIGHT; row++)
    {
        if(row < IMAGE_HALF_HEIGHT - 8)
        {
            if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
            {
                y_coordinate = (lane.kurve_parameter[K] *(distance * distance)) + (lane.kurve_parameter[M] * distance) + lane.kurve_parameter[B];

                distance--;
            }
            car_y_coordinate = (vehicle_path_polynomial[K] *(distance2 * distance2)) + (vehicle_path_polynomial[M] * distance2);


            distance2--;
        }

        for(col = 0; col < IMAGE_WIDTH; col++)
        {
            red = green = blue = 0;

            if(row < IMAGE_HALF_HEIGHT - 8)
            {
                if(lane.detect_mode == LTRACE)
                {
                    /*if(col == (int)(y_coordinate + (IMAGE_WIDTH/2)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                    else */if(col == (int)(y_coordinate + (half_lane_width)+ (IMAGE_HALF_WIDTH)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                    else if(col == (int)((IMAGE_HALF_WIDTH) + y_coordinate - (half_lane_width)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                }
                else if(lane.detect_mode == SL_TRACE)
                {
                    if(lane.left_or_right_line == SL_Right &&  col == (int)(y_coordinate + (half_lane_width)+ (IMAGE_HALF_WIDTH)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                    else if(lane.left_or_right_line == SL_Right &&  col == (int)((IMAGE_HALF_WIDTH) + y_coordinate - (half_lane_width)))
                    {
                        blue  = 128;  green = 128;  red   = 128;
                    }
                    else if(lane.left_or_right_line == SL_Left && col == (int)((IMAGE_HALF_WIDTH) + y_coordinate - (half_lane_width)))
                    {
                        blue  = 255;  green = 128;  red   = 0;
                    }
                    else if(lane.left_or_right_line == SL_Left && col == (int)(y_coordinate + (half_lane_width)+ (IMAGE_HALF_WIDTH)))
                    {
                        blue  = 128;  green = 128;  red   = 128;
                    }
                }

                if(col == (int)(((IMAGE_HALF_WIDTH) + car_y_coordinate) + (35/2)))
                {
                    blue  = 255;  green = 255;  red   = 255;
                }
                else if(col == (int)(((IMAGE_HALF_WIDTH) + car_y_coordinate) - (35 / 2)))
                {
                    blue  = 255;  green = 255;  red   = 255;
                }
            }


            outputImage.at<Vec3b>(row,col)[0] = red;    //R
            outputImage.at<Vec3b>(row,col)[1] = green;  //G
            outputImage.at<Vec3b>(row,col)[2] = blue;   //B
        }
    }

    //Draw Tracking Point
    if(last_driving_model_flag == LANE_KEEPING)
    {
        int image_lane_X = 0;
        int image_lane_Y = 0;
        if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
        {
            for(index = 0; index <= number_of_reference_point; index++)
            {
                image_lane_X =  (int)(path_reference_point.X[index] *  100);
                image_lane_Y =  (int)(path_reference_point.Y[index] * -100);

                image_lane_X = (IMAGE_HALF_HEIGHT - 8) - (image_lane_X);
                image_lane_Y = (image_lane_Y) + (IMAGE_HALF_WIDTH);


                if(image_lane_X > IMAGE_HEIGHT)
                    image_lane_X = IMAGE_HEIGHT - 5;
                else if(image_lane_X < 0)
                    image_lane_X = 5;

                if(image_lane_Y > IMAGE_WIDTH)
                    image_lane_Y = IMAGE_WIDTH - 5;
                else if(image_lane_Y < 0)
                    image_lane_Y = 5;

                circle(outputImage, Point(image_lane_Y,image_lane_X), 2, Scalar(255,128,0), -1);
            }
        }
    }



    //Draw MAP range
    rectangle(outputImage, Point(0,0), Point(IMAGE_WIDTH-1,IMAGE_HEIGHT-1), Scalar(0,255,0), 2);


    //Draw Car
    //    rectangle(outputImage, Point(IMAGE_WIDTH/2,0), Point(IMAGE_WIDTH/2+1,IMAGE_HEIGHT), Scalar(0,255,0), 1);      //Draw Car Center
    rectangle(outputImage, Point(200,200), Point(230,260), Scalar( 28, 17,238), 2);
    rectangle(outputImage, Point(200,212), Point(204,222), Scalar(117,120,138), 1);  //Front Wheel Left
    rectangle(outputImage, Point(226,212), Point(229,222), Scalar(117,120,138), 1);  //Front Wheel Right
    rectangle(outputImage, Point(200,238), Point(204,248), Scalar(117,120,138), 1);  //Rear Wheel Left
    rectangle(outputImage, Point(226,238), Point(229,248), Scalar(117,120,138), 1);  //Rear Wheel Rifht


    //Draw Ultrasonic
    circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.side_left.Y  , (IMAGE_HALF_HEIGHT) - US_coordinate.side_left.X), 3, Scalar(0,255,0), -1);
    circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.side_right.Y , (IMAGE_HALF_HEIGHT) - US_coordinate.side_right.X), 3, Scalar(0,255,0), -1);

    if(US_coordinate.r_l_state == EMERGENCY)
        circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.rear_left.Y  , (IMAGE_HALF_HEIGHT) - US_coordinate.rear_left.X), 5, Scalar(255,0,0), -1);
    else
        circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.rear_left.Y  , (IMAGE_HALF_HEIGHT) - US_coordinate.rear_left.X), 3, Scalar(0,255,0), -1);

    if(US_coordinate.r_c_state == EMERGENCY)
        circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.rear_center.Y, (IMAGE_HALF_HEIGHT) - US_coordinate.rear_center.X), 5, Scalar(255,0,0), -1);
    else
        circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.rear_center.Y, (IMAGE_HALF_HEIGHT) - US_coordinate.rear_center.X), 3, Scalar(0,255,0), -1);

    if(US_coordinate.r_r_state == EMERGENCY)
        circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.rear_right.Y , (IMAGE_HALF_HEIGHT) - US_coordinate.rear_right.X), 5, Scalar(255,0,0), -1);
    else
        circle(outputImage, Point((IMAGE_HALF_WIDTH) + US_coordinate.rear_right.Y , (IMAGE_HALF_HEIGHT) - US_coordinate.rear_right.X), 3, Scalar(0,255,0), -1);


    //Draw lacer scanner
    if(laser_scann.number_of_scan_point != 0)
    {
        int point[2] = {0,0};
        for(index = 0; index <laser_scann.number_of_scan_point; index++)
        {
            if(laser_scann.coordinate[index].X != 0 && laser_scann.coordinate[index].X != 0)
            {
                point[X] = (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT)) - laser_scann.coordinate[index].X ;
                point[Y] = (IMAGE_HALF_WIDTH)  + laser_scann.coordinate[index].Y ;

                if(point[X] > IMAGE_HEIGHT)
                    point[X] = IMAGE_HEIGHT - 5;
                else if(point[X] < 0)
                    point[X] = 5;

                if(point[Y] > IMAGE_WIDTH)
                    point[Y] = IMAGE_WIDTH - 5;
                else if(point[Y] < 0)
                    point[Y] = 5;

                if(laser_scann.point_state[index] == NORMAL)
                {
                    circle(outputImage, Point(point[Y], point[X]), 1, Scalar(0,255,0), -1);

                }
                else if(laser_scann.point_state[index] == WARNING)
                {
                    circle(outputImage, Point(point[Y], point[X]), 2, Scalar(255,242,0), -1);
                    //                LOG_INFO(cString::Format("Point %d, X: %g  Y: %g",index,laser_scann.coordinate[index].X, laser_scann.coordinate[index].Y));
                }
                else if(laser_scann.point_state[index] == EMERGENCY)
                {
                    circle(outputImage, Point(point[Y], point[X]), 3, Scalar(255,0,0), -1);
                    //                LOG_INFO(cString::Format("Point %d, X: %g  Y: %g",index,(laser_scann.coordinate[index].X - 5), laser_scann.coordinate[index].Y));
                }
            }

        }
    }


    //    if(obstacle_number != 0)
    //    {
    //        for(int obstacle_number_index = 0; obstacle_number_index <obstacle_number; obstacle_number_index++)
    //        {
    //            rectangle(outputImage, Point((IMAGE_HALF_WIDTH + obstacle_laserscanner[obstacle_number_index].boundary.left), (IMAGE_HALF_HEIGHT - obstacle_laserscanner[obstacle_number_index].boundary.top - (CAR_CENTER_TO_FRONT - 5))), Point((IMAGE_HALF_WIDTH + obstacle_laserscanner[obstacle_number_index].boundary.right), (IMAGE_HALF_HEIGHT - obstacle_laserscanner[obstacle_number_index].boundary.bom) - (CAR_CENTER_TO_FRONT - 5)), Scalar(255,0,0), -1);
    //        }
    //    }


    //Draw crossing detection region
    if(last_driving_model_flag == CAR_STOP && crossing_flag != CROSSING_FLAG_OFF)
    {
        for(index = 0; index < 4; index++)
        {
            if(crossing_busy_flag[index] == false)
                rectangle(outputImage, Point((IMAGE_HALF_WIDTH + crossing_Obstacle_boundary[index].left), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - crossing_Obstacle_boundary[index].top)), Point((IMAGE_HALF_WIDTH + crossing_Obstacle_boundary[index].right), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - crossing_Obstacle_boundary[index].bom)), Scalar(0,255,0), 1);
            else
                rectangle(outputImage, Point((IMAGE_HALF_WIDTH + crossing_Obstacle_boundary[index].left), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - crossing_Obstacle_boundary[index].top)), Point((IMAGE_HALF_WIDTH + crossing_Obstacle_boundary[index].right), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - crossing_Obstacle_boundary[index].bom)), Scalar(255,0,0), 3);

        }
    }

    //draw emergency vehicle detection region
    if(in_emergency_vehicle_boundary_flag == true)
    {

        if(emergency_vehicle_counter == 0)
            rectangle(outputImage, Point((IMAGE_HALF_WIDTH + emergency_car_detect_region.left), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - emergency_car_detect_region.top)), Point((IMAGE_HALF_WIDTH + emergency_car_detect_region.right), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - emergency_car_detect_region.bom)), Scalar(0,255,0), 1);
        else
            rectangle(outputImage, Point((IMAGE_HALF_WIDTH + emergency_car_detect_region.left), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - emergency_car_detect_region.top)), Point((IMAGE_HALF_WIDTH + emergency_car_detect_region.right), (IMAGE_HALF_HEIGHT - (CAR_CENTER_TO_FRONT - LASERSCANNER_CENTER_TO_CAR_FRONT) - emergency_car_detect_region.bom)), Scalar(255,0,0), 3);


    }

    //draw obstacle from laserscaner
    if(find_obstacle == true)
        rectangle(outputImage, Point((IMAGE_HALF_WIDTH + obstacle_from_laserscanner.left), (IMAGE_HALF_HEIGHT - CAR_CENTER_TO_FRONT - obstacle_from_laserscanner.top)), Point((IMAGE_HALF_WIDTH + obstacle_from_laserscanner.right), (IMAGE_HALF_HEIGHT - CAR_CENTER_TO_FRONT - obstacle_from_laserscanner.bom)), Scalar(255,0,0), -1);


    char text[100];
    if(road_sign_ID != NO_TRAFFIC_SIGN)
    {
        sprintf(text, "%d" ,road_sign_ID);
        putText(outputImage, text, Point((IMAGE_HALF_WIDTH - road_sign_lateral - 6), ((IMAGE_HALF_HEIGHT - CAR_CENTER_TO_FRONT) - road_sign_distance - 4)), 0, 0.3, Scalar(128,128,255),1);
        rectangle(outputImage, Point((IMAGE_HALF_WIDTH - road_sign_lateral - 7), ((IMAGE_HALF_HEIGHT - CAR_CENTER_TO_FRONT) - road_sign_distance - 14)), Point((IMAGE_HALF_WIDTH - road_sign_lateral + 7), ((IMAGE_HALF_HEIGHT - CAR_CENTER_TO_FRONT) - road_sign_distance)), Scalar(0,255,0), 1);
        sprintf(text, "%.1fcm" ,road_sign_distance);
        putText(outputImage, text, Point((IMAGE_HALF_WIDTH - road_sign_lateral + 10), ((IMAGE_HALF_HEIGHT - CAR_CENTER_TO_FRONT) - road_sign_distance - 4)), 0, 0.5, Scalar(255,255,255),1);
    }

    if(last_driving_model_flag == LANE_KEEPING)
    {
        float weight_fact = 0;
        double data[2] = {0, 0};
        data[0] = vehicle_speed.f32Value;
        data[1] = path_reference_point.Y[5];


        weight_fact = (float)myNeuralNetworkFunction(&data[0]);


        sprintf(text, "Lane deviation: %.1f cm  MPC Weight Fact: %.1f", lane.kurve_parameter[B], weight_fact);
        putText(outputImage, text, Point(10, IMAGE_HEIGHT - 80), 0, 0.5, Scalar(0,255,0),1);
    }
    else
    {
        sprintf(text, "Lane deviation: 0 cm  MPC Weight Fact: 0");
        putText(outputImage, text, Point(10, IMAGE_HEIGHT - 80), 0, 0.5, Scalar(255,0,0),1);
    }

    if(steering_angle.f32Value > 0)
        sprintf(text, "Steering Angle: %.1f degree", (steering_angle.f32Value / POSITIVE_STEERING_ANGLE_TO_PERCENT));
    else if(steering_angle.f32Value < 0)
        sprintf(text, "Steering Angle: %.1f degree", (steering_angle.f32Value / NEGATIVE_STEERING_ANGLE_TO_PERCENT));
    else if(steering_angle.f32Value == 0)
        sprintf(text, "Steering Angle: 0 degree");
    putText(outputImage, text, Point(10, IMAGE_HEIGHT - 60), 0, 0.5, Scalar(0,255,0),1);

    sprintf(text, "Real(Traget) Speed: %.2f (%.2f) m/s", vehicle_speed.f32Value, vehicle_target_speed);
    putText(outputImage, text, Point(10, IMAGE_HEIGHT - 40), 0, 0.5, Scalar(0,255,0),1);


    if(ManeuverList.number_of_id > 0)
    {
        switch (ManeuverList.action[ManeuverList.current_id][0])
        {
        case PULL_OUT_LEFT:
            sprintf(text, "Running: %d pull_out_left  Intersection id: %d", ManeuverList.current_id, intersection_list[ManeuverList.current_id] + 1);
            break;
        case PULL_OUT_RIGHT:
            sprintf(text, "Running: %d pull_out_right  Intersection id: %d", ManeuverList.current_id, intersection_list[ManeuverList.current_id] + 1);
            break;
        case TURN_LEFT:
            sprintf(text, "Running: %d left  Intersection id: %d", ManeuverList.current_id, intersection_list[ManeuverList.current_id] + 1);
            break;
        case TURN_RIGHT:
            sprintf(text, "Running: %d right  Intersection id: %d", ManeuverList.current_id, intersection_list[ManeuverList.current_id] + 1);
            break;
        case STRAIGHT:
            sprintf(text, "Running: %d straight  Intersection id: %d", ManeuverList.current_id, intersection_list[ManeuverList.current_id] + 1);
            break;
        case MERGE_LEFT:
            sprintf(text, "Running: %d merge_left  Intersection id: %d", ManeuverList.current_id, intersection_list[ManeuverList.current_id] + 1);
            break;
        case PARKING:
            sprintf(text, "Running: %d cross_parking %d", ManeuverList.current_id, ManeuverList.action[ManeuverList.current_id][1]);
            break;
        case CAR_STOP:
            sprintf(text, "Running: %d STOP!!!", ManeuverList.current_id);
            break;
        }


        putText(outputImage, text, Point(10, IMAGE_HEIGHT - 20), 0, 0.5, Scalar(0,255,0),1);

    }
    else
        putText(outputImage, "NO Maneuver List!!", Point(10, IMAGE_HEIGHT - 20), 0, 0.5, Scalar(255,0,0),1);


    //July Model oder Tester
    if(m_Driving_Model_Test)
        rectangle(outputImage, Point(IMAGE_WIDTH - 30,IMAGE_HEIGHT -30), Point(IMAGE_WIDTH - 10,IMAGE_HEIGHT -10), Scalar(255,0,0), -1);
    else
        rectangle(outputImage, Point(IMAGE_WIDTH - 30,IMAGE_HEIGHT -30), Point(IMAGE_WIDTH - 10,IMAGE_HEIGHT -10), Scalar(255,242,0), -1);



    //Write processed Image to Output Pin
    if (!outputImage.empty())
    {
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oVideo_Output, outputImage);
        }
        // write to pin
        writeMatToPin(m_oVideo_Output, outputImage, m_pClock->GetStreamTime());
    }



    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::ToggleLights(int buttonId, bool toggle)
{
    if(light_flag[buttonId] != toggle)
    {
        light_flag[buttonId] = toggle;

        switch (buttonId)
        {
        case 0: // Head
            transmitBoolSignalValue(m_oOutputHeadLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, light_flag[buttonId]);
            //                LOG_INFO(cString::Format("Heads toggled: %d", light_flag[buttonId]));
            break;
        case 1: // Brake
            transmitBoolSignalValue(m_oOutputBrakeLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, light_flag[buttonId]);
            //                LOG_INFO(cString::Format("Brake toggled: %d", light_flag[buttonId]));
            break;
        case 2: // Reverse
            transmitBoolSignalValue(m_oOutputReverseLight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, light_flag[buttonId]);
            //                LOG_INFO(cString::Format("Reverse toggled: %d", light_flag[buttonId]));
            break;
        case 3: // Hazard
            transmitBoolSignalValue(m_oOutputHazard, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, light_flag[buttonId]);
            //                LOG_INFO(cString::Format("Hazard toggled: %d", light_flag[buttonId]));
            break;
        case 4: // Left
            transmitBoolSignalValue(m_oOutputTurnLeft, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, light_flag[buttonId]);
            //                LOG_INFO(cString::Format("Turn Left toggled: %d", light_flag[buttonId]));
            break;
        case 5: // Right
            transmitBoolSignalValue(m_oOutputTurnRight, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0, m_ddlBoolSignalValueId.bValue, light_flag[buttonId]);
            //                LOG_INFO(cString::Format("Turn right toggled: %d", light_flag[buttonId]));
            break;


        default:
            break;
        }
    }

    RETURN_NOERROR;
}


tResult cOptilm_AutonomousDriving::RoutePlanning()
{


            int start_index = 0;

            for(int index = 0; index < ManeuverList.number_of_id; index++)
            {
                if(ManeuverList.action[index][0]==PULL_OUT_LEFT || ManeuverList.action[index][0]==PULL_OUT_RIGHT )
                {
                    start_index = index;
                    break;
                }
                else if (ManeuverList.action[index][0]==PARKING )
                {
                    start_index = index;
                    break;
                }
            }
            if (start_index == 0)
            {
                int current_angle = 90;
                int current_index = 0;
                intersection_list[0] = 0;

                for(int index = 0; index < ManeuverList.number_of_id; index++)
                {
                    int out_angle;

                    switch (ManeuverList.action[index][0])
                    {
                    case CAR_STOP:

                        break;
                    case PULL_OUT_LEFT:
                        out_angle = current_angle + 90;
                        break;
                    case PULL_OUT_RIGHT:
                        out_angle = current_angle - 90;
                        break;
                    case TURN_LEFT:
                        out_angle = current_angle + 90;
                        break;
                    case TURN_RIGHT:
                        out_angle = current_angle - 90;
                        break;
                    case STRAIGHT:
                        out_angle = current_angle;
                        break;
                    case MERGE_LEFT:
                        out_angle = current_angle;
                        break;
                    case PARKING:
                        out_angle = current_angle + 90;
                        break;
                    }


                    if (out_angle == 270)
                        out_angle = -90;
                    else if (out_angle == -180)
                        out_angle = 180;

                    if(current_index == 5 && ManeuverList.action[index][0] != STRAIGHT)
                        current_index = 4;
        //            else if (current_index == 2 && ManeuverList.action[index][0] != STRAIGHT)
        //                current_index = 3;
                    else if (current_index == 7 && ManeuverList.action[index][0] != STRAIGHT)
                        current_index = 6;
                    else if (current_index == 4)
                        current_index = 5;
                    else if (current_index == 6)
                        current_index = 7;

                    if( ManeuverList.action[index][0] == PARKING)
                    {

                        current_angle = 90;
                        intersection_list[index+1] = 0;
                        current_index = 0;
                    }
                    else
                    {
                        for(int i = 1; i < 8; i++)
                        {
                            int temp_direction;
                            if (intersection_adjacency_matrix[i][current_index]!=0)
                            {
                                if (intersection_direction[i][current_index] == 0)
                                    temp_direction = 180;
                                else if (intersection_direction[i][current_index] == 180)
                                    temp_direction = 0;
                                else if (intersection_direction[i][current_index] == -45)
                                    temp_direction = 135;
                                else
                                    temp_direction = -intersection_direction[i][current_index];

                                if (out_angle == temp_direction)
                                {
                                    current_angle = intersection_direction[current_index][i];
                                    intersection_list[index+1] = i;
                                    current_index = i;

    //                                LOG_INFO(adtf_util::cString::Format("Intersection id: %d out_angle %d current angle %d", current_index+1, out_angle, current_angle));
                                    break;
                                }
                            }

                        }
                    }

                }
            }
            else
            {
                int current_angle = 90;
                int current_index = 0;
                intersection_list[start_index] = 0;

                for(int index = start_index; index > 0; index--)
                {
                    int out_angle;

                    switch (ManeuverList.action[index][0])
                    {
                    case CAR_STOP:

                        break;
                    case PULL_OUT_LEFT:
                        out_angle = current_angle - 90;
                        break;
                    case PULL_OUT_RIGHT:
                        out_angle = current_angle + 90;
                        break;
                    case TURN_LEFT:
                        out_angle = current_angle - 90;
                        break;
                    case TURN_RIGHT:
                        out_angle = current_angle + 90;
                        break;
                    case STRAIGHT:
                        out_angle = current_angle;
                        break;
                    case MERGE_LEFT:
                        out_angle = current_angle;
                        break;
                    case PARKING:
                        out_angle = current_angle + 90;
                        break;
                    }


                    if (out_angle == 270)
                        out_angle = -90;
                    else if (out_angle == -180)
                        out_angle = 180;

                    if(current_index == 5 && ManeuverList.action[index][0] != STRAIGHT)
                        current_index = 4;
        //            else if (current_index == 2 && ManeuverList.action[index][0] != STRAIGHT)
        //                current_index = 3;
                    else if (current_index == 7 && ManeuverList.action[index][0] != STRAIGHT)
                        current_index = 6;
                    else if (current_index == 4)
                        current_index = 5;
                    else if (current_index == 6)
                        current_index = 7;

//                    if( ManeuverList.action[index][0] == PARKING)
//                    {

//                        current_angle = 90;
//                        intersection_list[index+1] = 0;
//                        current_index = 0;
//                    }
//                    else
                    {
                        for(int i = 1; i < 8; i++)
                        {
                            int temp_direction;
                            if (intersection_adjacency_matrix[i][current_index]!=0)
                            {
                                if (intersection_direction[i][current_index] == 0)
                                    temp_direction = 180;
                                else if (intersection_direction[i][current_index] == 180)
                                    temp_direction = 0;
                                else if (intersection_direction[i][current_index] == -45)
                                    temp_direction = 135;
                                else
                                    temp_direction = -intersection_direction[i][current_index];

                                if (out_angle == temp_direction)
                                {
                                    current_angle = intersection_direction[current_index][i];
                                    intersection_list[index-1] = i;
                                    current_index = i;

//                                    LOG_INFO(adtf_util::cString::Format("Intersection id: %d out_angle %d current angle %d", current_index+1, out_angle, current_angle));
                                    break;
                                }
                            }

                        }
                    }

                }

                current_angle = 90;
                current_index = 0;

                for(int index = start_index+1; index < ManeuverList.number_of_id; index++)
                {
                    int out_angle;

                    switch (ManeuverList.action[index][0])
                    {
                    case CAR_STOP:

                        break;
                    case PULL_OUT_LEFT:
                        out_angle = current_angle + 90;
                        break;
                    case PULL_OUT_RIGHT:
                        out_angle = current_angle - 90;
                        break;
                    case TURN_LEFT:
                        out_angle = current_angle + 90;
                        break;
                    case TURN_RIGHT:
                        out_angle = current_angle - 90;
                        break;
                    case STRAIGHT:
                        out_angle = current_angle;
                        break;
                    case MERGE_LEFT:
                        out_angle = current_angle;
                        break;
                    case PARKING:
                        out_angle = current_angle + 90;
                        break;
                    }


                    if (out_angle == 270)
                        out_angle = -90;
                    else if (out_angle == -180)
                        out_angle = 180;

                    if(current_index == 5 && ManeuverList.action[index][0] != STRAIGHT)
                        current_index = 4;
        //            else if (current_index == 2 && ManeuverList.action[index][0] != STRAIGHT)
        //                current_index = 3;
                    else if (current_index == 7 && ManeuverList.action[index][0] != STRAIGHT)
                        current_index = 6;
                    else if (current_index == 4)
                        current_index = 5;
                    else if (current_index == 6)
                        current_index = 7;

//                    if( ManeuverList.action[index][0] == PARKING)
//                    {

//                        current_angle = 90;
//                        intersection_list[index+1] = 0;
//                        current_index = 0;
//                    }
//                    else
                    {
                        for(int i = 1; i < 8; i++)
                        {
                            int temp_direction;
                            if (intersection_adjacency_matrix[i][current_index]!=0)
                            {
                                if (intersection_direction[i][current_index] == 0)
                                    temp_direction = 180;
                                else if (intersection_direction[i][current_index] == 180)
                                    temp_direction = 0;
                                else if (intersection_direction[i][current_index] == -45)
                                    temp_direction = 135;
                                else
                                    temp_direction = -intersection_direction[i][current_index];

                                if (out_angle == temp_direction)
                                {
                                    current_angle = intersection_direction[current_index][i];
                                    intersection_list[index+1] = i;
                                    current_index = i;

//                                    LOG_INFO(adtf_util::cString::Format("Intersection id: %d out_angle %d current angle %d", current_index+1, out_angle, current_angle));
                                    break;
                                }
                            }

                        }
                    }

                }
            }

}

tResult cOptilm_AutonomousDriving::RoutePlanningFinal()
{
    //Final
    int start_index = 0;

    for(int index = 0; index < ManeuverList.number_of_id; index++)
    {
        if(ManeuverList.action[index][0]==PULL_OUT_LEFT || ManeuverList.action[index][0]==PULL_OUT_RIGHT )
        {
            start_index = index;
            break;
        }
        else if (ManeuverList.action[index][0]==PARKING )
        {
            start_index = index;
            break;
        }
    }

    if (start_index == 0)
    {

            int current_angle = 90;
            int current_index = 0;
            intersection_list[0] = 0;

//            if (position_data[X]>-1 && position_data[X]<3 && position_data[Y] < 8 && position_data[Y] > 5)
//            {
////                current_angle = 90;
//                current_index = 0;
//                intersection_list[0] = 0;
//            }
//            else
//            {
////                current_angle = 90;
//                current_index = 1;
//                intersection_list[0] = 1;
//            }

            for(int index = 0; index < ManeuverList.number_of_id; index++)
            {
                int out_angle;

                switch (ManeuverList.action[index][0])
                {
                case CAR_STOP:

                    break;
                case PULL_OUT_LEFT:
                    out_angle = current_angle + 90;
                    break;
                case PULL_OUT_RIGHT:
                    out_angle = current_angle - 90;
                    break;
                case TURN_LEFT:
                    out_angle = current_angle + 90;
                    break;
                case TURN_RIGHT:
                    out_angle = current_angle - 90;
                    break;
                case STRAIGHT:
                    out_angle = current_angle;
                    break;
                case MERGE_LEFT:
                    out_angle = current_angle;
                    break;
                case PARKING:
                    out_angle = current_angle + 90;
                    break;
                }


                if (out_angle == 270)
                    out_angle = -90;
                else if (out_angle == -180)
                    out_angle = 180;



                if( ManeuverList.action[index][0] == PARKING)
                {
                    //Final
                    current_angle = 90;
                    intersection_list[index+1] = 1;
                    current_index = 1;

                }
                else
                {
                    for(int i = 2; i < 13; i++)
                    {
                        int temp_direction;
                        if (intersection_adjacency_matrix[i][current_index]!=0)
                        {
                            if (intersection_direction[i][current_index] == 0)
                                temp_direction = 180;
                            else if (intersection_direction[i][current_index] == 180)
                                temp_direction = 0;
                            else if (intersection_direction[i][current_index] == 45)
                                temp_direction = -135;
                            else
                                temp_direction = -intersection_direction[i][current_index];

                            if (out_angle == temp_direction)
                            {
                                current_angle = intersection_direction[current_index][i];
                                intersection_list[index+1] = i;
                                current_index = i;

//                                LOG_INFO(adtf_util::cString::Format("Intersection id: %d out_angle %d current angle %d", current_index+1, out_angle, current_angle));
                                break;
                            }
                        }

                    }
                }

            }
    }
    else
    {
        int current_angle = 90;
        int current_index = 1;
        intersection_list[start_index] = 1;

        for(int index = start_index; index > 0; index--)
        {
            int out_angle;

            switch (ManeuverList.action[index][0])
            {
            case CAR_STOP:

                break;
            case PULL_OUT_LEFT:
                out_angle = current_angle - 90;
                break;
            case PULL_OUT_RIGHT:
                out_angle = current_angle + 90;
                break;
            case TURN_LEFT:
                out_angle = current_angle - 90;
                break;
            case TURN_RIGHT:
                out_angle = current_angle + 90;
                break;
            case STRAIGHT:
                out_angle = current_angle;
                break;
            case MERGE_LEFT:
                out_angle = current_angle + 45;
                break;
            case PARKING:
                out_angle = current_angle + 90;
                break;
            }


            if (out_angle == 270)
                out_angle = -90;
            else if (out_angle == -180)
                out_angle = 180;

//            if( ManeuverList.action[index-1][0] == MERGE_LEFT)
//            {

//                current_angle = 180;
//                intersection_list[index-1] = 12;
//                current_index = 12;
//            }
//            else

            {
                for(int i = 2; i < 13; i++)
                {
                    int temp_direction;
                    if (intersection_adjacency_matrix[i][current_index]!=0)
                    {
                        if (intersection_direction[i][current_index] == 0)
                            temp_direction = 180;
                        else if (intersection_direction[i][current_index] == 180)
                            temp_direction = 0;
                        else if (intersection_direction[i][current_index] == -135)
                            temp_direction = 45;
                        else
                            temp_direction = -intersection_direction[i][current_index];

                        if (out_angle == temp_direction)
                        {
                                        if( ManeuverList.action[index-1][0] == MERGE_LEFT)
                                        {

                                            current_angle = 0;
                                            intersection_list[index-1] = 12;
                                            current_index = 12;
                                        }
                            else
                                        {
                                current_angle = intersection_direction[current_index][i];
                                intersection_list[index-1] = i;
                                current_index = i;
                            }


//                                    LOG_INFO(adtf_util::cString::Format("Intersection id: %d out_angle %d current angle %d", current_index+1, out_angle, current_angle));
                            break;
                        }
                    }

                }
            }

        }

        current_angle = 90;
        current_index = 1;

        for(int index = start_index+1; index < ManeuverList.number_of_id; index++)
        {
            int out_angle;

            switch (ManeuverList.action[index][0])
            {
            case CAR_STOP:

                break;
            case PULL_OUT_LEFT:
                out_angle = current_angle + 90;
                break;
            case PULL_OUT_RIGHT:
                out_angle = current_angle - 90;
                break;
            case TURN_LEFT:
                out_angle = current_angle + 90;
                break;
            case TURN_RIGHT:
                out_angle = current_angle - 90;
                break;
            case STRAIGHT:
                out_angle = current_angle;
                break;
            case MERGE_LEFT:
                out_angle = current_angle;
                break;
            case PARKING:
                out_angle = current_angle + 90;
                break;
            }


            if (out_angle == 270)
                out_angle = -90;
            else if (out_angle == -180)
                out_angle = 180;


                    if( ManeuverList.action[index][0] == PARKING)
                    {

                        current_angle = 90;
                        intersection_list[index+1] = 0;
                        current_index = 0;
                    }
                    else
            {
                for(int i = 2; i < 13; i++)
                {
                    int temp_direction;
                    if (intersection_adjacency_matrix[i][current_index]!=0)
                    {
                        if (intersection_direction[i][current_index] == 0)
                            temp_direction = 180;
                        else if (intersection_direction[i][current_index] == 180)
                            temp_direction = 0;
                        else if (intersection_direction[i][current_index] == 45)
                            temp_direction = -135;
                        else
                            temp_direction = -intersection_direction[i][current_index];

                        if (out_angle == temp_direction)
                        {
                            current_angle = intersection_direction[current_index][i];
                            intersection_list[index+1] = i;
                            current_index = i;

//                                    LOG_INFO(adtf_util::cString::Format("Intersection id: %d out_angle %d current angle %d", current_index+1, out_angle, current_angle));
                            break;
                        }
                    }

                }
            }

        }
    }
}

