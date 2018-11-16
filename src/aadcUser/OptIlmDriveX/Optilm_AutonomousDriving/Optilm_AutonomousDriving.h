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
#include "aadc_jury.h"
#include <a_utils/core/a_utils_core.h>
#include "TU_Ilmenau_SOP.h"

//*************************************************************************************************
#define CID_OPTILM_AUTONOMOUS_DRIVING_DATA_TRIGGERED_FILTER "Optilm_AutonomousDriving.filter.user.aadc.cid"


using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;
using namespace aadc::jury;

#define HOME_ID         26
#define OFFICE_ID       25
#define RESTAURANT_ID   28
#define POST_ID         27
#define SUPERMARKET_ID  29




typedef struct _TURN_AROUND_REFERENCE_COORDINATE
{
    float X[100];
    float Y[100];

}TURN_AROUND_REFERENCE_COORDINATE;

typedef struct _AVOIDANCE
{
    int step_counter;
    int dodge_distance;
    int dodge_lateral;
    int max_dodge_lateral;
    float last_overall_distance;
    bool flag;
    float last_lane_state;
    float last_SL_stae;
    short comeback_flag;
    int comeback_wait_counter;
}AVOIDANCE_STRUCT;


typedef struct _OBSTACLE_STRUCT
{
    SECTION_BOUNDARY boundary;
    X_Y_POINT point[200];
    int point_counter;
    float distance;
}OBSTACLE_STRUCT;



/*! the main class of the open cv template. */
class cOptilm_AutonomousDriving : public cTriggerFunction
{
private:

    //properties
    adtf::base::property_variable<tBool>    m_Driving_Model_Test = tFalse;
    adtf::base::property_variable<tBool>    m_Demo_Model = tFalse;
    adtf::base::property_variable<tFloat32> m_front_min_break_time = 0.7f;
    adtf::base::property_variable<tFloat32> m_rear_min_break_time = 0.45f;
    adtf::base::property_variable<tFloat32> m_min_break_distance = 15.0f;
    adtf::base::property_variable<tFloat32> m_vehicle_min_speed = 0.5f;
    adtf::base::property_variable<tFloat32> m_vehicle_max_speed = 1.0f;
    adtf::base::property_variable<tInt> parking_road_sign_distance = 120;
    adtf::base::property_variable<tFloat32> slot_distance[4] = {47, 0, -50, -99};
    adtf::base::property_variable<tFloat32> crossing_road_sign_distance = 120;
    adtf::base::property_variable<tFloat32> crossing_stop_line_distance = 5;
    adtf::base::property_variable<tFloat32> nostop_crossing_left = -20;
    adtf::base::property_variable<tFloat32> stop_crossing_left = -20;
    adtf::base::property_variable<tFloat32> nostop_crossing_right = -32;
    adtf::base::property_variable<tFloat32> stop_crossing_right = -20;
    adtf::base::property_variable<tFloat32> nostop_crossing_strgiht = -20;
    adtf::base::property_variable<tFloat32> stop_crossing_strgiht = -20;
    adtf::base::property_variable<tFloat32> obstacle_detection_distance = 200;
    adtf::base::property_variable<tFloat32> avoidance_start_distance = 150;
    adtf::base::property_variable<tFloat32> avoidance_side_distance = 50;
    adtf::base::property_variable<tFloat32> avoidance_comeBack_counter = 20;
    adtf::base::property_variable<tFloat32> pedestrian_road_sign_distance = 100;
    adtf::base::property_variable<tFloat32> pedestrian_low_speed_distance = 150;


    //Pins
    /*! Writer to an OutPin. Video*/
    cPinWriter m_oVideo_Output;

    //Stream Formats
        /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;


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

    /*! Media Descriptions. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! A bool signal value identifier. */
    struct tBoolSignalValueId
    {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;

    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;


    /*! A ddl laser scanner data identifier. */
    struct ddlLaserScannerDataId
    {
        tSize size;
        tSize scanArray;
    } m_ddlLSDataId;
    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;


    /*! A ddl ultrasonic structure index. */
    struct
    {
        tSignalValueId SideLeft;
        tSignalValueId SideRight;
        tSignalValueId RearLeft;
        tSignalValueId RearCenter;
        tSignalValueId RearRight;

    } m_ddlUltrasonicStructIndex;
    adtf::mediadescription::cSampleCodecFactory m_USDataSampleFactory;



    /*! A ddl laser scanner data identifier. */
    struct
    {
        tSize laneDetectMode;
        tSize parameter_k;
        tSize parameter_m;
        tSize parameter_b;
        tSize laneWidth;
        tSize leftOrRightLine;
        tSize adult_flag;
        tSize child_flag;
    } m_ddlLaneDataId;
    adtf::mediadescription::cSampleCodecFactory m_LaneDataStructSampleFactory;


    /*! The ddl indices for a tPosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    }  m_ddlPositionIndex;
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;


    /*! The ddl indices for a tRoadSignData */
    struct
    {
        tSize id;
        tSize distance;
        tSize Lateral;
        tSize direction;
        tSize RoadSign_X;
        tSize RoadSign_Y;
    } m_ddlRoadSignDataIndex;

    /*! The road sign sample factory */
    adtf::mediadescription::cSampleCodecFactory m_RoadSignDataSampleFactory;


    /*! A ddl polar coordinate identifier. */
    struct ddlPolarCoordinateId
    {
        tSize world_X;
        tSize world_Y;
    } m_ddlWorldCoordinateId;

    /*! The polar coordinate sample factory */
    adtf::mediadescription::cSampleCodecFactory m_WorldCoordinateSampleFactory;

    /*! A ddl laser scanner data identifier. */
    struct
    {
        tSize pointArray;
    } m_ddlTrajectoryPointDataId;
    adtf::mediadescription::cSampleCodecFactory m_TrajectoryPointSampleFactory;


    /*! A ddl Maneuver Listr data identifier. */
    struct
    {
        tSize maneuver;
    } m_ddlManeuverListDataId;
    adtf::mediadescription::cSampleCodecFactory m_ManeuverListSampleFactory;


    /*! The maneuver file string */
    cString     m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    aadc::jury::maneuverList m_sectorList;



    /*! Reader of an InPin. Info */
    cPinReader m_oInput_TestDriveingModel;
    cPinReader m_oInput_VehicleSpeed;
    cPinReader m_oInput_SteeringAngle;
    cPinReader m_oInput_DistanceOverall;
    cPinReader m_oInput_RoadSignData;
    cPinReader m_oInput_Position;
    cPinReader m_oInput_LaserScanner;
    cPinReader m_oInput_UltrasonicUnit;
    cPinReader m_oInput_LaneData;
    cPinReader m_oInputJuryStruct;
    cPinReader m_oInputManeuverList;
    cPinReader m_oInput_Demo_maneuver_list;


    /*! Writer to an OutPin. Info*/
    cPinWriter m_oOutputTurnRight;
    cPinWriter m_oOutputTurnLeft;
    cPinWriter m_oOutputHazard;
    cPinWriter m_oOutputHeadLight;
    cPinWriter m_oOutputReverseLight;
    cPinWriter m_oOutputBrakeLight;
    cPinWriter m_oOutput_DriveingModel;
    cPinWriter m_oOutput_EmergencyBreakFlag;
    cPinWriter m_oOutput_Position;
    cPinWriter m_oOutput_trajectory_point;
    cPinWriter m_oOutput_Image_control_mode;
    cPinWriter m_oOutputDriverStruct;



    tTimeStamp m_lastSampleTime;
    tFloat64 f64SampleTime;


public:

    /*! Default constructor. */
    cOptilm_AutonomousDriving();


    /*! Destructor. */
    virtual ~cOptilm_AutonomousDriving() = default;

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


    //Get pin Data
    tResult GetDrivingModelFromTester(void);
    tResult GetLaneData();
    tResult GetPosition();
    tResult GetCarSpeed();
    tResult GetRoadSignData();
    tResult GetDistanceOverall();
    tResult GetUltrasonicUnit();
    tResult GetLaserScanner();
    tResult GetSteeringAngle();
    tResult GetManeuverList(void);
    tResult LoadManeuverList();
    tResult GetJulyState();
    tResult GetDemoManeuverList(void);

    //Send pin data
    tResult TransmitDriverStruct(tInt16 StateID, tInt16 ManeuverEntry);
    tResult TransmitDriveModel(float value);
    tResult TransmitEmergencyBreakFlag(float flag);
    tResult TransmitTrajectoryPointAndSpeed(int max_count, float speed);
    tResult TransmitDataFusionVideo(void);
    tResult TransmitImageControl(float flag);
    tResult ToggleLights(int buttonId, bool toggle);


    tResult SystemInitial();
    tResult ResetLights();
    tResult ResetValue();
    tResult RoutePlanning();
    tResult RoutePlanningFinal();



public:
    MANEUVER_LIST ManeuverList;
    bool maneuverid_flag;
    bool light_flag[6];
    bool system_initial;
    int pull_out_light_counter;
    float last_driving_model_flag;
    LANE_DETECTION_STRUCT lane;
    REFERENCE_POINT path_reference_point;
    int path_reference_counter;
    float lane_data[5];
    float position_data[5];
    bool position_initial_flag;
    char image_processing_function_switch;

    tSignalValue vehicle_speed;
    tSignalValue steering_angle;
    tSignalValue distance_overall;
    tFloat32 last_distance_overall;
    float vehicle_target_speed;

    float current_car_HeadingAngle;

    int parking_Ready_flag;
    float parking_temp_road_sign_X;
    float parking_temp_road_sign_Y;
    bool parking_sign_found;

    float first_parking_direction_heading;

    int road_sign_ID;
    float road_sign_distance;
    float road_sign_lateral;
    float road_sign_direction;
    float road_sign_X;
    float road_sign_Y;
    int temp_road_sign_ID;
    float temp_road_sign_X;
    float temp_road_sign_Y;
    float temp_car_direction_heading;


    int crossing_flag;
    int stop_decision_flag;
    int crossing_stop_wait_counter;
    float nostop_crossing;
    float stop_crossing;
    int T_crossing_current_index;
    int T_crossing_direction;
    SECTION_BOUNDARY crossing_Obstacle_boundary[4];
    bool crossing_busy_flag[4];
    bool crossing_sign_found;

    bool pedestrian_flag;
    int pedestrian_stop_counter;
    float last_distance_overall_for_pedestrian;

    int find_side_direction;

    bool roundabout_flag;
    bool roundabout_in;

    bool S_CurveDecisionflag;

    int ramp_state_flag;

    int avoidance_permit_flag;
    int avoidance_region_nummer;
    int avoidance_car_HeadingAngle;
    AVOIDANCE_STRUCT avoidance;


    int emergency_vehicle_counter;
    float emergency_vehicle_shift;
    bool emergency_vehicle_flag;
    bool in_emergency_vehicle_boundary_flag;
    SECTION_BOUNDARY emergency_car_detect_region;



    float temp_heading;
//    int stopLine_check_flag;


    SECTION_BOUNDARY obstacle_from_laserscanner;
    bool find_obstacle;
//    int obstacle_number;
//    OBSTACLE_STRUCT obstacle_laserscanner[10];
    LASER_SCANNER_DATA laser_scann;
    tUltrasonicStruct US_data;
    ULTRASONIC_STR    US_coordinate;


    float vehicle_path_polynomial[3];
    int number_of_reference_point;
    TURN_AROUND_REFERENCE_COORDINATE turn_left_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE turn_left_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE turn_right_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE turn_right_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE straight_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE straight_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE pullout_left_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE pullout_left_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE pullout_right_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE pullout_right_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE avoidance_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE avoidance_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE merg_left_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE merg_left_ref_global_coord;
    TURN_AROUND_REFERENCE_COORDINATE parking_ref_coord;
    TURN_AROUND_REFERENCE_COORDINATE parking_ref_gloabal_coord;



    int T_crossing_nummer;
    SECTION_BOUNDARY T_section_boundary[30];
    int avoidance_nummer;
    SECTION_BOUNDARY avoidance_section_boundary[30];
    int pedestrian_nummer;
    SECTION_BOUNDARY pedestrian_section_boundary[30];
    int child_nummer;
    SECTION_BOUNDARY child_section_boundary[30];
    int low_speed_region_nummer;
    SECTION_BOUNDARY low_speed_region[5];
    int ramper_nummer;
    SECTION_BOUNDARY ramper_region[5];
    int S_curve_nummer;
    SECTION_BOUNDARY S_curve_region[5];
    int emergency_vehicle_nummer;
    SECTION_BOUNDARY emergency_vehicle_boundary[5];

    int Intersection_nummer;
    SECTION_BOUNDARY Intersection_boundary[30];

    int intersection_direction[20][20];
    int intersection_adjacency_matrix[20][20];
    int intersection_list[150];



    //Demo
    int space [10][3];
    int space_id;
    int space_id_counter;






    //Data_Processing.cpp
    tResult CalculateUltrasonicWorldCoordinate(void);
    void CalculateVehiclePath(float steering);
    tResult CalculateTrackingPoint(float number_of_interval);
    tResult CalculateTurnAroundReferencePoint(char status_flag, int N, int roundIdx);
    float GetDistanceBetweenCoordinates(float x2, float y2, float x1, float y1);
    float VehicleDirectionDecision(float car_heading);
    float vs(float x, float a, float b);
    tResult SetDigitialMapRegion();
    tResult SetDigitialMapRegionTestEvent();
    tResult SetDigitialMapRegionFinal();

    tResult SetIntersectionGraph();
    tResult SetIntersectionGraphAugumented();

    tResult SetIntersectionGraphFinal();
    tResult SetIntersectionGraphDemo();

    double myNeuralNetworkFunction(const double x1[2]);
    double SpeedNetworkFunction(double x1);



    //State_Control.cpp
    int StateMachine(int driving_mode);
    int StateMachineFromTester(int driving_mode);
    int StateControl(int input_car_state_flag);
    int EmergencyBreak(int driving_mode);
    tResult SpeedDecision(int driving_mode);
    int ParkingProcess(int driving_mode);
    tResult ObstacleDetection(void);
    int CrossingDecision(int driving_mode);
    int CrossingWaitTimeDecision(int marker_ID);
    tResult CrossingObstacleDetection();
    int AvoidanceProcess(int driving_mode);
    int RoundaboutCrossingDecision(int driving_mode_flag);
    int RampStateDecision();
    int StartMergeLeft(int driving_mode);
    int S_CurveIntervalsChangeDecision(int current_intervals);
    bool LowSpeedRegionDecision();
    tResult VehicleCurrentHeadingDecision();
    tResult CrossingDetectionDecision();
    int EmergencyVehicleProcess(int driving_mode);

    int CrossingDecision_Car2x(int driving_mode_flag);
    int PedestrianDecision(int driving_mode_flag);
    tResult CarFollowing(void);

    tResult AvoidanceWithImage(void);
    tResult ChildDetection();

    /*! The mutex */
    std::mutex m_oMutex;



    FILE *log_file; // debug file


};


//*************************************************************************************************
