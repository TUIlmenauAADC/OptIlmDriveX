#include "Optilm_AutonomousDriving.h"


float temp_crossing_stop_distance = 0;
float temp_crossing_nosign_stop_distance = 0;


//**XH
float temp_road_sign_X = 0;
float temp_road_sign_Y = 0;
float temp_car_direction_heading = 0;
//**




float temp_parking_distance = 0;



//Emergency Break
#define NO_BREAK_INDEX_FLAG 55
float emergency_break_last_driving_model_flag = NO_BREAK_INDEX_FLAG;
int emergency_break_counter;
float emergency_break_flag;


int left_section_counter = 0;
int right_section_counter = 0;
int front_section_counter = 0;
int in_section_counter = 0;
int out_section_counter = 0;


float distance_of_first_to_second_point = 0;




/* Function for decision making on vehicle maneuver mode:
 *  {CAR_STOP , LANE_FOLLOW, TURN_LEFT, TURN_RIGHT, STRAIGHT, PARKING, PULL_OUT_LEFT, PULL_OUT_RIGHT}
*/
int cOptilm_AutonomousDriving::StateMachine(int driving_mode)
{

    VehicleCurrentHeadingDecision();


    if(ManeuverList.action[ManeuverList.current_id][0] == PARKING)
    {
        driving_mode = ParkingProcess(driving_mode);
    }

    if(ManeuverList.action[ManeuverList.current_id][0] != PARKING &&
            ManeuverList.action[ManeuverList.current_id][0] != PULL_OUT_LEFT &&
            ManeuverList.action[ManeuverList.current_id][0] != PULL_OUT_RIGHT &&
            maneuverid_flag == false)
    {
        driving_mode = CrossingDecision(driving_mode);
        driving_mode = PedestrianDecision(driving_mode);
        ChildDetection();


    }


    //Emergency Vehicle Process
    if(ManeuverList.action[ManeuverList.current_id][0] != PARKING &&
            ManeuverList.action[ManeuverList.current_id][0] != PULL_OUT_LEFT &&
            ManeuverList.action[ManeuverList.current_id][0] != PULL_OUT_RIGHT &&
            ManeuverList.action[ManeuverList.current_id][0] != MERGE_LEFT)
    {
        driving_mode = EmergencyVehicleProcess(driving_mode);
    }



    if (ManeuverList.action[ManeuverList.current_id][0] == MERGE_LEFT)
    {
        //NO_RAMP = 0,  RAMP_UP = 1,  RAMP_IN = 2,  RAMP_DOWN = 3
        ramp_state_flag = RampStateDecision();
        driving_mode = StartMergeLeft(driving_mode);
    }






    //    LOG_INFO(adtf_util::cString::Format("AVOIDANCE comeback_flag %d wait flag %d driving_mode_flag %d", avoidance.comeback_flag , avoidance.comeback_wait_counter,  driving_mode_flag));
    if((driving_mode == LANE_KEEPING || driving_mode == AVOIDANCE) && driving_mode != PARKING)
        driving_mode = AvoidanceProcess(driving_mode);



    //only for free demo
    if(m_Demo_Model == tTrue && ManeuverList.ready_flag == tTrue)
    {
        if(ManeuverList.action[ManeuverList.current_id][0] == CAR_STOP)
            driving_mode = LANE_KEEPING;

//        LOG_INFO(adtf_util::cString::Format("space[space_id_counter][1] %d %d", space[space_id_counter][1], space_id_counter));

        if(road_sign_ID == space[space_id_counter][1] && road_sign_distance < 50 && road_sign_distance != 0 && ManeuverList.action[ManeuverList.current_id][0] != PARKING)
        {
            driving_mode = CAR_STOP;
            LOG_INFO(adtf_util::cString::Format("Ziel %d Ankunft!!", space[space_id_counter][0]));
            space_id_counter++;

        }
    }




    //    LOG_INFO(adtf_util::cString::Format("driving_mode %d parking_Ready_flag %d ramp_state_flag %d",driving_mode, parking_Ready_flag, ramp_state_flag));
    if ((driving_mode != AVOIDANCE || parking_Ready_flag != PARKING_READY_FLAG_ON) && ramp_state_flag == NO_RAMP && fabs(steering_angle.f32Value) < 20 /* && T_crossing_current_index != 4*/)
        driving_mode = EmergencyBreak(driving_mode);

    return driving_mode;
}

int cOptilm_AutonomousDriving::StateControl(int input_car_state_flag)
{
    float reldistance =0;
    float goal_distance =0;
    float car_direction_heading = 0;

    SpeedDecision(input_car_state_flag);

    if (path_reference_counter == 2)
    {
        temp_road_sign_X = ROAD_SIGN_NOT_IN_XML;
        temp_road_sign_Y = ROAD_SIGN_NOT_IN_XML;
        crossing_sign_found = false;
    }


    switch (input_car_state_flag)
    {
    case CAR_STOP:
        memset(path_reference_point.X, 0, 20*sizeof(float));
        memset(path_reference_point.Y, 0, 20*sizeof(float));
        path_reference_counter = 0;
        vehicle_target_speed = 0;
        ToggleLights(BRAKE, tTrue);

        break;


    case EMERGENCY_BREAK:
        vehicle_target_speed = 0;
        break;

    case LANE_KEEPING:

        //        image_processing_function_switch |= LANE_DETECTION;

//        number_of_reference_point = S_CurveIntervalsChangeDecision(number_of_reference_point);

        CalculateTrackingPoint(number_of_reference_point);


        path_reference_counter = 0;
        last_distance_overall = distance_overall.f32Value;


        if(ManeuverList.ready_flag == tTrue && ManeuverList.action[ManeuverList.current_id][0] == CAR_STOP)
        {
            TransmitDriverStruct(stateCar_COMPLETE , ManeuverList.current_id);
            input_car_state_flag = ManeuverList.action[ManeuverList.current_id][0];
        }

        break;



    case TURN_LEFT:

        number_of_reference_point = 12;
        ToggleLights(BRAKE, tFalse);


        reldistance = distance_overall.f32Value - last_distance_overall;
        if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.001)
        {

            CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

            last_distance_overall = distance_overall.f32Value;
            distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

            if(path_reference_counter == 0)
            {
                temp_heading = fabs(position_data[HEADING]);

                image_processing_function_switch &= ~LANE_DETECTION;
                maneuverid_flag = true;
            }

            if(path_reference_counter < 3 * number_of_reference_point)
                path_reference_counter++;
        }


        if(path_reference_counter >= 3 * number_of_reference_point)
        {
            vehicle_target_speed = 0;
            image_processing_function_switch |= LANE_DETECTION;

            ToggleLights(LEFT, tFalse);

            if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
            {
                 ManeuverList.current_id++;
                 CrossingDetectionDecision();
                TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                maneuverid_flag = false;
            }

            LOG_INFO(adtf_util::cString::Format("turn left stop"));
        }

        else if((fabs(fabs(position_data[HEADING]) - temp_heading)*RADIAN_TO_DEGREES)>65)
        {
            image_processing_function_switch |= LANE_DETECTION;


            vehicle_target_speed = 0.4;
            ToggleLights(LEFT, tFalse);

            if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
            {
                    ManeuverList.current_id++;

                    CrossingDetectionDecision();

                TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                maneuverid_flag = false;

            }
        }


        if(ManeuverList.ready_flag == tTrue && path_reference_counter > 2) //7//10//111
        {
            if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
            {
                input_car_state_flag = LANE_KEEPING;


                //                    LOG_INFO(adtf_util::cString::Format("**** TURN_LEFT finished ****"));
            }
        }

        break;


    case TURN_RIGHT:

        number_of_reference_point = 12;
        ToggleLights(BRAKE, tFalse);

        reldistance = distance_overall.f32Value - last_distance_overall;
        if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.001)
        {

            CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

            last_distance_overall = distance_overall.f32Value;
            distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

            if(path_reference_counter == 0)
            {
                temp_heading = fabs(position_data[HEADING]);

                image_processing_function_switch &= ~LANE_DETECTION;

                maneuverid_flag = true;
            }

            if(path_reference_counter < 2 * number_of_reference_point)
                path_reference_counter++;
        }

        if(path_reference_counter >= 2 * number_of_reference_point)
        {
            vehicle_target_speed = 0;
            image_processing_function_switch |= LANE_DETECTION;

            ToggleLights(RIGHT, tFalse);

            if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
            {
                ManeuverList.current_id++;
                    CrossingDetectionDecision();
                TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                maneuverid_flag = false;

            }

            LOG_INFO(adtf_util::cString::Format("turn right stop"));
        }
        else if((fabs(fabs(position_data[HEADING]) - temp_heading)*RADIAN_TO_DEGREES)>60)//75  70
        {
            image_processing_function_switch |= LANE_DETECTION;
            vehicle_target_speed = 0.40;

            ToggleLights(RIGHT, tFalse);

            if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
            {
                    ManeuverList.current_id++;
                    CrossingDetectionDecision();

                TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                maneuverid_flag = false;

            }
        }


        if(ManeuverList.ready_flag == tTrue && path_reference_counter > 2)
        {
            if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
            {
                input_car_state_flag = LANE_KEEPING;


                ToggleLights(RIGHT, tFalse);

                //                    LOG_INFO(adtf_util::cString::Format("**** TURN_RIGHT finished ****"));

            }
        }
        break;

    case STRAIGHT:

        //        LOG_INFO(adtf_util::cString::Format("T_crossing_direction %d ", T_crossing_direction));

        number_of_reference_point = 12;
        ToggleLights(BRAKE, tFalse);

        if(T_crossing_direction == 0)
        {
            input_car_state_flag = LANE_KEEPING;

            if(ManeuverList.ready_flag == tTrue)
            {
                ManeuverList.current_id++;
                CrossingDetectionDecision();
                TransmitDriverStruct(statecar_running , ManeuverList.current_id);
            }
        }
        else
        {
            reldistance = distance_overall.f32Value - last_distance_overall;
            if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.005)
            {

                CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

                last_distance_overall = distance_overall.f32Value;
                distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

                if(path_reference_counter == 0)
                {
                    image_processing_function_switch &= ~LANE_DETECTION;

                    maneuverid_flag = true;
                }

                if(path_reference_counter < 4 * number_of_reference_point)
                    path_reference_counter++;
            }

            if(path_reference_counter >= 4 * number_of_reference_point)
            {

                vehicle_target_speed = 0;
                image_processing_function_switch |= LANE_DETECTION;
                LOG_INFO(adtf_util::cString::Format("STRAIGHT stop"));

                if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
                {
                        ManeuverList.current_id++;
                        CrossingDetectionDecision();

                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    maneuverid_flag = false;

                }
            }
            else if((path_reference_counter >= ((4*number_of_reference_point)- 24)) ||
                    (T_crossing_direction == 1 && path_reference_counter >= ((4*number_of_reference_point)- 28)))
            {
                image_processing_function_switch |= LANE_DETECTION;

                if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
                {
                     ManeuverList.current_id++;
                        CrossingDetectionDecision();

                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    maneuverid_flag = false;

                }


            }


            if(ManeuverList.ready_flag == tTrue && path_reference_counter > 2)  //1
            {
                if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
                {
                    input_car_state_flag = LANE_KEEPING;


                    //LOG_INFO(adtf_util::cString::Format("**** STRAIGHT finished ****"));
                }
            }


        }

        break;

    case PULL_OUT_LEFT:

        number_of_reference_point = 12;

        if(ManeuverList.current_id != 0 && ManeuverList.action[ManeuverList.current_id - 1][0] == PARKING)
        {
            if(pull_out_light_counter > 65)
            {
                ToggleLights(LEFT, tTrue);
            }
            else if(pull_out_light_counter > 60 && pull_out_light_counter < 65)
            {
                ToggleLights(HAZARD, tFalse);
            }
            else if(pull_out_light_counter > 0 && pull_out_light_counter < 5)
            {
                ToggleLights(HAZARD, tTrue);
            }
        }
        else
        {
            if(pull_out_light_counter == 0)
            {
                pull_out_light_counter = 55;
                ToggleLights(LEFT, tTrue);
            }
        }
        if(pull_out_light_counter > 80)
        {
            ToggleLights(BRAKE, tFalse);

            reldistance = distance_overall.f32Value - last_distance_overall;
            if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.01)
            {

                CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

                last_distance_overall = distance_overall.f32Value;
                distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

                if(path_reference_counter == 0)
                {
                    temp_heading = fabs(position_data[HEADING]);

                    image_processing_function_switch &= ~LANE_DETECTION;

                    maneuverid_flag = true;
                }

                if(path_reference_counter < 4 * number_of_reference_point)
                    path_reference_counter++;
            }

            if(path_reference_counter >= 4 * number_of_reference_point)
            {
                vehicle_target_speed = 0;
                image_processing_function_switch |= LANE_DETECTION;
                ToggleLights(LEFT, tFalse);


                if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
                {
                     ManeuverList.current_id++;
                    CrossingDetectionDecision();
                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    maneuverid_flag = false;

                }
                LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_LEFT stop ****"));
            }
            else if((fabs(fabs(position_data[HEADING]) - temp_heading)*RADIAN_TO_DEGREES)>70)
            {
                image_processing_function_switch |= LANE_DETECTION;
                vehicle_target_speed = 0.4;
                ToggleLights(LEFT, tFalse);

                if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
                {
                     ManeuverList.current_id++;
                    CrossingDetectionDecision();
                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    maneuverid_flag = false;

                }
            }


            if(ManeuverList.ready_flag == tTrue && path_reference_counter > 2)
            {
                if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
                {
                    input_car_state_flag = LANE_KEEPING;

                    ToggleLights(LEFT, tFalse);

                    //                        pull_out_light_counter = 0;

                    //                        LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_LEFT finished ****"));
                }
            }
        }
        else
        {
            if (road_sign_direction != ROAD_SIGN_NOT_IN_XML)
            {
                temp_road_sign_X = road_sign_X;
                temp_road_sign_Y = road_sign_Y;
                crossing_sign_found = true;
            }

            pull_out_light_counter++;
            vehicle_target_speed = 0;
        }
        break;


    case PULL_OUT_RIGHT:

        number_of_reference_point = 12;

        if(ManeuverList.current_id != 0 && ManeuverList.action[ManeuverList.current_id - 1][0] == PARKING)
        {
            if(pull_out_light_counter > 65)
            {
                ToggleLights(RIGHT, tTrue);
            }
            else if(pull_out_light_counter > 60 && pull_out_light_counter < 65)
            {
                ToggleLights(HAZARD, tFalse);
            }
            else if(pull_out_light_counter > 0 && pull_out_light_counter < 5)
            {
                ToggleLights(HAZARD, tTrue);
            }

        }
        else
        {
            if(pull_out_light_counter == 0)
            {
                pull_out_light_counter = 55;
                ToggleLights(RIGHT, tTrue);
            }
        }

        if(pull_out_light_counter > 80)
        {
            ToggleLights(BRAKE, tFalse);

            reldistance = distance_overall.f32Value - last_distance_overall;
            if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.01)
            {

                CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

                last_distance_overall = distance_overall.f32Value;
                distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );
                //LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_RIGHT round %d ****", path_reference_counter));
                if(path_reference_counter == 0)
                {
                    temp_heading = fabs(position_data[HEADING]);

                    image_processing_function_switch &= ~LANE_DETECTION;

                    maneuverid_flag = true;
                }

                if(path_reference_counter < 3*number_of_reference_point)
                    path_reference_counter++;
            }

            if(path_reference_counter >= 3*number_of_reference_point)
            {
                vehicle_target_speed = 0;
                image_processing_function_switch |= LANE_DETECTION;
                ToggleLights(RIGHT, tFalse);

                if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
                {
                     ManeuverList.current_id++;
                    CrossingDetectionDecision();
                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    maneuverid_flag = false;

                }

                //                LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_RIGHT stop ****"));
            }
            else if((fabs(fabs(position_data[HEADING]) - temp_heading)*RADIAN_TO_DEGREES) > 75)
            {
                image_processing_function_switch |= LANE_DETECTION;
                vehicle_target_speed = 0.4;
                ToggleLights(RIGHT, tFalse);

                if(ManeuverList.ready_flag == tTrue && maneuverid_flag == true)
                {
                     ManeuverList.current_id++;
                     CrossingDetectionDecision();

                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    maneuverid_flag = false;

                }
            }

            if(ManeuverList.ready_flag == tTrue && path_reference_counter > 2)
            {
                if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
                {

                    input_car_state_flag = LANE_KEEPING;

                    ToggleLights(RIGHT, tFalse);

                    //                        pull_out_light_counter = 0;

                    //                        LOG_INFO(adtf_util::cString::Format("**** PULL_OUT_RIGHT finished ****"));
                }
            }
        }
        else
        {
            if (road_sign_direction != ROAD_SIGN_NOT_IN_XML)
            {
                temp_road_sign_X = road_sign_X;
                temp_road_sign_Y = road_sign_Y;
                crossing_sign_found = true;
            }

            pull_out_light_counter++;
            vehicle_target_speed = 0;
        }
        break;

    case PARKING:

        number_of_reference_point = 12;

        reldistance = distance_overall.f32Value - last_distance_overall;
        if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.01||(path_reference_counter > number_of_reference_point  && path_reference_counter <= 2*number_of_reference_point ))
        {

            CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

            last_distance_overall = distance_overall.f32Value;
            distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

            //            path_reference_counter++;

            if(path_reference_counter == 0)
            {
                image_processing_function_switch &= ~LANE_DETECTION;

                //                ToggleLights(RIGHT, tFalse);
                //                ToggleLights(LEFT, tTrue);
            }

            if(path_reference_counter < 6 * number_of_reference_point)
                path_reference_counter++;
        }


        if(path_reference_counter <= number_of_reference_point )
        {


            if (first_parking_direction_heading == 0 || first_parking_direction_heading == PI)
            {
                goal_distance = fabs(position_data[Y] - parking_ref_gloabal_coord.Y[number_of_reference_point]) ;
            }
            else
            {
                goal_distance = fabs(position_data[X] - parking_ref_gloabal_coord.X[number_of_reference_point]) ;

            }
            // when reach goal point, stop
            if (goal_distance < 0.025)
            {
                vehicle_target_speed = 0;

                //                LOG_INFO(adtf_util::cString::Format("**** Reach terminal point ****  Reference counter: %d", path_reference_counter));

            }
        }
        else if (path_reference_counter > number_of_reference_point  && path_reference_counter <= 2*number_of_reference_point )
        {
            vehicle_target_speed = 0;
            //            LOG_INFO(adtf_util::cString::Format("**** Reach Rounds end ****"));
            //            path_reference_counter++;
            ToggleLights(LEFT, tFalse);
            ToggleLights(RIGHT, tTrue);
            ToggleLights(REVERSE, tTrue);

        }
        else if (path_reference_counter > 2*number_of_reference_point && path_reference_counter < 5*number_of_reference_point )
        {
            //            LOG_INFO(adtf_util::cString::Format("**** Second phase ****  Reference counter: %d", path_reference_counter));

            //            if(parking_sign_found == true)
            //            {
            //                goal_distance = fabs(position_data[X] - (parking_temp_road_sign_X+0.21)) ;
            //            }
            //            else
            //            {
            //                goal_distance = fabs(position_data[X] - (parking_ref_gloabal_coord.X[0]+0.55)) ;
            //            }


            if (first_parking_direction_heading == 0)
            {
                goal_distance = fabs(position_data[Y] - (parking_temp_road_sign_Y - 0.21)) ;
            }

            //            if (first_parking_direction_heading == 0)
            //            {
            //                goal_distance = fabs(position_data[Y] - (parking_ref_gloabal_coord.Y[0]-0.58)) ;
            //            }
            //            else if ( first_parking_direction_heading == PI )
            //            {
            //                goal_distance = fabs(position_data[Y] - (parking_ref_gloabal_coord.Y[0]+0.58)) ;
            //            }
            //            else if ( first_parking_direction_heading == PI/2 )
            //            {
            //                goal_distance = fabs(position_data[X] - (parking_ref_gloabal_coord.X[0]+0.58)) ;
            //                LOG_INFO(adtf_util::cString::Format("**** Second phase ****  Reference counter: %g", goal_distance));

            //            }
            //            else if ( first_parking_direction_heading == -PI/2 )
            //            {
            //                goal_distance = fabs(position_data[X] - (parking_ref_gloabal_coord.X[0]-0.58)) ;

            //            }




            // when reach goal point, stop
            if (goal_distance < 0.05 || path_reference_counter > 4*number_of_reference_point+7)
            {
                vehicle_target_speed = 0;
                //finish signal
                path_reference_counter =  5*number_of_reference_point;
            }
            //            vehicle_target_speed = 0.4;
            vehicle_target_speed = vehicle_target_speed * -1;
            //                LOG_INFO(adtf_util::cString::Format("**** Phase 2 ****"));
        }
        else
        {
            input_car_state_flag = CAR_STOP;
            vehicle_target_speed = 0;

            path_reference_counter = 0;

            //                SendParkingSpace(&ParkingSpace_output, ManeuverList.action[ManeuverList.id][1], car_est_position.X_Position, car_est_position.Y_Position, 1);

            if(ManeuverList.ready_flag == tTrue)
            {
                if(ManeuverList.action[ManeuverList.current_id + 1][0] == CAR_STOP)
                {
                    TransmitDriverStruct(stateCar_COMPLETE , ManeuverList.current_id);
                    ManeuverList.current_id++;
                    input_car_state_flag = ManeuverList.action[ManeuverList.current_id][0];
                    ToggleLights(HAZARD, tTrue);
                    pull_out_light_counter = 0;
                }
                else
                {
                    ManeuverList.current_id++;
                    input_car_state_flag = ManeuverList.action[ManeuverList.current_id][0];
                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                    pull_out_light_counter = 0;
                }
            }

            parking_sign_found = false;
            ToggleLights(RIGHT, tFalse);
            ToggleLights(REVERSE, tFalse);

            LOG_INFO(adtf_util::cString::Format("**** PARKING finished ****"));
        }
        break;

    case AVOIDANCE:

        number_of_reference_point = 12;

        reldistance = distance_overall.f32Value - last_distance_overall;
        if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.005)
        {
            //            LOG_INFO(adtf_util::cString::Format("Avoidance rounds %d",path_reference_counter));
            CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

            last_distance_overall = distance_overall.f32Value;
            distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

            //    #ifdef OUTPUT_TURN_AROUND_COUTER_DEBUG
            //                LOG_INFO(adtf_util::cString::Format("AVOIDANCE Ziel%d distance %f currentDist %f lastDist %f reldist %f",turn_around_reference_counter,dist,distance_overall,last_distance_overall,rdist));
            //    #endif
            if(path_reference_counter == 0)
            {
                image_processing_function_switch &= ~LANE_DETECTION;
                avoidance_car_HeadingAngle = (int)current_car_HeadingAngle;
            }
            if(path_reference_counter < 3*number_of_reference_point)
                path_reference_counter++;
        }

        if(path_reference_counter >= 3 * number_of_reference_point)
        {
            vehicle_target_speed = 0;

            ToggleLights(LEFT, tFalse);
            ToggleLights(RIGHT, tFalse);

            image_processing_function_switch |= LANE_DETECTION;
        }


        if((((avoidance_car_HeadingAngle == 0 || avoidance_car_HeadingAngle == 180) && fabs(position_data[Y] - avoidance_ref_global_coord.Y[2*number_of_reference_point])<0.07) ||
            ((avoidance_car_HeadingAngle == 90 || avoidance_car_HeadingAngle == -90) && fabs(position_data[X] - avoidance_ref_global_coord.X[2*number_of_reference_point])<0.07)) &&
                path_reference_counter >= 2 * number_of_reference_point - 6 ) //7//10//111
        {
            image_processing_function_switch |= LANE_DETECTION;
        }


        if((lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE) && path_reference_counter > 12)
        {
            input_car_state_flag = LANE_KEEPING;
            if(avoidance.comeback_flag == 0)
            {
                avoidance.comeback_flag = 1;
                avoidance.comeback_wait_counter = avoidance_comeBack_counter;
                path_reference_counter = 0;
            }
            else if(avoidance.comeback_flag == 2 )
            {
                avoidance.flag = tFalse;
                avoidance.comeback_flag = 0;
                LOG_INFO(adtf_util::cString::Format("**** Avoidance finished ****"));
            }

            ToggleLights(LEFT, tFalse);
            ToggleLights(RIGHT, tFalse);
        }

        break;

    case MERGE_LEFT:

        number_of_reference_point = 12;
        ToggleLights(BRAKE, tFalse);

        reldistance = distance_overall.f32Value - last_distance_overall;
        if (path_reference_counter == 0 || fabs(reldistance) > distance_of_first_to_second_point - 0.001)
        {

            CalculateTurnAroundReferencePoint(input_car_state_flag, number_of_reference_point, path_reference_counter);

            last_distance_overall = distance_overall.f32Value;
            distance_of_first_to_second_point = GetDistanceBetweenCoordinates(path_reference_point.X[1], path_reference_point.Y[1] , path_reference_point.X[0], path_reference_point.Y[0] );

            if(path_reference_counter == 0)
            {
                temp_heading = fabs(position_data[HEADING]);

                image_processing_function_switch &= ~LANE_DETECTION;

                if(ManeuverList.ready_flag == tTrue)
                {
                    ManeuverList.current_id++;
                    CrossingDetectionDecision();

                    TransmitDriverStruct(statecar_running , ManeuverList.current_id);
                }
            }

            if(path_reference_counter < 5 * number_of_reference_point)
                path_reference_counter++;
        }


        if(path_reference_counter >= 5 * number_of_reference_point)
        {
            vehicle_target_speed = 0;

            image_processing_function_switch |= LANE_DETECTION;
            ToggleLights(LEFT, tFalse);

            LOG_INFO(adtf_util::cString::Format("Merge left stop"));
        }
        else if((((car_direction_heading == 0 || car_direction_heading == PI) && fabs(position_data[Y] - merg_left_ref_global_coord.Y[5*number_of_reference_point])<0.15) ||
            ((car_direction_heading == PI/2 || car_direction_heading == -PI/2) && fabs(position_data[X] - merg_left_ref_global_coord.X[5*number_of_reference_point])<0.15)) &&
                path_reference_counter > 4*number_of_reference_point - 6)
        {
            image_processing_function_switch |= LANE_DETECTION;
            ToggleLights(LEFT, tFalse);
        }

        if((lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE) && path_reference_counter > 2)
        {
            input_car_state_flag = LANE_KEEPING;

            //                    LOG_INFO(adtf_util::cString::Format("**** TURN_LEFT finished ****"));
        }

//        if(path_reference_counter >= 2 * number_of_reference_point && US_data.tRearLeft.f32Value < 100)
//        {
//            vehicle_target_speed = 0;
//        }

        break;



    default:
        number_of_reference_point = 12;
        memset(path_reference_point.X, 0, 20*sizeof(float));
        memset(path_reference_point.Y, 0, 20*sizeof(float));
        path_reference_counter = 0;
        vehicle_target_speed = 0;

        break;
    }

    if(emergency_vehicle_flag == true)
        vehicle_target_speed = 0;
    else if(emergency_vehicle_shift != 0)
        vehicle_target_speed = 0.35;


    static float last_traget_speed = 0;
    if(vehicle_target_speed >= last_traget_speed)
    {
        ToggleLights(BRAKE, tFalse);
        last_traget_speed = vehicle_target_speed;
    }
    else if(vehicle_target_speed < (last_traget_speed - 0.05))
    {
        ToggleLights(BRAKE, tTrue);
        last_traget_speed = vehicle_target_speed;
    }


//    image_processing_function_switch |= CHILD_DETECTION;
//    image_processing_function_switch |= ADULT_DETECTION;

    TransmitDriveModel(input_car_state_flag);
    TransmitImageControl((float)image_processing_function_switch);
    TransmitTrajectoryPointAndSpeed(number_of_reference_point, vehicle_target_speed);


    return input_car_state_flag;
}

tResult cOptilm_AutonomousDriving::SpeedDecision(int driving_mode)
{
    switch (driving_mode)
    {
    case CAR_STOP:
        vehicle_target_speed = 0;
        break;

    case EMERGENCY_BREAK:
        vehicle_target_speed = 0;
        break;

    case PARKING:
        vehicle_target_speed = 0.45;
        break;

    case AVOIDANCE:
        vehicle_target_speed = 0.5;
        break;

    case LANE_KEEPING:
        if(crossing_flag != CROSSING_FLAG_OFF) //in curve of single lane
        {

            vehicle_target_speed = vehicle_target_speed - ( m_vehicle_max_speed-0.4)/30;
            if (vehicle_target_speed < 0.4)
                vehicle_target_speed = 0.4;
        }
        else if (avoidance.flag == true)
        {
            vehicle_target_speed = 0.5;
        }
        else if (ramp_state_flag != NO_RAMP)
        {
            float min_speed = 0.5;
            switch (ramp_state_flag)
            {
            case RAMP_UP:
                min_speed = 0.5;
                break;
            case RAMP_IN:
                min_speed = 0.4;
                break;
            case RAMP_DOWN:
                min_speed = 0.3;
                break;
            case 4:
                min_speed = 0.4;
                break;
            default:
                min_speed = 0.5;
                break;
            }
            vehicle_target_speed = vehicle_target_speed - ( m_vehicle_max_speed - min_speed)/20;
            if (vehicle_target_speed < min_speed)
                vehicle_target_speed = min_speed;
        }
        else if (find_obstacle == true)
        {
            if(avoidance_permit_flag == 1)
            {
                vehicle_target_speed = vehicle_target_speed - ( m_vehicle_max_speed-0.5)/15;
                if (vehicle_target_speed < 0.5)
                    vehicle_target_speed = 0.5;
            }
            else
            {
                vehicle_target_speed = vehicle_target_speed - ( m_vehicle_max_speed-0.4)/30;
                if (vehicle_target_speed < 0.4)
                    vehicle_target_speed = 0.4;
            }
        }
        else if (pedestrian_flag == true)
        {
            vehicle_target_speed = vehicle_target_speed - ( m_vehicle_max_speed-0.4)/20;
            if (vehicle_target_speed < 0.4)
                vehicle_target_speed = 0.4;

        }
        else if(parking_Ready_flag == PARKING_READY_FLAG_ON)
        {
            vehicle_target_speed = vehicle_target_speed - ( m_vehicle_max_speed-0.4)/30;
            if (vehicle_target_speed < 0.4)
                vehicle_target_speed = 0.4;
        }

        else
        {
            if(lane.child_flag == 1)
            {
                vehicle_target_speed = 0.4;
            }
            else if(LowSpeedRegionDecision())
            {
                vehicle_target_speed = 0.5;
            }
            else if(S_CurveDecisionflag == true)
            {
                vehicle_target_speed = 0.55;
            }
            else if(lane.detect_mode == LSEARCH || lane.detect_mode == SL_SEARCH)
            {
                vehicle_target_speed = 0.4;
            }
            else if (vehicle_target_speed < m_vehicle_min_speed)
            {
                vehicle_target_speed = vehicle_target_speed + ( m_vehicle_min_speed - vehicle_target_speed)/25 + 0.005;  //20
            }
            else
            {
                const float predict_time = 2.0;
                float predict_distance = 0;
                float predict_lane_bias = 0;

                predict_distance = (vehicle_target_speed/*vehicle_speed.f32Value*/ * predict_time) * 100;
                predict_lane_bias = (vehicle_path_polynomial[K] * (predict_distance * predict_distance)) + (predict_distance * vehicle_path_polynomial[M]) + vehicle_path_polynomial[B];
                //                    LOG_INFO(adtf_util::cString::Format("predict_lane_bias:% g  lane.kurve_parameter[B]:%g", predict_lane_bias, lane.kurve_parameter[B]));

//                if(lane.detect_mode == LTRACE && fabs(predict_lane_bias) < 15 &&  fabs(lane.kurve_parameter[B]) < 10) // straight and two lanes recoginzed
                if((lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE) && fabs(predict_lane_bias) < 10) // straight and two lanes recoginzed
                {
                    if (vehicle_target_speed >= m_vehicle_max_speed - 0.01)
                        vehicle_target_speed = m_vehicle_max_speed;
                    else
                        vehicle_target_speed = vehicle_target_speed + sin((m_vehicle_max_speed-vehicle_target_speed)*PI/(m_vehicle_max_speed-m_vehicle_min_speed)- 0.01)*0.01;
                }
                else //in curve of single lane
                {
                    if (vehicle_target_speed <= m_vehicle_min_speed + 0.01)
                        vehicle_target_speed = m_vehicle_min_speed;
                    else
                        vehicle_target_speed = vehicle_target_speed - sin((vehicle_target_speed-m_vehicle_min_speed)*PI/(m_vehicle_max_speed-m_vehicle_min_speed)- 0.01)*0.03;
                }

//                double offset = fabs(path_reference_point.Y[5]);
//                if(offset > 0.2)
//                    offset = 0.2;
//                vehicle_target_speed = SpeedNetworkFunction(offset);


            }
            //only for debug
            //            vehicle_target_speed = 0.5;

        }
        break;

    default:
        vehicle_target_speed = 0.5;
        break;
    }

    //    LOG_INFO(adtf_util::cString::Format("driving_mode_flag %d vehicle_target_speed %g",driving_mode_flag, vehicle_target_speed));
    RETURN_NOERROR;
}

int cOptilm_AutonomousDriving::AvoidanceProcess(int driving_mode)
{
    /* Avoidance maneuver decision from ultrasensor
     *
     *
    */
    int index = 0;
    for(index = 0; index < avoidance_nummer;index++)
    {
        if (position_data[X] < avoidance_section_boundary[index].right && position_data[X] > avoidance_section_boundary[index].left &&
            position_data[Y] < avoidance_section_boundary[index].top   && position_data[Y] > avoidance_section_boundary[index].bom)
        {
            avoidance_region_nummer = index + 1;

            if(avoidance_region_nummer == 1)
            {
                avoidance_permit_flag = 1;
                break;
            }
            if(avoidance_region_nummer == 2 && fabs((position_data[HEADING] * RADIAN_TO_DEGREES) - avoidance_section_boundary[index].HeadingAngle) < 15)
            {
                avoidance_permit_flag = 1;
                break;
            }
            else
            {
                avoidance_permit_flag = 0;
                avoidance_region_nummer = 0;
            }
                        //            LOG_INFO(adtf_util::cString::Format("Avoidance permit block %d", index));
        }
        else
        {
            avoidance_permit_flag = 0;
            avoidance_region_nummer = 0;
        }
    }

    if(avoidance.flag == tFalse)
        ObstacleDetection();
    else
        find_obstacle = false;



    if (avoidance_permit_flag == 1)
    {
        if(find_obstacle == true && obstacle_from_laserscanner.bom < avoidance_start_distance/*&& obstacle_from_Ultrasonic.counter > 2*/)
        {
            if(avoidance.flag == tFalse)
            {
                path_reference_counter = 0;
                avoidance.comeback_flag = 0;
                avoidance.flag = tTrue;

                driving_mode = AVOIDANCE;


                ToggleLights(LEFT, tTrue);

                LOG_INFO(adtf_util::cString::Format("Avoidance start %g",obstacle_from_laserscanner.bom));
            }
        }
    }

    if(avoidance.flag == tTrue)
    {
        if(US_coordinate.side_right.Y  < (avoidance_side_distance + (15))  && avoidance.comeback_flag == 1)
        {
            avoidance.comeback_wait_counter = avoidance_comeBack_counter;
        }
        else if(US_coordinate.side_right.Y > (avoidance_side_distance + (15)) && avoidance.comeback_flag == 1 && driving_mode == LANE_KEEPING)
        {
            if(avoidance.comeback_wait_counter > 0)
                avoidance.comeback_wait_counter--;

            if(avoidance.comeback_wait_counter <= 0)
                avoidance.comeback_flag = 2;

            ToggleLights(RIGHT, tTrue);
        }

        if(avoidance.comeback_flag == 2 )
        {
            //            avoidance.flag = tFalse;
            driving_mode = AVOIDANCE;
        }
    }


    return driving_mode;
}


tResult cOptilm_AutonomousDriving::ObstacleDetection(void)
{
    int index = 0;
    float car_collision_y_range = 0;
    X_Y_POINT temp_point[360];
    int temp_point_counter = 0;


    obstacle_from_laserscanner.bom = 1000;
    obstacle_from_laserscanner.top = 0;
    obstacle_from_laserscanner.left = -500;
    obstacle_from_laserscanner.right = 500;


    if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
    {
        for(index = 0; index < laser_scann.number_of_scan_point; index++)
        {
            if(laser_scann.coordinate[index].X != 0 && laser_scann.coordinate[index].Y != 0)
            {
                car_collision_y_range = (lane.kurve_parameter[K] * (laser_scann.coordinate[index].X * laser_scann.coordinate[index].X)) + (laser_scann.coordinate[index].X * lane.kurve_parameter[M]) + lane.kurve_parameter[B];
                if(fabs(laser_scann.coordinate[index].Y - car_collision_y_range) < (50/2))
                {
                    if(laser_scann.coordinate[index].X < (obstacle_detection_distance + LASERSCANNER_CENTER_TO_CAR_FRONT))
                    {
                        temp_point[temp_point_counter].X = laser_scann.coordinate[index].X;
                        temp_point[temp_point_counter].Y = laser_scann.coordinate[index].Y;
                        temp_point_counter++;
                        //                    LOG_INFO(cString::Format("Point %d, X: %g  Y: %g",index,laser_scann.coordinate[index].X, laser_scann.coordinate[index].Y));
                    }
                }
            }
        }
    }
    else
    {
        for(index = 0; index < laser_scann.number_of_scan_point; index++)
        {
            if(laser_scann.coordinate[index].X != 0 && laser_scann.coordinate[index].Y != 0)
            {
                car_collision_y_range = (vehicle_path_polynomial[K] * (laser_scann.coordinate[index].X * laser_scann.coordinate[index].X)) + (laser_scann.coordinate[index].X * vehicle_path_polynomial[M]);
                if(fabs(laser_scann.coordinate[index].Y - car_collision_y_range) < (35/2))
                {
                    if(laser_scann.coordinate[index].X < (obstacle_detection_distance + LASERSCANNER_CENTER_TO_CAR_FRONT))
                    {
                        temp_point[temp_point_counter].X = laser_scann.coordinate[index].X;
                        temp_point[temp_point_counter].Y = laser_scann.coordinate[index].Y;
                        temp_point_counter++;
                        //                    LOG_INFO(cString::Format("Point %d, X: %g  Y: %g",index,laser_scann.coordinate[index].X, laser_scann.coordinate[index].Y));
                    }
                }
            }
        }
    }

    for(index = 0; index < temp_point_counter; index++)
    {
        if(temp_point[index].X > obstacle_from_laserscanner.top)
            obstacle_from_laserscanner.top = temp_point[index].X;
        if(temp_point[index].X < obstacle_from_laserscanner.bom)
            obstacle_from_laserscanner.bom = temp_point[index].X;

        if(temp_point[index].Y > obstacle_from_laserscanner.left)
            obstacle_from_laserscanner.left = temp_point[index].Y;
        if(temp_point[index].Y < obstacle_from_laserscanner.right)
            obstacle_from_laserscanner.right = temp_point[index].Y;

    }
    //    LOG_INFO(cString::Format("left: %g  right: %g",obstacle_from_laserscanner.left, obstacle_from_laserscanner.right));

    if(fabs(obstacle_from_laserscanner.left - obstacle_from_laserscanner.right) > 5 && obstacle_from_laserscanner.bom < (obstacle_detection_distance + LASERSCANNER_CENTER_TO_CAR_FRONT) && temp_point_counter != 0)
    {
        find_obstacle = true;
        //        LOG_INFO(adtf_util::cString::Format("Obstacle find"));
    }
    else
    {
        find_obstacle = false;
    }

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::ChildDetection()
{
    int index = 0;
    tBool  detection_flag = tFalse;

    for(index = 0; index < child_nummer; index++)
    {
        tFloat32 Tx = (position_data[X] + cos(position_data[HEADING]*DEGREES_TO_RADIAN)*0.5);
        tFloat32 Ty = (position_data[Y] + sin(position_data[HEADING]*DEGREES_TO_RADIAN)*0.5);
        if (Tx < child_section_boundary[index].right && Tx > child_section_boundary[index].left &&
            Ty < child_section_boundary[index].top && Ty > child_section_boundary[index].bom)
        {
            detection_flag = tTrue;
            //bLOG_INFO(adtf_util::cString::Format("Child Detection On"));
            break;
        }
        else
        {
            detection_flag = tFalse;
            lane.child_flag = 0;
        }
    }


    if(detection_flag == tTrue && ManeuverList.action[ManeuverList.current_id][0] != MERGE_LEFT)
    {
//        LOG_INFO(adtf_util::cString::Format("Child Detection On"));
        image_processing_function_switch |= CHILD_DETECTION;
    }
    else
    {
//        LOG_INFO(adtf_util::cString::Format("Child Detection Off"));
        image_processing_function_switch &= ~CHILD_DETECTION;
    }


    RETURN_NOERROR;
}

int cOptilm_AutonomousDriving::PedestrianDecision(int driving_mode_flag)
{
    //    int index = 0;

    if (road_sign_ID == PEDESTRIAN_CROSSING && road_sign_distance < pedestrian_road_sign_distance && pedestrian_flag == false)
    {
        last_distance_overall_for_pedestrian = distance_overall.f32Value;
        pedestrian_flag = true;
        pedestrian_stop_counter = 180;
    }
    if ((position_data[X] < pedestrian_section_boundary[0].right && position_data[X] > pedestrian_section_boundary[0].left &&
         position_data[Y] < pedestrian_section_boundary[0].top   && position_data[Y] > pedestrian_section_boundary[0].bom) && pedestrian_flag == false)
    {
        pedestrian_flag = true;
        pedestrian_stop_counter = 180;
    }

    else if (pedestrian_flag == true)
    {
        //        if(KI_adult)
        //            image_processing_function_switch |= ADULT_DETECTION;
        //        if(KI_child)
        //            image_processing_function_switch |= CHILD_DETECTION;


        ////        if(adult_flag != 0 || child_flag != 0)
        ////            lane_follow_speed = 0.4;

        //        if(adult_flag == 2 && KI_adult == tTrue)
        //        {
        //            if(temp_crossing_stop_distance < 15)
        //                driving_mode_flag = CAR_STOP;
        //            pedestrian_stop_counter = 20;
        //        }
        //        else if(child_flag == 1 && KI_child == tTrue)
        //        {
        //            if(temp_crossing_stop_distance < 15)
        //                driving_mode_flag = CAR_STOP;
        //            pedestrian_stop_counter = 20;
        //        }



        if (position_data[X] < pedestrian_section_boundary[0].right && position_data[X] > pedestrian_section_boundary[0].left &&
                position_data[Y] < pedestrian_section_boundary[0].top   && position_data[Y] > pedestrian_section_boundary[0].bom)
        {

            if(pedestrian_stop_counter > 0)
            {
                pedestrian_stop_counter--;

                if(pedestrian_stop_counter <= 0 && driving_mode_flag == CAR_STOP)
                {
                    driving_mode_flag = LANE_KEEPING;
                    image_processing_function_switch |= LANE_DETECTION;

                    //clos audilt detection
                    image_processing_function_switch &= ~ADULT_DETECTION;
                }
                else if(pedestrian_stop_counter < 170)
                {
                    driving_mode_flag = CAR_STOP;
                    image_processing_function_switch &= ~LANE_DETECTION;


                    //open audilt detection
                    image_processing_function_switch |= ADULT_DETECTION;
                }

            }

            //            LOG_INFO(cString::Format("pedestrian_stop_counter: %d",pedestrian_stop_counter));
        }
        else
        {
            if(distance_overall.f32Value > (last_distance_overall_for_pedestrian + (pedestrian_low_speed_distance*0.01)))
            {
                pedestrian_flag = false;
                pedestrian_low_speed_distance = 0;
                //                image_processing_function_switch &= ~ADULT_DETECTION;
                //                image_processing_function_switch &= ~CHILD_DETECTION;
            }
        }


    }
    return driving_mode_flag;
}



int cOptilm_AutonomousDriving::ParkingProcess(int driving_mode)
{
    //    LOG_INFO(adtf_util::cString::Format("**** Maneuver ID %d ****", maneuver_ID ));
    //    LOG_INFO(adtf_util::cString::Format("**** Road mark iD %d ****", road_marker_ID ));
    //    LOG_INFO(adtf_util::cString::Format("**** Mark distance %d ****", marker_distance ));
    short parking_id = 1;


    if(parking_Ready_flag == PARKING_READY_FLAG_ON)
    {
        //Parking Distance
        temp_parking_distance -= ((distance_overall.f32Value - last_distance_overall) * 100);
        last_distance_overall = distance_overall.f32Value;

        //Parking Distance Process
        //LOG_INFO(adtf_util::cString::Format("**** Temp parking distance %g ****", temp_parking_distance));

        if(ManeuverList.ready_flag == tTrue)
        {
            //get parking id and transform park id from 8 to 4
            if( ManeuverList.action[ManeuverList.current_id][1] > 4)
            {
                if(ManeuverList.action[ManeuverList.current_id][1] == 5)
                    parking_id = 1;
                else if(ManeuverList.action[ManeuverList.current_id][1] == 6)
                    parking_id = 2;
                else if(ManeuverList.action[ManeuverList.current_id][1] == 7)
                    parking_id = 3;
                else if(ManeuverList.action[ManeuverList.current_id][1] == 8)
                    parking_id = 4;
            }
            else
                parking_id =  ManeuverList.action[ManeuverList.current_id][1];
        }

        if((position_data[X] + 0.52) >= (parking_temp_road_sign_X - (slot_distance[parking_id - 1] * 0.01)))
        {
            driving_mode = PARKING;
            parking_Ready_flag = PARKING_READY_FLAG_OFF;
            temp_parking_distance = 0;

            LOG_INFO(adtf_util::cString::Format("**** parking_id %d %d ****", parking_id, ManeuverList.action[ManeuverList.current_id][1] ));
        }
    }

    //parking decision
    else if(ManeuverList.action[ManeuverList.current_id][0] == PARKING && road_sign_ID == PARKEN && road_sign_distance < parking_road_sign_distance)//70
    {
        ToggleLights(RIGHT, tTrue);

        temp_parking_distance = road_sign_distance;
        parking_Ready_flag = PARKING_READY_FLAG_ON;
        last_distance_overall = distance_overall.f32Value;

        if (road_sign_direction != ROAD_SIGN_NOT_IN_XML)
        {
            parking_temp_road_sign_X = road_sign_X;
            parking_temp_road_sign_Y = road_sign_Y;
            parking_sign_found = true;
        }

        if(parking_sign_found == false)
        {
            if(ManeuverList.action[ManeuverList.current_id][1] > 4)
            {
                parking_temp_road_sign_X = 11.644;
                parking_temp_road_sign_Y = 6.865;
            }
            else
            {
                parking_temp_road_sign_X = 5.240;
                parking_temp_road_sign_Y = 2.805;
            }
        }




        //        LOG_INFO(adtf_util::cString::Format("slot_distance[0] = %g", (float)slot_distance[0]));
        //        LOG_INFO(adtf_util::cString::Format("slot_distance[1] = %g", (float)slot_distance[1]));
        //        LOG_INFO(adtf_util::cString::Format("slot_distance[2] = %g", (float)slot_distance[2]));
        //        LOG_INFO(adtf_util::cString::Format("slot_distance[3] = %g", (float)slot_distance[3]));
        //        LOG_INFO(adtf_util::cString::Format("**** Parking recognized ****"));
    }



    return driving_mode;
}

int cOptilm_AutonomousDriving::EmergencyBreak(int driving_mode)
{
    int index = 0;
    float min_break_distance = 0;

    if(vehicle_speed.f32Value > 0.0)
    {
        min_break_distance = (vehicle_speed.f32Value * *m_front_min_break_time) * 100;
        if(min_break_distance < m_min_break_distance)
            min_break_distance = m_min_break_distance;

    }
    else if(vehicle_speed.f32Value < 0.0)
    {
        min_break_distance = (vehicle_speed.f32Value * m_rear_min_break_time) * 100;
        if(min_break_distance > -m_min_break_distance)
            min_break_distance = -m_min_break_distance;
    }
    else if(vehicle_speed.f32Value == 0.0)
    {
        min_break_distance =  m_min_break_distance;
    }

    min_break_distance = min_break_distance + LASERSCANNER_CENTER_TO_CAR_FRONT;



    float car_collision_y_range = 0;
    int break_counter = 0;

    for(index = 0; index < laser_scann.number_of_scan_point; index++)
    {
        if(laser_scann.coordinate[index].X != 0 && laser_scann.coordinate[index].Y != 0)
        {
            car_collision_y_range = (vehicle_path_polynomial[K] * (laser_scann.coordinate[index].X * laser_scann.coordinate[index].X)) + (laser_scann.coordinate[index].X * vehicle_path_polynomial[M]);
            if(fabs(laser_scann.coordinate[index].Y - car_collision_y_range) < (35/2))
            {

                if(laser_scann.coordinate[index].X < m_min_break_distance)
                {
                    if(vehicle_speed.f32Value >= 0 && break_counter > 2)
                    {
                        if(emergency_break_last_driving_model_flag == NO_BREAK_INDEX_FLAG)
                            emergency_break_last_driving_model_flag = driving_mode;

                        emergency_break_counter = 30;
                        emergency_break_flag = ON;
                    }

                    break_counter++;
                    laser_scann.point_state[index] = EMERGENCY;
                }
                else
                {
                    laser_scann.point_state[index] = WARNING;
                }
            }
            else
            {
                laser_scann.point_state[index] = NORMAL;
            }
        }
        else
        {
            laser_scann.point_state[index] = NORMAL;
        }
    }


    if(US_coordinate.rear_left.X > (-min_break_distance - 30))
    {
        US_coordinate.r_l_state = EMERGENCY;
    }
    else
        US_coordinate.r_l_state = NORMAL;

    if(US_coordinate.rear_center.X > (-min_break_distance - 30))
    {
        US_coordinate.r_c_state = EMERGENCY;
    }
    else
        US_coordinate.r_c_state = NORMAL;

    if(US_coordinate.rear_right.X > (-min_break_distance - 30))
    {
        US_coordinate.r_r_state = EMERGENCY;
    }
    else
        US_coordinate.r_r_state = EMERGENCY;




    if(emergency_break_counter > 0)
    {
        if(driving_mode == CAR_STOP)
        {
            driving_mode = CAR_STOP;
            emergency_break_counter = 0;
            emergency_break_flag = OFF;
            emergency_break_last_driving_model_flag = NO_BREAK_INDEX_FLAG;
            image_processing_function_switch |= LANE_DETECTION;

        }
        else
        {
            emergency_break_counter--;
            if(emergency_break_counter <= 0)
            {
                vehicle_target_speed = 0;
                emergency_break_flag = OFF;
            }

            driving_mode = EMERGENCY_BREAK;
            image_processing_function_switch &= ~LANE_DETECTION;


            ToggleLights(HAZARD, tTrue);
            ToggleLights(BRAKE, tTrue);
        }

        // for debug
        //        LOG_INFO(adtf_util::cString::Format("(EmergencyBreak) emergency break counter %d",emergency_break_counter));
    }
    else if(emergency_break_last_driving_model_flag != NO_BREAK_INDEX_FLAG && emergency_break_counter == 0)
    {

        image_processing_function_switch |= LANE_DETECTION;

        if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
        {
            driving_mode = emergency_break_last_driving_model_flag;
            emergency_break_last_driving_model_flag = NO_BREAK_INDEX_FLAG;
        }


        ToggleLights(HAZARD, tFalse);
    }


    TransmitEmergencyBreakFlag(emergency_break_flag);

    return driving_mode;
}

int cOptilm_AutonomousDriving::CrossingDecision(int driving_mode)
{
    int index = 0;
    T_crossing_direction = -1;
    tFloat32 Tx = 0, Ty = 0;
    float relative_distance_intersection = 0;

    if(crossing_flag != CROSSING_FLAG_OFF)
    {
        Tx = static_cast<tFloat32>(position_data[X] + cos(current_car_HeadingAngle*DEGREES_TO_RADIAN)*0.7);
        Ty = static_cast<tFloat32>(position_data[Y] + sin(current_car_HeadingAngle*DEGREES_TO_RADIAN)*0.7);
        //        LOG_INFO(cString::Format("Position x %g Position y %g", Tx, Ty));

        for(index = 0; index < T_crossing_nummer; index++)
        {
            if (Tx < T_section_boundary[index].right && Tx > T_section_boundary[index].left &&
                    Ty < T_section_boundary[index].top   && Ty > T_section_boundary[index].bom)
            {
                if(current_car_HeadingAngle == T_section_boundary[index].HeadingAngle)
                    T_crossing_direction = 2;
                else if((current_car_HeadingAngle - T_section_boundary[index].HeadingAngle) == -90 || (current_car_HeadingAngle - T_section_boundary[index].HeadingAngle) == 270)
                    T_crossing_direction = 1;
                else if((current_car_HeadingAngle - T_section_boundary[index].HeadingAngle) == 90  || (current_car_HeadingAngle - T_section_boundary[index].HeadingAngle) == -270)
                    T_crossing_direction = 0;

                T_crossing_current_index = index + 1;
                //for debug
                //          LOG_INFO(cString::Format("T crossing section id %d Direction %d heading %f", index, T_crossing_direction, T_section_boundary[index].HeadingAngle));
                break;

            }
        }
    }


    //for debug
    //    LOG_INFO(adtf_util::cString::Format("***********Road Sign %d", road_sign_ID));

    //open crossing flag distance in Cm
    if((road_sign_ID == UNMARKED_INTERSECTION || road_sign_ID == STOP_GIVE_WAY || road_sign_ID == HAVEWAY || road_sign_ID == GIVE_WAY ) && road_sign_distance < crossing_road_sign_distance && road_sign_direction != ROAD_SIGN_NOT_IN_XML)
    {
        //        if(pedestrian_flag == tTrue)
        //        {
        //            pedestrian_flag = tFalse;
        //            image_processing_function_switch &= ~ADULT_DETECTION;
        //            image_processing_function_switch &= ~CHILD_DETECTION;
        //        }

        temp_road_sign_ID = road_sign_ID;


        if (road_sign_direction != ROAD_SIGN_NOT_IN_XML)
        {
            temp_road_sign_X = road_sign_X;
            temp_road_sign_Y = road_sign_Y;
            temp_car_direction_heading = current_car_HeadingAngle;
            crossing_sign_found = true;
        }


        //debug
        // LOG_INFO(adtf_util::cString::Format("***********Road Sign %d", temp_road_sign_ID));

        if((road_sign_ID == HAVEWAY                    && ManeuverList.action[ManeuverList.current_id][0] != TURN_LEFT) ||
                (road_sign_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.current_id][0] == TURN_RIGHT))
        {
            stop_decision_flag = NOSTOP_DECISION;
        }
        else
        {
            stop_decision_flag = STOP_DECISION;
            crossing_stop_wait_counter = CrossingWaitTimeDecision(temp_road_sign_ID);
        }

        if(crossing_sign_found == true)
            crossing_flag = CROSSING_FLAG_TRAFFIC_SIGNS;


        if (ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT)
        {
            ToggleLights(LEFT, tTrue);
        }
        else if (ManeuverList.action[ManeuverList.current_id][0] == TURN_RIGHT)
        {
            ToggleLights(RIGHT, tTrue);
        }


        //        LOG_INFO(adtf_util::cString::Format("Crossing traffic sign recognized %d",temp_road_sign_ID));
    }
    else if(crossing_flag == CROSSING_FLAG_TRAFFIC_SIGNS)
    {
        if (ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT)
        {
            nostop_crossing = nostop_crossing_left;
            stop_crossing = stop_crossing_left;
        }
        else if(ManeuverList.action[ManeuverList.current_id][0] == TURN_RIGHT)
        {
            nostop_crossing = nostop_crossing_right;
            stop_crossing = stop_crossing_right;
            //            LOG_INFO(adtf_util::cString::Format("Right Stop distance %g",(float)stop_crossing));
        }
        else
        {
            nostop_crossing = nostop_crossing_strgiht;
            stop_crossing = stop_crossing_strgiht;
        }

        //XH**
        if (temp_car_direction_heading == 0 || temp_car_direction_heading == 180)
            temp_crossing_stop_distance = fabs(position_data[X] - temp_road_sign_X)*100;
        else
            temp_crossing_stop_distance = fabs(position_data[Y] - temp_road_sign_Y)*100;
        //            LOG_INFO(adtf_util::cString::Format("Stop distance %g car position %g road sign position %g car direction %g",temp_crossing_stop_distance, position_data[X], temp_road_sign_X,temp_car_direction_heading));
        //**XH


        //close the image processing
        if(temp_crossing_stop_distance < 20 && T_crossing_direction != 0)
        {
            image_processing_function_switch &= ~LANE_DETECTION;
            image_processing_function_switch &= ~ADULT_DETECTION;
            image_processing_function_switch &= ~CHILD_DETECTION;
        }

        if(stop_decision_flag == STOP_DECISION && temp_crossing_stop_distance <= stop_crossing && driving_mode != CAR_STOP)//-3//15//29 //stop_decision_flag = stop_decision
        {
            LOG_INFO(adtf_util::cString::Format("Stop distance %g",stop_crossing));
            driving_mode = CAR_STOP;
            temp_crossing_stop_distance = 0;
        }

        else if(stop_decision_flag == NOSTOP_DECISION && temp_crossing_stop_distance <= nostop_crossing)  //-27//left is 4, right is -14
        {
            if(driving_mode != ManeuverList.action[ManeuverList.current_id][0])
                path_reference_counter = 0;

            driving_mode = ManeuverList.action[ManeuverList.current_id][0];

            crossing_flag = CROSSING_FLAG_OFF;
            stop_decision_flag = STOP_DECISION;
            temp_crossing_stop_distance = 0;

            temp_car_direction_heading = 0;

            //for debug
            //            switch (ManeuverList.action[ManeuverList.id][0])
            //            {
            //                case TURN_LEFT:
            //                    LOG_INFO(adtf_util::cString::Format("--------------------"));
            //                    LOG_INFO(adtf_util::cString::Format("Start turn left"));
            //                    break;
            //                case TURN_RIGHT:
            //                    LOG_INFO(adtf_util::cString::Format("--------------------"));
            //                    LOG_INFO(adtf_util::cString::Format("Start turn right"));
            //                    break;
            //                case STRAIGHT:
            //                    LOG_INFO(adtf_util::cString::Format("--------------------"));
            //                    LOG_INFO(adtf_util::cString::Format("Start straight"));
            //                    break;
            //                default:
            //                break;
            //            }
        }
    }

    else if(ManeuverList.action[ManeuverList.current_id][0] != PARKING && ManeuverList.action[ManeuverList.current_id][0] != PULL_OUT_LEFT && ManeuverList.action[ManeuverList.current_id][0] != PULL_OUT_RIGHT && ManeuverList.action[ManeuverList.current_id][0] != MERGE_LEFT
            && crossing_flag != CROSSING_FLAG_TRAFFIC_SIGNS)
    {

        if(GetDistanceBetweenCoordinates(position_data[X], position_data[Y] , Intersection_boundary[intersection_list[ManeuverList.current_id]].center.X, Intersection_boundary[intersection_list[ManeuverList.current_id]].center.Y) < 2.0 &&
                crossing_flag != CROSSING_FLAG_INTERSECTION)
        {
            crossing_flag = CROSSING_FLAG_INTERSECTION;
            temp_road_sign_ID = GIVE_WAY;
        }


        if(crossing_flag == CROSSING_FLAG_INTERSECTION && driving_mode != CAR_STOP)
        {
            if (find_side_direction == 0 )
                relative_distance_intersection = fabs(position_data[X] - Intersection_boundary[intersection_list[ManeuverList.current_id]].left);
            else if (find_side_direction == 1)
                relative_distance_intersection = fabs(position_data[Y] - Intersection_boundary[intersection_list[ManeuverList.current_id]].bom);
            else if (find_side_direction == 2)
                relative_distance_intersection = fabs(position_data[Y] - Intersection_boundary[intersection_list[ManeuverList.current_id]].top);
            else
                relative_distance_intersection = fabs(position_data[X] - Intersection_boundary[intersection_list[ManeuverList.current_id]].right);

//            LOG_INFO(adtf_util::cString::Format("relative_distance_intersection %g  position_data[X] %g  position_data[Y] %g  side_flag %d",relative_distance_intersection, position_data[X], position_data[Y], find_side_direction));

            if (ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT)
            {
                stop_crossing = 0.6;
                ToggleLights(LEFT, tTrue);
            }
            else if(ManeuverList.action[ManeuverList.current_id][0] == TURN_RIGHT)
            {
                stop_crossing = 0.62;//-18
                ToggleLights(RIGHT, tTrue);
                //            LOG_INFO(adtf_util::cString::Format("Right Stop distance %g",(float)stop_crossing));
            }
            else
            {
                stop_crossing = 0.45;
            }
            //debug
            //        LOG_INFO(adtf_util::cString::Format("Temp_Stop %g",temp_crossing_stop_distance));
            //        LOG_INFO(adtf_util::cString::Format("Temp_marker %d",temp_road_sign_ID));

            crossing_stop_wait_counter = CrossingWaitTimeDecision(temp_road_sign_ID);


            //close the image processing
            if(relative_distance_intersection < 0.6 && T_crossing_direction != 0)
            {
                image_processing_function_switch &= ~LANE_DETECTION;
                image_processing_function_switch &= ~ADULT_DETECTION;
                image_processing_function_switch &= ~CHILD_DETECTION;
            }



            if(relative_distance_intersection <= stop_crossing && driving_mode != CAR_STOP)//-3//15//29 //stop_decision_flag = stop_decision
            {
                LOG_INFO(adtf_util::cString::Format("Stop distance %g",stop_crossing));
                driving_mode = CAR_STOP;
            }

        }
    }


    if(driving_mode == CAR_STOP && crossing_flag != CROSSING_FLAG_OFF)
    {
        //detect Obstacle in crossing
        CrossingObstacleDetection();
        //        LOG_INFO(adtf_util::cString::Format("flag[0 1 2 3]   %d %d %d %d", (int)crossing_busy_flag[0], (int)crossing_busy_flag[1], (int)crossing_busy_flag[2], (int)crossing_busy_flag[3]));

        if ((T_crossing_direction == -1 && (temp_road_sign_ID == GIVE_WAY || temp_road_sign_ID == STOP_GIVE_WAY) && crossing_busy_flag[0] == false && crossing_busy_flag[1] == false &&  crossing_busy_flag[2] == false && crossing_busy_flag[3] == false) ||
                (T_crossing_direction == -1 &&  temp_road_sign_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT && crossing_busy_flag[1] == false && crossing_busy_flag[2] == false && crossing_busy_flag[3] == false ) ||
                (T_crossing_direction == -1 &&  temp_road_sign_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.current_id][0] == STRAIGHT  && crossing_busy_flag[2] == false && crossing_busy_flag[3] == false ) ||
                (T_crossing_direction == -1 &&  temp_road_sign_ID == HAVEWAY && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT && crossing_busy_flag[1] == false  && crossing_busy_flag[3] == false) ||

                (T_crossing_direction == 0 && (temp_road_sign_ID == GIVE_WAY || temp_road_sign_ID == STOP_GIVE_WAY) && crossing_busy_flag[0] == false && crossing_busy_flag[1] == false  && crossing_busy_flag[3] == false) ||
                (T_crossing_direction == 0 &&  temp_road_sign_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT && crossing_busy_flag[1] == false  && crossing_busy_flag[3] == false) ||
                (T_crossing_direction == 0 &&  temp_road_sign_ID == HAVEWAY && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT && crossing_busy_flag[1] == false  && crossing_busy_flag[3] == false) ||

                (T_crossing_direction == 1 && (temp_road_sign_ID == GIVE_WAY || temp_road_sign_ID == STOP_GIVE_WAY) &&  crossing_busy_flag[2] == false && crossing_busy_flag[3] == false) ||
                (T_crossing_direction == 1 &&  temp_road_sign_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.current_id][0] == STRAIGHT &&  crossing_busy_flag[2] == false && crossing_busy_flag[3] == false) ||

                (T_crossing_direction == 2 && (temp_road_sign_ID == GIVE_WAY || temp_road_sign_ID == STOP_GIVE_WAY) && crossing_busy_flag[0] == false &&  crossing_busy_flag[2] == false && crossing_busy_flag[3] == false) ||
                (T_crossing_direction == 2 &&  temp_road_sign_ID == UNMARKED_INTERSECTION && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT && crossing_busy_flag[2] == false && crossing_busy_flag[3] == false) ||
                (T_crossing_direction == 2 &&  temp_road_sign_ID == HAVEWAY && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT  && crossing_busy_flag[3] == false))
        {
            if(crossing_stop_wait_counter > 0)
                crossing_stop_wait_counter--;

            if (crossing_stop_wait_counter <= 0)
            {
                //                LOG_INFO(adtf_util::cString::Format("**** Section not Occupied ****"));

                if(driving_mode != ManeuverList.action[ManeuverList.current_id][0])
                    path_reference_counter = 0;

                driving_mode = ManeuverList.action[ManeuverList.current_id][0];

                crossing_flag = CROSSING_FLAG_OFF;
                crossing_stop_wait_counter = 0;

                temp_car_direction_heading = 0;

                ToggleLights(BRAKE, tFalse);
            }


            //for debug
            //            LOG_INFO(adtf_util::cString::Format("**** crossing_stop_wait_counter %d", crossing_stop_wait_counter));
        }
        else
        {
            crossing_stop_wait_counter = CrossingWaitTimeDecision(temp_road_sign_ID);
        }

    }

    return driving_mode;
}

//XH
int cOptilm_AutonomousDriving::RoundaboutCrossingDecision(int driving_mode_flag)
{
    //          degree      0      90   -90
    float out_point[3] = {10.48,  9.5,  11.55};

    if (roundabout_in == true && driving_mode_flag == LANE_KEEPING /*&& roundabout_flag == true*/)
    {
        //        float temp_roundabout_HeadingAngle = position_data[HEADING];
        if (current_car_HeadingAngle == 0 )
            temp_crossing_stop_distance = fabs(position_data[X] - out_point[0]);
        else if (current_car_HeadingAngle == 90  )
            temp_crossing_stop_distance = fabs(position_data[Y] - out_point[1]);
        else if (current_car_HeadingAngle == -90  )
            temp_crossing_stop_distance = fabs(position_data[Y] - out_point[2]);
        else
            temp_crossing_stop_distance = 100;

        if (temp_crossing_stop_distance < 0.8 && ManeuverList.action[ManeuverList.current_id][0]==TURN_RIGHT)
            ToggleLights(RIGHT, tTrue);

        if (temp_crossing_stop_distance < 0.1)
        {
            LOG_INFO(adtf_util::cString::Format("**** Roundabout out"));

            driving_mode_flag = ManeuverList.action[ManeuverList.current_id][0];
            if (driving_mode_flag == TURN_RIGHT)
                roundabout_in = false;
            temp_crossing_stop_distance = 0;

        }

    }


    return driving_mode_flag;
}


int cOptilm_AutonomousDriving::CrossingWaitTimeDecision(int road_sign_id)
{
    int wait_time_counter = 10;  //60 * 50 mS = 3 S

    if(road_sign_id == HAVEWAY && ManeuverList.action[ManeuverList.current_id][0] != TURN_LEFT)
        wait_time_counter = 0;
    else if(road_sign_id == HAVEWAY && ManeuverList.action[ManeuverList.current_id][0] == TURN_LEFT)
        wait_time_counter = 5;
    else if(road_sign_id == STOP_GIVE_WAY || road_sign_id == GIVE_WAY )
        wait_time_counter = 30; //30
    else
    {
        wait_time_counter = 10;//
    }

    return wait_time_counter;
}

int cOptilm_AutonomousDriving::RampStateDecision()
{
    int state = NO_RAMP;
    static int last_state = NO_RAMP;
    for(int index = 0; index < ramper_nummer; index++)
    {
        if (position_data[X] < ramper_region[index].right && position_data[X] > ramper_region[index].left &&
            position_data[Y] < ramper_region[index].top   && position_data[Y] > ramper_region[index].bom)
        {
            state = (index + 1);
        }
    }

    if(state == 4)
    {
        image_processing_function_switch &= ~LANE_DETECTION;
        last_state = state;
    }
    else
    {
        if(state == 1)
        {
            if(lane.detect_mode == LSEARCH || lane.detect_mode == SL_SEARCH)
            {
                lane.kurve_parameter[K] = 0;
                lane.kurve_parameter[M] = 0;
                lane.kurve_parameter[B] = 0;
            }
        }


        if(last_state == 4)
        {
            lane.kurve_parameter[K] = 0;
            lane.kurve_parameter[M] = -0.5;
            lane.kurve_parameter[B] = 0;
        }
        last_state = state;
        image_processing_function_switch |= LANE_DETECTION;
    }

    return state;
}

int cOptilm_AutonomousDriving::StartMergeLeft(int driving_mode)
{
//    if (position_data[X] > 8.5 &&
//            position_data[Y] < 1.5 && position_data[Y] > -0.5)
//    {
//        driving_mode = MERGE_LEFT;
//    }

//    if (position_data[X] > 8 &&
//            position_data[Y] < 1.5 && position_data[Y] > -0.5)
//    {
//        ToggleLights(LEFT, tTrue);
//    }

    if (position_data[X] > 11 && position_data[X] < 11.5 &&
            position_data[Y] < 9.5 && position_data[Y] > 7.5)
    {
        LOG_INFO(adtf_util::cString::Format("Merge left start position x %g, y %g",position_data[X] , position_data[Y] ));

        driving_mode = MERGE_LEFT;

    }

    if (position_data[X] < 12 &&
            position_data[Y] < 9.5 && position_data[Y] > 7.5)
    {
        ToggleLights(LEFT, tTrue);
    }

    return driving_mode;
}

int cOptilm_AutonomousDriving::S_CurveIntervalsChangeDecision(int current_intervals)
{
    int number_of_time_intervals = current_intervals;

    for(int index = 0; index < S_curve_nummer; index++)
    {
        if (position_data[X] < S_curve_region[index].right && position_data[X] > S_curve_region[index].left &&
                position_data[Y] < S_curve_region[index].top   && position_data[Y] > S_curve_region[index].bom)
        {
            if(current_car_HeadingAngle == S_curve_region[index].HeadingAngle)
            {
                number_of_time_intervals = 18;
                S_CurveDecisionflag = true;
            }
            else if(current_car_HeadingAngle != S_curve_region[index].HeadingAngle)
            {
                number_of_time_intervals = 12;
                S_CurveDecisionflag = false;
            }
            break;
        }
    }
    return number_of_time_intervals;
}

bool cOptilm_AutonomousDriving::LowSpeedRegionDecision()
{
    bool flag = false;

    for(int index = 0; index < low_speed_region_nummer; index++)
    {
        if (position_data[X] < low_speed_region[index].right && position_data[X] > low_speed_region[index].left &&
                position_data[Y] < low_speed_region[index].top   && position_data[Y] > low_speed_region[index].bom)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

tResult cOptilm_AutonomousDriving::VehicleCurrentHeadingDecision()
{
    current_car_HeadingAngle = position_data[HEADING];
    if (position_data[HEADING] >= -PI/4 && position_data[HEADING] <= PI/4 )
        current_car_HeadingAngle = 0;
    else if (position_data[HEADING] >= PI/4 && position_data[HEADING] <= 3*PI/4 )
        current_car_HeadingAngle = 90;
    else if (position_data[HEADING] >= -3*PI/4  && position_data[HEADING] <= -PI/4 )
        current_car_HeadingAngle = -90;
    else
        current_car_HeadingAngle = 180;


    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::CrossingDetectionDecision()
{
    int current_intersection = intersection_list[ManeuverList.current_id-1];
    int next_intersection = intersection_list[ManeuverList.current_id];

//    if(current_intersection == 5 && ManeuverList.action[ManeuverList.current_id-1][0] != STRAIGHT)
//        current_intersection = 4;
////            else if (current_index == 2 && ManeuverList.action[index][0] != STRAIGHT)
////                current_index = 3;
//    else if (current_intersection == 7 && ManeuverList.action[ManeuverList.current_id-1][0] != STRAIGHT)
//        current_intersection = 6;
//    else if (current_intersection == 4)
//        current_intersection = 5;
//    else if (current_intersection == 6)
//        current_intersection = 7;

    if (intersection_direction[current_intersection][next_intersection]  == 0 )
        find_side_direction = 0;
    else if (intersection_direction[current_intersection][next_intersection]  == 90)
        find_side_direction = 1;
    else if (intersection_direction[current_intersection][next_intersection]  == -90)
        find_side_direction = 2;
    else
        find_side_direction = 3;

    LOG_INFO(adtf_util::cString::Format("current_intersection % d next_intersection %d find_side_direction %d",current_intersection,next_intersection, find_side_direction));


    RETURN_NOERROR;
}


int cOptilm_AutonomousDriving::EmergencyVehicleProcess(int driving_mode)
{

    for(int index = 0; index < emergency_vehicle_nummer; index++)
    {
        if (position_data[X] < emergency_vehicle_boundary[index].right && position_data[X] > emergency_vehicle_boundary[index].left &&
            position_data[Y] < emergency_vehicle_boundary[index].top   && position_data[Y] > emergency_vehicle_boundary[index].bom &&
            fabs((position_data[HEADING] * RADIAN_TO_DEGREES) - emergency_vehicle_boundary[index].HeadingAngle) < 15)
        {
            in_emergency_vehicle_boundary_flag = true;
            break;
        }
        else
            in_emergency_vehicle_boundary_flag = false;
    }

    //0,0 is car center   cm
    emergency_car_detect_region.left  = -60;  //-60
    emergency_car_detect_region.right = 25;   //20
    emergency_car_detect_region.top   = 180 + LASERSCANNER_CENTER_TO_CAR_FRONT;  //180
    emergency_car_detect_region.bom   = 15  + LASERSCANNER_CENTER_TO_CAR_FRONT;//117


    int emergency_car_counter = 0;
    if(in_emergency_vehicle_boundary_flag == true)
    {
        int index = 0;
        for(index = 90; index < laser_scann.number_of_scan_point; index++)
        {
            if(laser_scann.coordinate[index].X != 0 && laser_scann.coordinate[index].Y != 0)
            {
                if(laser_scann.coordinate[index].X > emergency_car_detect_region.bom  && laser_scann.coordinate[index].X < emergency_car_detect_region.top &&
                   laser_scann.coordinate[index].Y > emergency_car_detect_region.left && laser_scann.coordinate[index].Y < emergency_car_detect_region.right)
                {
                    emergency_car_counter++;
                }
            }
        }


        if(emergency_car_counter >= 3)
        {

            emergency_vehicle_counter = 100;

            if(emergency_vehicle_flag == true)
                emergency_vehicle_counter = 30;
        }
    }
    else
    {
        emergency_vehicle_shift = 0;
        emergency_vehicle_counter = 0;
        emergency_break_flag = OFF;
    }

    if(driving_mode != LANE_KEEPING && driving_mode != CAR_STOP)
    {
        emergency_vehicle_shift = 0;
        emergency_vehicle_counter = 0;
        emergency_break_flag = OFF;
    }



    if(emergency_vehicle_counter > 0)
    {
        if(emergency_vehicle_counter > 40 && driving_mode == LANE_KEEPING)
        {
            emergency_vehicle_shift = 10;
        }
        else
        {
            emergency_vehicle_shift = 10;
            emergency_vehicle_flag = true;
            emergency_break_flag = ON;

        }

        if(crossing_stop_wait_counter != 0)
            crossing_stop_wait_counter = CrossingWaitTimeDecision(GIVE_WAY);
        ToggleLights(HAZARD, tTrue);
        ToggleLights(BRAKE, tTrue);

        emergency_vehicle_counter--;
        TransmitEmergencyBreakFlag(emergency_break_flag);
    }
    else if(emergency_vehicle_counter <= 0)
    {

        emergency_break_flag = OFF;
        emergency_vehicle_shift = 0;
        emergency_vehicle_flag = false;
        ToggleLights(HAZARD, tFalse);
        TransmitEmergencyBreakFlag(emergency_break_flag);
    }


    //for debug
    //    if(object_counter[0] != 0 || object_counter[1] != 0 || object_counter[2] != 0 || object_counter[3] != 0)
    //        LOG_INFO(adtf_util::cString::Format("boundary[0 1 2 3]   %d %d %d %d", object_counter[0], object_counter[1], object_counter[2], object_counter[3]));

    return driving_mode;
}

///*
// *
// *
// *
// *
// *
// *
// *
// *
// *
// *
// *
// *
// *
// *
// * */
int cOptilm_AutonomousDriving::StateMachineFromTester(int driving_mode)
{

    ObstacleDetection();

    //    CrossingObstacleDetection();


    switch (driving_mode)
    {
    case CAR_STOP:
        driving_mode = CAR_STOP;
        path_reference_counter = 0;

        break;

    case EMERGENCY_BREAK:
        vehicle_target_speed = 0;


        break;

        /* Lane follow stop in two conditions:
         * Distance to a stop line smaller than 70 || Distance to a road sign smaller than 50
         * If true, than set crossing_flag = 1, than estimate the distance with car speed
         * If rest distance smaller than 50, then set CAR_STOP
        */
    case LANE_KEEPING:
        driving_mode = LANE_KEEPING;
        image_processing_function_switch |= LANE_DETECTION;

//        number_of_reference_point = 18;
        //        ramp_state_flag = RampStateDecision();
        //        driving_mode = StartMergeLeft(driving_mode);
        
        //            if((driving_mode_flag == LANE_KEEPING || driving_mode_flag == AVOIDANCE) && driving_mode_flag != PARKING)
        //                driving_mode_flag = AvoidanceProcess(driving_mode_flag);

        //            //************* traffic sign and stop line: stop **************//
        //            if(crossing_flag != CROSSING_FLAG_OFF)
        //            {
        //                //Stop Distance in Cm
        //                //            temp_crossing_stop_distance -= ((car_speed * 0.05) * 100);
        //                temp_crossing_stop_distance -= ((distance_overall - last_distance_overall_for_crossing) * 100);
        //                last_distance_overall_for_crossing = distance_overall;
        //                //   LOG_INFO(adtf_util::cString::Format("Temp_Stop %g",temp_crossing_stop_distance));
        //                if(temp_crossing_stop_distance <= 20)//20//17
        //                {
        //                    driving_mode_flag = CAR_STOP;
        //                    input_state_flag = CAR_STOP;
        //                    crossing_flag = CROSSING_FLAG_OFF;
        //                    LOG_INFO(adtf_util::cString::Format(" Crossing stop flag %d, Distance %g", crossing_flag, temp_crossing_stop_distance));


        //                    temp_crossing_stop_distance = 0;
        //                }
        //            }



        //            else if(road_marker_ID == STOP_GIVE_WAY && marker_distance < 70)
        //            {
        //                temp_crossing_stop_distance = marker_distance;
        //                crossing_flag = CROSSING_FLAG_TRAFFIC_SIGNS;
        //            }


        //        driving_mode_flag = CrossingDecision(driving_mode_flag);
        //            driving_mode_flag = ParkingProcess(driving_mode_flag);
        //***************************//



        break;

    case MERGE_LEFT:
        driving_mode = MERGE_LEFT;

        if(path_reference_counter >= (5 * number_of_reference_point))
        {
            driving_mode = CAR_STOP;
        }



        break;

        /* Turn_left begins to check conditions before end, i.e. Round > 4*N -5
         * The stop conditions are the same as lane follow
         * If not stopped, check if driving recognized from image processing.
         *      If true, switch to lane follow
         * If round >= 4*N, then stop
        */
    case TURN_LEFT:
        driving_mode = TURN_LEFT;

        if(path_reference_counter >= (3 * number_of_reference_point))
        {
            driving_mode = CAR_STOP;
        }



        break;

    case TURN_RIGHT:
        driving_mode = TURN_RIGHT;

        if(path_reference_counter >= (2 * number_of_reference_point))
        {
            driving_mode = CAR_STOP;

            LOG_INFO(adtf_util::cString::Format("stop TURN_RIGHT"));
        }


        break;

    case STRAIGHT:
        driving_mode = STRAIGHT;

        break;

    case PARKING:
        driving_mode = PARKING;

        if(path_reference_counter >= (9 * number_of_reference_point-10))
        {
            driving_mode = CAR_STOP;
        }
        break;

    case PULL_OUT_LEFT:

        driving_mode = PULL_OUT_LEFT;
        if(path_reference_counter >= (4*number_of_reference_point))
        {
            driving_mode = CAR_STOP;

            path_reference_counter = 0;

        }


        break;

    case PULL_OUT_RIGHT:
        driving_mode = PULL_OUT_RIGHT;

        if(path_reference_counter >= (3*number_of_reference_point))
        {
            driving_mode = CAR_STOP;

            //                if(light_flag[LEFT] == tTrue)
            //                    ToggleLights(LEFT, tFalse);

            path_reference_counter = 0;
        }
        break;

    case AVOIDANCE:

        //driving_mode_flag = AVOIDANCE;
        if(path_reference_counter >= (2*number_of_reference_point))
        {
            //            driving_mode_flag = CAR_STOP;
            vehicle_target_speed = 0;
            path_reference_counter = 0;

        }


        break;


    default:
        driving_mode = CAR_STOP;
        break;
    }
    // LOG_INFO(adtf_util::cString::Format("Driving Flag Input %d",driving_mode_flag));

    if(driving_mode != CAR_STOP || driving_mode != AVOIDANCE || parking_Ready_flag != PARKING_READY_FLAG_ON || ramp_state_flag != RAMP_DOWN)
        driving_mode = EmergencyBreak(driving_mode);

    return driving_mode;
}
