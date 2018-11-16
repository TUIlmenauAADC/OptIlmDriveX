#include "Optilm_AutonomousDriving.h"



tResult cOptilm_AutonomousDriving::CalculateUltrasonicWorldCoordinate(void)
{
    float radian = 0;



    US_coordinate.side_left.X = 15; //X is
    US_coordinate.side_left.Y = -US_data.tSideLeft.f32Value - 15; //Y is ->

    US_coordinate.side_right.X = 15; //X is
    US_coordinate.side_right.Y = US_data.tSideRight.f32Value + 15; //Y is ->



    radian = 210 * DEGREES_TO_RADIAN;
    US_coordinate.rear_left.X = (US_data.tRearLeft.f32Value * sin(radian)) - 30; //X is
    US_coordinate.rear_left.Y = (US_data.tRearLeft.f32Value * cos(radian)) - 10; //Y is ->

    US_coordinate.rear_center.X = -US_data.tRearCenter.f32Value - 30; //X is
    US_coordinate.rear_center.Y = 0; //Y is ->

    radian = 330 * DEGREES_TO_RADIAN;
    US_coordinate.rear_right.X = (US_data.tRearRight.f32Value * sin(radian)) - 30; //X is
    US_coordinate.rear_right.Y = (US_data.tRearRight.f32Value * cos(radian)) + 10; //Y is ->



    //    LOG_INFO(adtf_util::cString::Format("Side  L to R: %.0f , %.0f; %.0f , %.0f;", ult_world_coord[S_LEFT][X], ult_world_coord[S_LEFT][Y],ult_world_coord[S_RIGHT][X], ult_world_coord[S_RIGHT][Y]));

    //    LOG_INFO(adtf_util::cString::Format("Rear  L to R: %.0f , %.0f; %.0f , %.0f; %.0f , %.0f;", ult_world_coord[R_LEFT][X],   ult_world_coord[R_LEFT][Y],
    //                                                                                                ult_world_coord[R_CENTER][X], ult_world_coord[R_CENTER][Y],
    //                                                                                                ult_world_coord[R_RIGHT][X],  ult_world_coord[R_RIGHT][Y]));


    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::CalculateTrackingPoint(float number_of_interval)
{

    //Draw Tracking tResultPoint
    int index = 0;

    float speed = 0.5;
    float DT =  0.100;
    float lane_ref_distance = 0;

    float lane_offset = 0;


    lane.kurve_parameter[B] = lane.kurve_parameter[B] + emergency_vehicle_shift;

    //    if(lane.detect_mode == LTRACE || lane.detect_mode == SL_TRACE)
    {
        lane_ref_distance =  ((speed * 100) * DT);   // m/s -> cm/s * DT

        //                lane_offset = lane_offset_coff_a * speed + lane_offset_coff_b;


        //                if(lane_offset > CAMERA_DISTANCE_MAX-1)
        //                    lane_offset = CAMERA_DISTANCE_MAX-1;
        //                else if(lane_offset < CAMERA_DISTANCE_MIN)
        //                    lane_offset = CAMERA_DISTANCE_MIN;
        lane_offset = CAMERA_DISTANCE;
        if (lane.kurve_parameter[K]==0 && lane.detect_mode == SL_TRACE)
        {
            lane_offset = 25;//22
        }

        //        LOG_INFO(adtf_util::cString::Format("------Lane offset: %f", lane_offset));
        //        lane_offset = CAMERA_DISTANCE;

        for(index = 0; index <= number_of_interval; index++)
        {
            float s = /*0 + */index*lane_ref_distance;
            float k1 = 0;
            float k2 = 0;
            float k3 = 0;
            float k4 = 0;
            float t = lane_offset;//CAMERA_DISTANCE;
            float th = s/5;
            for(int ind = 0;ind <= 5; ind++)
            {
                k1 = th/vs(t,lane.kurve_parameter[K], lane.kurve_parameter[M]);
                k2 = th/vs(t+k1/2,lane.kurve_parameter[K], lane.kurve_parameter[M]);
                k3 = th/vs(t+k2/2,lane.kurve_parameter[K], lane.kurve_parameter[M]);
                k4 = th/vs(t+k3,lane.kurve_parameter[K], lane.kurve_parameter[M]);


                t = t + (k1+2*(k2+k3)+k4)/6;
            }

            {
                path_reference_point.X[index] = t*0.01;
                path_reference_point.Y[index] = -((lane.kurve_parameter[K] * (t * t)) + (t* lane.kurve_parameter[M]) + lane.kurve_parameter[B])*0.01;
            }

            // for debug
            // LOG_INFO(adtf_util::cString::Format("Point%d Lane X: %f Y: %f", index, path_reference_point.X[index], path_reference_point.Y[index]));

        }

    }

    RETURN_NOERROR;
}


float cOptilm_AutonomousDriving::GetDistanceBetweenCoordinates(float x2, float y2, float x1, float y1)
{
    return fabs((sqrt(pow((x2 - x1), 2)+ pow((y2 - y1),2))));

}

///* Calculate reference point for trajectory tracking in MPC:
//Mauneuver includes: turn_left, turn_right, parking, turn_out_left, turn_out_right
//Global coordinates are used here. The origin point comes from estimated position: car_est_position.X_Position & car_est_position.Y_Position
//Reference trajectories are calculated using Bezier curve with four points （https://en.wikipedia.org/wiki/B%C3%A9zier_curve）:
//B(t) = P1*(1-t)^3 + P2*(1-t)^2*t + P3*(1-t)*t^2 + P4*t^3， where 0<=t<=1
//The steps to construct reference trajectories are stated as follows:
//Step 1: In the first round (i.e., (turn_around_reference_counter == 0) ), defined the entire reference trajectories depending on difference cases
//Step 2: In each round (roundIdx), give N points to ref_lane_world_coord.X & ref_lane_world_coord.Y
//*/
tResult cOptilm_AutonomousDriving::CalculateTurnAroundReferencePoint(char status_flag, int N, int roundIdx)
{
    int index = 0;
    int p_index =0;
    double tt = 0;
    int indexout = 0;
    float pt[8][2] = {0};
    float pt_global[8][2] = {0};
    float car_direction_heading = 0;  //only 0, PI/2, -PI/2, PI

    float tx = 0;
    float ty = 0;

    float dest_dist =0;

    //    if (roundIdx == 2)
    //    {
    //        temp_road_sign_X = ROAD_SIGN_NOT_IN_XML;
    //        temp_road_sign_Y = ROAD_SIGN_NOT_IN_XML;
    //    }

    switch (status_flag)
    {
    /////////////////////////////////////////////////////////////
    /// Turn left includes
    /// 3*N points with Bezier curve
    /// 1*N points with prolonged straight line
    case TURN_LEFT:

#ifdef AUTO_A
        pt[0][X] = 0   ; pt[0][Y]=0;
        pt[1][X] = 0.47 ; pt[1][Y]=0.28;  //pt[1][X] = 0.57 ; pt[1][Y]=0.25;
        pt[2][X] = 0.55; pt[2][Y]=0.72; //  pt[2][X] = 0.67; pt[2][Y]=0.67;
        pt[3][X] = 0.75; pt[3][Y]=1.88; // pt[3][X] = 0.67; pt[3][Y]=1.87;
        tx = 0.40; ty = 0.415; //0.28
#else
//        if (T_crossing_direction == 2 && T_crossing_current_index == 4)
//        {
//            pt[0][X] = 0   ; pt[0][Y]=0;
//            pt[1][X] = 0.5 ; pt[1][Y]=0.25;  //pt[1][X] = 0.57 ; pt[1][Y]=0.25;
//            pt[2][X] = 0.56; pt[2][Y]=0.68; // pt[2][X] = 0.67; pt[2][Y]=0.67;
//            pt[3][X] = 0.67; pt[3][Y]=1.88; //pt[3][X] = 0.67; pt[3][Y]=1.87;
//            tx = 0.40; ty = 0.415; //0.28
//        }
//        else
        {
            pt[0][X] = 0   ; pt[0][Y]=0;
            pt[1][X] = 0.47 ; pt[1][Y]=0.28;  //pt[1][X] = 0.5 ; pt[1][Y]=0.25;
            pt[2][X] = 0.55; pt[2][Y]=0.72; // pt[2][X] = 0.59; pt[2][Y]=0.68;
            pt[3][X] = 0.75; pt[3][Y]=1.88; //pt[3][X] = 0.67; pt[3][Y]=1.87;
            tx = 0.40; ty = 0.415; //0.28
        }
#endif

        // bezier curve Calculate
        if(roundIdx == 0) //turn_around_reference_counter: round counter
        {


            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);

            if (crossing_sign_found)
            {
                for(p_index = 0; p_index < 4; p_index++)
                {
                    pt_global[p_index][X] = (cos(car_direction_heading) * (pt[p_index][X]+tx)) - (sin(car_direction_heading) * (pt[p_index][Y]+ty)) + temp_road_sign_X ;
                    pt_global[p_index][Y] = (sin(car_direction_heading) * (pt[p_index][X]+tx)) + (cos(car_direction_heading) * (pt[p_index][Y]+ty)) + temp_road_sign_Y ;
                }


                for(index = 0; index < 2*N; index++)
                {
                    tt = (index+1)/(double)(2*N);

                    turn_left_ref_global_coord.X[index] = pow((1-tt), 2)*position_data[X] + 2*(1-tt)*tt*pt_global[1][X] + pow(tt, 2)*pt_global[2][X];
                    turn_left_ref_global_coord.Y[index] = pow((1-tt), 2)*position_data[Y] + 2*(1-tt)*tt*pt_global[1][Y] + pow(tt, 2)*pt_global[2][Y];
//                    LOG_INFO(adtf_util::cString::Format("Coordinate 2 %d  X=%f  Y=%f ",index, turn_left_ref_global_coord.X[index], turn_left_ref_global_coord.Y[index]));

                }

                for(index = 2*N; index < 4*N; index++)
                {
                    tt = (index+1-2*N)/(double)(2*N);

                    turn_left_ref_global_coord.X[index] = (1-tt)*pt_global[2][X] + tt*pt_global[3][X];
                    turn_left_ref_global_coord.Y[index] = (1-tt)*pt_global[2][Y] + tt*pt_global[3][Y];
//                    LOG_INFO(adtf_util::cString::Format("Coordinate 2 %d  X=%f  Y=%f ",index, turn_left_ref_global_coord.X[index], turn_left_ref_global_coord.Y[index]));

                }
                LOG_INFO("st1");

            }
            else
            {
                pt[0][X] = 0   ; pt[0][Y]=0;
                pt[1][X] = 1.0 ; pt[1][Y]=0;  //x: 0.8
                pt[2][X] = 1.3; pt[2][Y]=0.75; // x: 1.1
                pt[3][X] = 1.3; pt[3][Y]=1.8;


                for(index = 0; index < 4*N; index++)
                {
                    tt = (index+1)/(double)(4*N);

                    turn_left_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                    turn_left_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

                    turn_left_ref_global_coord.X[index] = (cos(car_direction_heading) * turn_left_ref_coord.X[index]) - (sin(car_direction_heading) * turn_left_ref_coord.Y[index]) + position_data[X];
                    turn_left_ref_global_coord.Y[index] = (sin(car_direction_heading) * turn_left_ref_coord.X[index]) + (cos(car_direction_heading) * turn_left_ref_coord.Y[index]) + position_data[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    LOG_INFO(adtf_util::cString::Format("Coordinate 2 %d  X=%f  Y=%f ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
#endif


                }
                LOG_INFO("st2");
            }



        }

        // write reference point
        for(index = 0; index < N; index++)
        {

            indexout = roundIdx+index ;

            path_reference_point.X[index] = turn_left_ref_global_coord.X[indexout];
            path_reference_point.Y[index] = turn_left_ref_global_coord.Y[indexout];

        }
        //            for(int i =0; i< N; i++)
        //                LOG_INFO(adtf_util::cString::Format("Ref %d X Y: %g   %g",i, path_reference_point.X[i], path_reference_point.Y[i]));
        //            LOG_INFO(adtf_util::cString::Format("----------------------------------"));
        break;

        /////////////////////////////////////////////////////////////
        /// Turn right includes
        /// N points with Bezier curve
        /// N points with prolonged straight line
    case TURN_RIGHT:

        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);


            //new
            if (crossing_sign_found)
            {

#ifdef AUTO_A
                //        LOG_INFO(adtf_util::cString::Format("T_crossing_direction %d T_crossing_current_index %d",T_crossing_direction, T_crossing_current_index));

                //        if (roundabout_in == false && T_crossing_direction == 2 && (T_crossing_current_index >= 1 && T_crossing_current_index <= 3))
                //        {
                //            pt[0][X] = 0   ; pt[0][Y]=0;
                //            pt[1][X] = 0.01 ; pt[1][Y]=-0.3; //0.1
                //            pt[2][X] = 0.1; pt[2][Y]=-0.6; //0.3
                //            pt[3][X] = 0.44; pt[3][Y]=-1; //1.5

                //            roundabout_in = true;
                //        }
                //        else
                        if (intersection_list[ManeuverList.current_id] == 9 && find_side_direction == 0) //(intersection_list[ManeuverList.current_id] == 3 && find_side_direction == 3)
                        {
                            pt[0][X] = 0   ; pt[0][Y]=0;
                            pt[1][X] = 0.18 ; pt[1][Y]=-0.55; //0.1
                            pt[2][X] = -0.3; pt[2][Y]=-1.2; //0.3
                            pt[3][X] = -1.3; pt[3][Y]=-0.7; //1.5
                            LOG_INFO("Special turn right");

                        }
                        else
                        {
                            pt[0][X] = 0   ; pt[0][Y]=0;
                            pt[1][X] = 0.01 ; pt[1][Y]=-0.3; //0.1
                            pt[2][X] = 0.20; pt[2][Y]=-0.44; //0.3
                            pt[3][X] = 0.19; pt[3][Y]=-1.64; //1.5
                        }
                        tx = 0.39; ty = 0.415; //0.25
        #else
        //        LOG_INFO(adtf_util::cString::Format("T_crossing_direction %d T_crossing_current_index %d",T_crossing_direction, T_crossing_current_index));

        //        if (roundabout_in == false && T_crossing_direction == 2 && (T_crossing_current_index >= 1 && T_crossing_current_index <= 3))
        //        {
        //            pt[0][X] = 0   ; pt[0][Y]=0;
        //            pt[1][X] = 0.01 ; pt[1][Y]=-0.3; //0.1
        //            pt[2][X] = 0.1; pt[2][Y]=-0.6; //0.3
        //            pt[3][X] = 0.44; pt[3][Y]=-1; //1.5

        //            roundabout_in = true;
        //        }
        //        else
                if (T_crossing_direction == 2 && T_crossing_current_index == 5)
                {
                    pt[0][X] = 0   ; pt[0][Y]=0;
                    pt[1][X] = 0.01 ; pt[1][Y]=-0.3; //0.1
                    pt[2][X] = 0.1; pt[2][Y]=-1; //0.3
                    pt[3][X] = -1.3; pt[3][Y]=-0.7; //1.5
                    LOG_INFO("Special turn right");

                }
                else
                {
                    pt[0][X] = 0   ; pt[0][Y]=0;
                    pt[1][X] = 0.01 ; pt[1][Y]=-0.3; //0.1
                    pt[2][X] = 0.20; pt[2][Y]=-0.44; //0.3
                    pt[3][X] = 0.19; pt[3][Y]=-1.64; //1.5
                }
                tx = 0.39; ty = 0.415; //0.25
#endif


                for(p_index = 0; p_index < 4; p_index++)
                {
                    pt_global[p_index][X] = (cos(car_direction_heading) * (pt[p_index][X]+tx)) - (sin(car_direction_heading) * (pt[p_index][Y]+ty)) + temp_road_sign_X ;
                    pt_global[p_index][Y] = (sin(car_direction_heading) * (pt[p_index][X]+tx)) + (cos(car_direction_heading) * (pt[p_index][Y]+ty)) + temp_road_sign_Y ;
                }


                for(index = 0; index < N; index++)
                {
                    tt = (index+1)/(double)(N);

                    turn_right_ref_global_coord.X[index] = pow((1-tt), 2)*position_data[X] + 2*(1-tt)*tt*pt_global[1][X] + pow(tt, 2)*pt_global[2][X];
                    turn_right_ref_global_coord.Y[index] = pow((1-tt), 2)*position_data[Y] + 2*(1-tt)*tt*pt_global[1][Y] + pow(tt, 2)*pt_global[2][Y];
                }

                for(index = N; index < 3*N; index++)
                {
                    tt = (index+1-N)/(double)(2*N);

                    turn_right_ref_global_coord.X[index] = (1-tt)*pt_global[2][X] + tt*pt_global[3][X];
                    turn_right_ref_global_coord.Y[index] = (1-tt)*pt_global[2][Y] + tt*pt_global[3][Y];
                }
                LOG_INFO("st1");
            }
            else
            {
                pt[0][X] = 0   ; pt[0][Y]=0;
                pt[1][X] = 0.4 ; pt[1][Y]=0;
                pt[2][X] = 0.82; pt[2][Y]=-0.5;
                pt[3][X] = 0.82; pt[3][Y]=-1.2;

                for(index = 0; index < 2*N; index++)
                {
                    tt = (index+1)/(double)(2*N);
                    turn_right_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                    turn_right_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

                    //                turn_right_ref_global_coord.X[index] = (cos(car_direction_heading) * turn_right_ref_coord.X[index]) - (sin(car_direction_heading) * turn_right_ref_coord.Y[index]) + position_data[X];
                    //                turn_right_ref_global_coord.Y[index] = (sin(car_direction_heading) * turn_right_ref_coord.X[index]) + (cos(car_direction_heading) * turn_right_ref_coord.Y[index]) + position_data[Y];

                    //#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
                    //#endif
                }
                for(index = 2*N; index < 3*N; index++)
                {
                    //turn_right_ref_coord.X[index] = 0.80+car_est_position.X_Position;
                    turn_right_ref_coord.X[index] = pt[3][X];
                    turn_right_ref_coord.Y[index] = turn_right_ref_coord.Y[index-1] - 0.01;

                }

                // rotate and shift the reference trajectory to new coordinates
                for(index = 0; index < 3*N; index++)
                {
                    turn_right_ref_global_coord.X[index] = (cos(car_direction_heading) * turn_right_ref_coord.X[index]) - (sin(car_direction_heading) * turn_right_ref_coord.Y[index]) + position_data[X];
                    turn_right_ref_global_coord.Y[index] = (sin(car_direction_heading) * turn_right_ref_coord.X[index]) + (cos(car_direction_heading) * turn_right_ref_coord.Y[index]) + position_data[Y];
                    //                            LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
                    //#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
                    //#endif
                }
                LOG_INFO("st2");
            }



        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;

            path_reference_point.X[index] = turn_right_ref_global_coord.X[indexout];  //cm to meter
            path_reference_point.Y[index] = turn_right_ref_global_coord.Y[indexout];

        }
        break;


        /////////////////////////////////////////////////////////////
        /// straight includes
        /// N points with Bezier curve
        /// N points with prolonged straight line
    case STRAIGHT:

#ifdef AUTO_A
        pt[0][X] = 0   ; pt[0][Y]=0;
        pt[1][X] = 0.3 ; pt[1][Y]=0;
        pt[2][X] = 0.65; pt[2][Y]=0;
        pt[3][X] = 2   ; pt[3][Y]=0;
#else
        pt[0][X] = 0   ; pt[0][Y]=0;
        pt[1][X] = 0.3 ; pt[1][Y]=0;
        pt[2][X] = 0.65; pt[2][Y]=0;
        pt[3][X] = 2   ; pt[3][Y]=0;
#endif

        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);

            for(index = 0; index < 5*N; index++)
            {
                tt = (index+1)/(double)(5*N);
                straight_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                straight_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];
            }


            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 5*N; index++)
            {
                straight_ref_global_coord.X[index] = (cos(car_direction_heading) * straight_ref_coord.X[index]) - (sin(car_direction_heading) * straight_ref_coord.Y[index]) + position_data[X];
                straight_ref_global_coord.Y[index] = (sin(car_direction_heading) * straight_ref_coord.X[index]) + (cos(car_direction_heading) * straight_ref_coord.Y[index]) + position_data[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, straight_ref_global_coord.X[index], straight_ref_global_coord.Y[index]));
#endif
            }
        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;

            path_reference_point.X[index] = straight_ref_global_coord.X[indexout];  //cm to meter
            path_reference_point.Y[index] = straight_ref_global_coord.Y[indexout];
        }
        break;


        /////////////////////////////////////////////////////////////
        /// Pullout left includes
        /// 3*N points with Bezier curve
        /// 2*N points with prolonged straight line
    case PULL_OUT_LEFT:


        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);
            if(crossing_sign_found == true)
            {
#ifdef AUTO_A
                pt[0][X] = 0  ; pt[0][Y]=0;
                pt[1][X] = 0.58 ; pt[1][Y]=0;  //x: 0.65
                pt[2][X] = 0.63; pt[2][Y]=0.7; // 0.82
                pt[3][X] = 0.68; pt[3][Y]=2; //0.82
                tx = -1.13; ty = 0; //-1.35
#else
                pt[0][X] = 0  ; pt[0][Y]=0;
                pt[1][X] = 0.55 ; pt[1][Y]=0;  //x: 0.65
                pt[2][X] = 0.60; pt[2][Y]=0.75; // 0.82
                pt[3][X] = 0.68; pt[3][Y]=2; //0.82
                tx = -1.13; ty = 0; //-1.18
#endif
                for(p_index = 0; p_index < 4; p_index++)
                {
                    {
                        pt_global[p_index][X] = (cos(car_direction_heading) * (pt[p_index][X]+tx)) - (sin(car_direction_heading) * (pt[p_index][Y]+ty)) + fabs(sin(car_direction_heading)) * position_data[X] + fabs(cos(car_direction_heading)) * temp_road_sign_X ;
                        pt_global[p_index][Y] = (sin(car_direction_heading) * (pt[p_index][X]+tx)) + (cos(car_direction_heading) * (pt[p_index][Y]+ty)) + fabs(cos(car_direction_heading)) * position_data[Y] + fabs(sin(car_direction_heading)) * temp_road_sign_Y;
                        //                LOG_INFO(adtf_util::cString::Format("Bezier curve point %d  X=%f  Y=%f  ",p_index, pt_global[p_index][X], pt_global[p_index][Y]));

                    }


                    for(index = 0; index < 3*N; index++)
                    {
                        tt = (index+1)/(double)(3*N);

                        pullout_left_ref_global_coord.X[index] = pow((1-tt), 2)*position_data[X] + 2*(1-tt)*tt*pt_global[1][X] + pow(tt, 2)*pt_global[2][X];
                        pullout_left_ref_global_coord.Y[index] = pow((1-tt), 2)*position_data[Y] + 2*(1-tt)*tt*pt_global[1][Y] + pow(tt, 2)*pt_global[2][Y];
                        //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, pullout_left_ref_global_coord.X[index], pullout_left_ref_global_coord.Y[index]));

                    }

                    for(index = 3*N; index < 5*N; index++)
                    {
                        tt = (index+1-3*N)/(double)(2*N);

                        pullout_left_ref_global_coord.X[index] = (1-tt)*pt_global[2][X] + tt*pt_global[3][X];
                        pullout_left_ref_global_coord.Y[index] = (1-tt)*pt_global[2][Y] + tt*pt_global[3][Y];
                        //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, pullout_left_ref_global_coord.X[index], pullout_left_ref_global_coord.Y[index]));

                    }
                }
            }
            else
            {
                pt[0][X] = 0   ; pt[0][Y]=0;
                pt[1][X] = 0.7 ; pt[1][Y]=0;
                pt[2][X] = 1.14; pt[2][Y]=0.70;
                pt[3][X] = 1.14; pt[3][Y]=2;

                for(index = 0; index < 5*N; index++)
                {
                    tt = (index+1)/(double)(5*N);
                    pullout_left_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                    pullout_left_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];
                }


                // rotate and shift the reference trajectory to new coordinates
                for(index = 0; index < 5*N; index++)
                {
                    pullout_left_ref_global_coord.X[index] = (cos(car_direction_heading) * pullout_left_ref_coord.X[index]) - (sin(car_direction_heading) * pullout_left_ref_coord.Y[index]) + position_data[X];
                    pullout_left_ref_global_coord.Y[index] = (sin(car_direction_heading) * pullout_left_ref_coord.X[index]) + (cos(car_direction_heading) * pullout_left_ref_coord.Y[index]) + position_data[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, straight_ref_global_coord.X[index], straight_ref_global_coord.Y[index]));
#endif
                }
            }

        }

        for(index = 0; index < N; index++)
        {

            indexout = roundIdx+index ;

            path_reference_point.X[index] = pullout_left_ref_global_coord.X[indexout];
            path_reference_point.Y[index] = pullout_left_ref_global_coord.Y[indexout];

            //           LOG_INFO(adtf_util::cString::Format("Ref %d X Y: %g   %g",index, path_reference_point.X[index], path_reference_point.Y[index]));


        }

        break;

        /////////////////////////////////////////////////////////////
        /// Pullout right includes
        /// N points with Bezier curve
        /// N points with prolonged straight line
    case PULL_OUT_RIGHT:



        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);

            if(crossing_sign_found == true)
            {
#ifdef AUTO_A
                pt[0][X] = 0   ; pt[0][Y]=0;
                pt[1][X] = 0.01 ; pt[1][Y]=-0.3;  //0.25
                pt[2][X] = 0.18; pt[2][Y]=-0.44; // 0.44
                pt[3][X] = 0.22; pt[3][Y]=-1.6; //0.48
                tx = -1.13; ty = 0;  //-1.4
#else
                pt[0][X] = 0   ; pt[0][Y]=0;
                pt[1][X] = 0.01 ; pt[1][Y]=-0.3;  //0.25
                pt[2][X] = 0.15; pt[2][Y]=-0.47; // 0.44
                pt[3][X] = 0.22; pt[3][Y]=-1.6; //0.48
                tx = -1.13; ty = 0;  //-1.4
#endif

                for(p_index = 0; p_index < 4; p_index++)
                {
                    pt_global[p_index][X] = (cos(car_direction_heading) * (pt[p_index][X]+tx)) - (sin(car_direction_heading) * (pt[p_index][Y]+ty)) + fabs(sin(car_direction_heading)) * position_data[X] + fabs(cos(car_direction_heading)) * temp_road_sign_X ;
                    pt_global[p_index][Y] = (sin(car_direction_heading) * (pt[p_index][X]+tx)) + (cos(car_direction_heading) * (pt[p_index][Y]+ty)) + fabs(cos(car_direction_heading)) * position_data[Y] + fabs(sin(car_direction_heading)) * temp_road_sign_Y;
                    //                LOG_INFO(adtf_util::cString::Format("Bezier curve point %d  X=%f  Y=%f  ",p_index, pt_global[p_index][X], pt_global[p_index][Y]));

                }


                for(index = 0; index < 2*N; index++)
                {
                    tt = (index+1)/(double)(2*N);

                    pullout_right_ref_global_coord.X[index] = pow((1-tt), 2)*position_data[X] + 2*(1-tt)*tt*pt_global[1][X] + pow(tt, 2)*pt_global[2][X];
                    pullout_right_ref_global_coord.Y[index] = pow((1-tt), 2)*position_data[Y] + 2*(1-tt)*tt*pt_global[1][Y] + pow(tt, 2)*pt_global[2][Y];
                    //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, pullout_right_ref_global_coord.X[index], pullout_right_ref_global_coord.Y[index]));

                }

                for(index = 2*N; index < 4*N; index++)
                {
                    tt = (index+1-2*N)/(double)(2*N);

                    pullout_right_ref_global_coord.X[index] = (1-tt)*pt_global[2][X] + tt*pt_global[3][X];
                    pullout_right_ref_global_coord.Y[index] = (1-tt)*pt_global[2][Y] + tt*pt_global[3][Y];
                    //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, pullout_right_ref_global_coord.X[index], pullout_right_ref_global_coord.Y[index]));

                }
            }
            else
            {

                pt[0][X] = 0   ; pt[0][Y]=0;
                pt[1][X] = 0.42 ; pt[1][Y]=-0.25; //pt[1][X] = 0.4 ; pt[1][Y]=0;
                pt[2][X] = 0.75; pt[2][Y]=-0.5; //pt[2][X] = 0.84; pt[2][Y]=-0.25;
                pt[3][X] = 0.75; pt[3][Y]=-1.5; //pt[3][X] = 0.84; pt[3][Y]=-1.2;

                for(index = 0; index < 2*N; index++)
                {
                    tt = (index+1)/(double)(2*N);
                    pullout_right_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                    pullout_right_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

                    //                turn_right_ref_global_coord.X[index] = (cos(car_direction_heading) * turn_right_ref_coord.X[index]) - (sin(car_direction_heading) * turn_right_ref_coord.Y[index]) + position_data[X];
                    //                turn_right_ref_global_coord.Y[index] = (sin(car_direction_heading) * turn_right_ref_coord.X[index]) + (cos(car_direction_heading) * turn_right_ref_coord.Y[index]) + position_data[Y];

                    //#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
                    //#endif
                }
                for(index = 2*N; index < 4*N; index++)
                {
                    //turn_right_ref_coord.X[index] = 0.80+car_est_position.X_Position;
                    pullout_right_ref_coord.X[index] = pt[3][X];
                    pullout_right_ref_coord.Y[index] = pullout_right_ref_coord.Y[index-1] - 0.05;

                }

                // rotate and shift the reference trajectory to new coordinates
                for(index = 0; index < 4*N; index++)
                {
                    pullout_right_ref_global_coord.X[index] = (cos(car_direction_heading) * pullout_right_ref_coord.X[index]) - (sin(car_direction_heading) * pullout_right_ref_coord.Y[index]) + position_data[X];
                    pullout_right_ref_global_coord.Y[index] = (sin(car_direction_heading) * pullout_right_ref_coord.X[index]) + (cos(car_direction_heading) * pullout_right_ref_coord.Y[index]) + position_data[Y];
                    //                            LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
                    //#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, turn_right_ref_global_coord.X[index], turn_right_ref_global_coord.Y[index]));
                    //#endif
                }
            }
        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;

            path_reference_point.X[index] = pullout_right_ref_global_coord.X[indexout];  //cm to meter
            path_reference_point.Y[index] = pullout_right_ref_global_coord.Y[indexout];

        }
        break;

        /////////////////////////////////////////////////////////////
        /// Parking includes
        /// 4*N points with Bezier curve (forward part)
        /// N points with connecting point
        /// 4*N points with Bezier curve (backward part)
        /// N points with terminate point
    case PARKING:

#ifdef AUTO_A
        pt[0][X] = 0  ; pt[0][Y]=0;
        pt[1][X] = 0.1  ; pt[1][Y]=0.1;
        pt[2][X] = 0.3; pt[2][Y]=0.2;
        pt[3][X] = 0.5; pt[3][Y]=0.3;

        //        pt[0][X] = 0  ; pt[0][Y]=0;
        //        pt[1][X] = 0.1  ; pt[1][Y]=0;
        //        pt[2][X] = 0.3; pt[2][Y]=0.2;
        //        pt[3][X] = 0.6; pt[3][Y]=0.3;

        pt[4][X] = 0    ; pt[4][Y]=0   ;
        pt[5][X] = -0.15; pt[5][Y]=-0.2;
        pt[6][X] = -0.32; pt[6][Y]=-0.6;
        pt[7][X] = -0.35; pt[7][Y]=-1.68;
#else

        pt[0][X] = 0  ; pt[0][Y]=0;
        pt[1][X] = 0.1  ; pt[1][Y]=0.1;
        pt[2][X] = 0.3; pt[2][Y]=0.2;
        pt[3][X] = 0.5; pt[3][Y]=0.3;

        //        pt[0][X] = 0  ; pt[0][Y]=0;
        //        pt[1][X] = 0.1  ; pt[1][Y]=0;
        //        pt[2][X] = 0.3; pt[2][Y]=0.2;
        //        pt[3][X] = 0.6; pt[3][Y]=0.3;

        pt[4][X] = 0    ; pt[4][Y]=0   ;
        pt[5][X] = -0.15; pt[5][Y]=-0.2;
        pt[6][X] = -0.32; pt[6][Y]=-0.6;
        pt[7][X] = -0.35; pt[7][Y]=-1.68;
#endif

        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);
            first_parking_direction_heading = car_direction_heading;
            LOG_INFO(adtf_util::cString::Format("PARKING Heading angle %g",car_direction_heading));

            for(index = 0; index < N; index++)
            {
                tt = (index+1)/(double)(N);
                parking_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[2][X];//car_est_position.X_Position;
                parking_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];//car_est_position.Y_Position;
            }

            for(index = N; index < 2*N; index++)
            {

                parking_ref_coord.X[index] = pt[3][X];//pt4[X];//+car_est_position.X_Position;
                parking_ref_coord.Y[index] = pt[3][Y];//pt4[Y];//+car_est_position.Y_Position;
                // LOG_INFO(adtf_util::cString::Format("ORiginal Coordinate 1 %d  X=%f  Y=%f  ",index, parking_ref_coord.X[index], parking_ref_coord.Y[index]));

            }


            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 2*N; index++)
            {
                //LOG_INFO(adtf_util::cString::Format("ORiginal Coordinate 2 %d  X=%f  Y=%f  ",index, parking_ref_coord.X[index], parking_ref_coord.Y[index]));
                parking_ref_gloabal_coord.X[index] = (cos(car_direction_heading) * parking_ref_coord.X[index]) - (sin(car_direction_heading) * parking_ref_coord.Y[index]) + position_data[X];
                parking_ref_gloabal_coord.Y[index] = (sin(car_direction_heading) * parking_ref_coord.X[index]) + (cos(car_direction_heading) * parking_ref_coord.Y[index]) + position_data[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, parking_ref_gloabal_coord.X[index], parking_ref_gloabal_coord.Y[index]));
#endif
            }


        }

        //Update the backward curve with current position
        if(roundIdx == 2*N)
        {
            car_direction_heading = first_parking_direction_heading;
            //            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);
                            LOG_INFO(adtf_util::cString::Format("PARKING Heading angle %g",car_direction_heading));

            for(index = 2*N; index < 4*N; index++)
            {
                tt = (index-2*N+1)/(double)(2*N);
                parking_ref_coord.X[index] = pow((1-tt), 3)*pt[4][X] + 3*pow((1-tt), 2)*tt*pt[5][X] + 3*pow(tt, 2)*(1-tt)*pt[6][X]+pow(tt, 3)*pt[7][X];//+car_est_position.X_Position;
                parking_ref_coord.Y[index] = pow((1-tt), 3)*pt[4][Y] + 3*pow((1-tt), 2)*tt*pt[5][Y] + 3*pow(tt, 2)*(1-tt)*pt[6][Y]+pow(tt, 3)*pt[7][Y];//+car_est_position.Y_Position;
            }

            for(index = 4*N; index < 6*N; index++)
            {
                tt = (index-4*N+1)/(double)(2*N);
                //                parking_ref_coord.X[index] = (1-tt)*pt[6][X] + tt*pt[7][X];
                //                parking_ref_coord.Y[index] = (1-tt)*pt[6][Y] + tt*pt_global[7][Y];

                parking_ref_coord.X[index] = pt[7][X];//+car_est_position.X_Position;
                parking_ref_coord.Y[index] = parking_ref_coord.Y[index-1]-0.045;//+car_est_position.Y_Position;

            }
            for(index = 2*N; index < 6*N; index++)
            {
                parking_ref_gloabal_coord.X[index] = (cos(car_direction_heading) * parking_ref_coord.X[index]) - (sin(car_direction_heading) * parking_ref_coord.Y[index]) + position_data[X];
                parking_ref_gloabal_coord.Y[index] = (sin(car_direction_heading) * parking_ref_coord.X[index]) + (cos(car_direction_heading) * parking_ref_coord.Y[index]) + position_data[Y];
                //                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, parking_ref_gloabal_coord.X[index], parking_ref_gloabal_coord.Y[index]));
            }

        }


        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;
            path_reference_point.X[index] = parking_ref_gloabal_coord.X[indexout];
            path_reference_point.Y[index] = parking_ref_gloabal_coord.Y[indexout];


        }
        break;


        /////////////////////////////////////////////////////////////
        /// Avoidance includes
        /// N points with Bezier curve
        /// N points with prolonged straight line
    case AVOIDANCE:

#ifdef AUTO_A
        if (avoidance.comeback_flag == 0)
        {
            pt[0][X] = 0   ; pt[0][Y]=0;
            pt[1][X] = 0.5 ; pt[1][Y]=0;
            pt[2][X] = 0.5 ; pt[2][Y]=0.46;
            pt[3][X] = 1.0 ; pt[3][Y]=0.46;
        }
        else if (avoidance.comeback_flag == 2)
        {
            pt[0][X] = 0   ; pt[0][Y]=0;
            pt[1][X] = 0.75; pt[1][Y]=0;
            pt[2][X] = 0.75; pt[2][Y]=-0.44;
            pt[3][X] = 1.5 ; pt[3][Y]=-0.44;
        }
#else
        if (avoidance.comeback_flag == 0)
        {
            pt[0][X] = 0   ; pt[0][Y]=0;
            pt[1][X] = 0.8 ; pt[1][Y]=0;
            pt[2][X] = 0.8 ; pt[2][Y]=0.46;
            pt[3][X] = 1.6 ; pt[3][Y]=0.46;
        }
        else if (avoidance.comeback_flag == 2)
        {
            pt[0][X] = 0    ; pt[0][Y]=0;
            pt[1][X] = 0.5 ; pt[1][Y]=-0;
            pt[2][X] = 1.2 ; pt[2][Y]=-0.44;
            pt[3][X] = 1.8    ; pt[3][Y]=-0.44;
        }
#endif



        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);


            for(index = 0; index < 2*N; index++)
            {
                tt = (index+1)/(double)(2*N);

                avoidance_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                avoidance_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, avoidance_ref_coord.X[index], avoidance_ref_coord.Y[index]));
#endif

            }
            for(index = 2*N; index < 4*N; index++)
            {
                avoidance_ref_coord.X[index] = avoidance_ref_coord.X[index-1]+0.1;
                avoidance_ref_coord.Y[index] = pt[3][Y];
            }


            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 4*N; index++)
            {
                avoidance_ref_global_coord.X[index] = (cos(car_direction_heading) * avoidance_ref_coord.X[index]) - (sin(car_direction_heading) * avoidance_ref_coord.Y[index]) + position_data[X];
                avoidance_ref_global_coord.Y[index] = (sin(car_direction_heading) * avoidance_ref_coord.X[index]) + (cos(car_direction_heading) * avoidance_ref_coord.Y[index]) + position_data[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, avoidance_ref_global_coord.X[index], avoidance_ref_global_coord.Y[index]));
#endif
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  offset=%f",index, avoidance_ref_global_coord.X[index], avoidance_ref_global_coord.Y[index],lane.kurve_parameter[B]));

            }

            if (avoidance_region_nummer==1 && avoidance.comeback_flag == 0)
            {
                pt[0][X] = 18.27   ; pt[0][Y]=3;
                pt[1][X] = 18.27 ; pt[1][Y]=3;
                pt[2][X] = 18.27 ; pt[2][Y]=5;
                pt[3][X] = 18.27 ; pt[3][Y]=5;

                for(index = 0; index < 4*N; index++)
                {
                    tt = (index+1)/(double)(4*N);

                    avoidance_ref_global_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                    avoidance_ref_global_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

    #ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, avoidance_ref_coord.X[index], avoidance_ref_coord.Y[index]));
    #endif
                    LOG_INFO(adtf_util::cString::Format("Special avoidance Coordinate%d  X=%f  Y=%f  offset=%f",index, avoidance_ref_global_coord.X[index], avoidance_ref_global_coord.Y[index],lane.kurve_parameter[B]));

                }
            }


            if (avoidance_region_nummer==1 && avoidance.comeback_flag == 2)
            {
                pt[0][X] = 18.7   ; pt[0][Y]=5;
                pt[1][X] = 18.7 ; pt[1][Y]=5;
                pt[2][X] = 18.7 ; pt[2][Y]=7;
                pt[3][X] = 18.7 ; pt[3][Y]=7;

                for(index = 0; index < 4*N; index++)
                {
                    tt = (index+1)/(double)(4*N);

                    avoidance_ref_global_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                    avoidance_ref_global_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

    #ifdef OUTPUT_BEZIER_CURVE_DEBUG
                    LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, avoidance_ref_coord.X[index], avoidance_ref_coord.Y[index]));
    #endif
                    LOG_INFO(adtf_util::cString::Format("Special avoidance Coordinate%d  X=%f  Y=%f  offset=%f",index, avoidance_ref_global_coord.X[index], avoidance_ref_global_coord.Y[index],lane.kurve_parameter[B]));

                }
            }
        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;
            path_reference_point.X[index] = avoidance_ref_global_coord.X[indexout];
            path_reference_point.Y[index] = avoidance_ref_global_coord.Y[indexout];


            //            LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, path_reference_point.X[index], path_reference_point.Y[index]));

        }

        break;

        /////////////////////////////////////////////////////////////
        /// Turn left includes
        /// 3*N points with Bezier curve
        /// 1*N points with prolonged straight line
    case MERGE_LEFT:

#ifdef AUTO_A
        pt[0][X] = 0   ; pt[0][Y]=0;
        pt[1][X] = 1 ; pt[1][Y]=0.35;
        pt[2][X] = 2 ; pt[2][Y]=0.6;
        pt[3][X] = 3 ; pt[3][Y]=1.05;
#else

        pt[0][X] = 0   ; pt[0][Y]=8.75;
        pt[1][X] = 1 ; pt[1][Y]=8.25;//0.30
        pt[2][X] = 2 ; pt[2][Y]=8;//0.55
        pt[3][X] = 3 ; pt[3][Y]=7.75;//0.95
#endif

        // bezier curve Calculate
        if(roundIdx == 0)
        {
            car_direction_heading = VehicleDirectionDecision(position_data[HEADING]);


            for(index = 0; index < 4*N; index++)
            {
                tt = (index+1)/(double)(4*N);

                merg_left_ref_coord.X[index] = pow((1-tt), 3)*pt[0][X] + 3*pow((1-tt), 2)*tt*pt[1][X] + 3*pow(tt, 2)*(1-tt)*pt[2][X]+pow(tt, 3)*pt[3][X];
                merg_left_ref_coord.Y[index] = pow((1-tt), 3)*pt[0][Y] + 3*pow((1-tt), 2)*tt*pt[1][Y] + 3*pow(tt, 2)*(1-tt)*pt[2][Y]+pow(tt, 3)*pt[3][Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, avoidance_ref_coord.X[index], avoidance_ref_coord.Y[index]));
#endif

            }
            for(index = 4*N; index < 6*N; index++)
            {
                merg_left_ref_coord.X[index] = merg_left_ref_coord.X[index-1]+0.1;
                merg_left_ref_coord.Y[index] = pt[3][Y];
            }


            // rotate and shift the reference trajectory to new coordinates
            for(index = 0; index < 6*N; index++)
            {
                merg_left_ref_global_coord.X[index] = (cos(car_direction_heading) * merg_left_ref_coord.X[index]) - (sin(car_direction_heading) * merg_left_ref_coord.Y[index]) + position_data[X];
                merg_left_ref_global_coord.Y[index] =  merg_left_ref_coord.Y[index];//(sin(car_direction_heading) * merg_left_ref_coord.X[index]) + (cos(car_direction_heading) * merg_left_ref_coord.Y[index]) + position_data[Y];

#ifdef OUTPUT_BEZIER_CURVE_DEBUG
                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, merg_left_ref_global_coord.X[index], merg_left_ref_global_coord.Y[index]));
#endif
//                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, merg_left_ref_global_coord.X[index], merg_left_ref_global_coord.Y[index]));

            }

//            for(index = 4*N; index < 6*N; index++)
//            {
////                merg_left_ref_global_coord.X[index] = (cos(car_direction_heading) * merg_left_ref_coord.X[index]) - (sin(car_direction_heading) * merg_left_ref_coord.Y[index]) + position_data[X];
//                merg_left_ref_global_coord.Y[index] = 7.75;

//#ifdef OUTPUT_BEZIER_CURVE_DEBUG
//                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, merg_left_ref_global_coord.X[index], merg_left_ref_global_coord.Y[index]));
//#endif
////                LOG_INFO(adtf_util::cString::Format("Coordinate%d  X=%f  Y=%f  ",index, merg_left_ref_global_coord.X[index], merg_left_ref_global_coord.Y[index]));

//            }
        }

        // write reference point
        for(index = 0; index < N; index++)
        {
            indexout = roundIdx+index ;
            path_reference_point.X[index] = merg_left_ref_global_coord.X[indexout];
            path_reference_point.Y[index] = merg_left_ref_global_coord.Y[indexout];


            //            LOG_INFO(adtf_util::cString::Format("Coordinate 1 %d  X=%f  Y=%f  ",index, path_reference_point.X[index], path_reference_point.Y[index]));

        }
        break;


    default:
        break;
    }
    RETURN_NOERROR;
}


void cOptilm_AutonomousDriving::CalculateVehiclePath(float steering)
{
    if(steering == 0)
    {
        vehicle_path_polynomial[K] = 0;
        vehicle_path_polynomial[M] = 0;
        vehicle_path_polynomial[B] = 0;
    }
    else if(steering > 0)
    {
        vehicle_path_polynomial[K] = -steering * 0.0000795;
        vehicle_path_polynomial[M] = -steering * 0.00338;
        vehicle_path_polynomial[B] = 0;
    }
    else if(steering < 0)
    {
        vehicle_path_polynomial[K] = -steering * 0.0001662;
        vehicle_path_polynomial[M] = -steering * -0.004729;
        vehicle_path_polynomial[B] = 0;
    }


}

float cOptilm_AutonomousDriving::VehicleDirectionDecision(float car_heading)
{
    float car_direction = 0;

    if (car_heading >= -PI/4 && car_heading <= PI/4 )
        car_direction = 0;
    else if (car_heading >= PI/4 && car_heading <= 3*PI/4 )
        car_direction = PI/2;
    else if (car_heading >= -3*PI/4  && car_heading <= -PI/4 )
        car_direction = -PI/2;
    else
        car_direction = PI;

    return car_direction;
}

float cOptilm_AutonomousDriving::vs(float x, float a, float b)
{

    return  fabs((sqrt(1+ pow((2*a*x + b),2))));

}

tResult cOptilm_AutonomousDriving::CrossingObstacleDetection()
{
    //0,0 is car center   cm
    //           1
    //    0      3      2
    //          car
    crossing_Obstacle_boundary[0].left  = -110;
    crossing_Obstacle_boundary[0].right = -68;
    crossing_Obstacle_boundary[0].top   = 75 + LASERSCANNER_CENTER_TO_CAR_FRONT; //110
    crossing_Obstacle_boundary[0].bom   = 30  + LASERSCANNER_CENTER_TO_CAR_FRONT; //25

    crossing_Obstacle_boundary[1].left  = -50;  //-60
    crossing_Obstacle_boundary[1].right = 5;   //20
    crossing_Obstacle_boundary[1].top   = 150  + LASERSCANNER_CENTER_TO_CAR_FRONT;  //180
    crossing_Obstacle_boundary[1].bom   = 110  + LASERSCANNER_CENTER_TO_CAR_FRONT;//117

    crossing_Obstacle_boundary[2].left  = 22;
    crossing_Obstacle_boundary[2].right = 80;  //110
    crossing_Obstacle_boundary[2].top   = 110  + LASERSCANNER_CENTER_TO_CAR_FRONT;  //110
    crossing_Obstacle_boundary[2].bom   = 70  + LASERSCANNER_CENTER_TO_CAR_FRONT;  //25

    crossing_Obstacle_boundary[3].left  = -60;//crossing_Obstacle_boundary[0].right;
    crossing_Obstacle_boundary[3].right = 15;//crossing_Obstacle_boundary[2].left;
    crossing_Obstacle_boundary[3].top   = 90 + LASERSCANNER_CENTER_TO_CAR_FRONT;//crossing_Obstacle_boundary[1].bom;
    crossing_Obstacle_boundary[3].bom   = 30 + LASERSCANNER_CENTER_TO_CAR_FRONT;//crossing_Obstacle_boundary[2].bom;

    int object_counter[4] = {0, 0, 0, 0};
    crossing_busy_flag[0] = false;
    crossing_busy_flag[1] = false;
    crossing_busy_flag[2] = false;
    crossing_busy_flag[3] = false;


    int index = 0;
    int boundary_index = 0;
    for(index = 0; index < laser_scann.number_of_scan_point; index++)
    {
        if(laser_scann.coordinate[index].X != 0 && laser_scann.coordinate[index].Y != 0)
        {
            for(boundary_index = 0; boundary_index < 4; boundary_index++)
            {
                if(laser_scann.coordinate[index].X > crossing_Obstacle_boundary[boundary_index].bom  && laser_scann.coordinate[index].X < crossing_Obstacle_boundary[boundary_index].top &&
                        laser_scann.coordinate[index].Y > crossing_Obstacle_boundary[boundary_index].left && laser_scann.coordinate[index].Y < crossing_Obstacle_boundary[boundary_index].right)
                {
                    object_counter[boundary_index]++;
                }
            }
        }
    }

    if(object_counter[0] >= 2)
        crossing_busy_flag[0] = true;

    if(object_counter[1] >= 2)
        crossing_busy_flag[1] = true;

    if(object_counter[2] >= 3)
        crossing_busy_flag[2] = true;

    if(object_counter[3] >= 2)
        crossing_busy_flag[3] = true;

    //for debug
    //    if(object_counter[0] != 0 || object_counter[1] != 0 || object_counter[2] != 0 || object_counter[3] != 0)
    //        LOG_INFO(adtf_util::cString::Format("boundary[0 1 2 3]   %d %d %d %d", object_counter[0], object_counter[1], object_counter[2], object_counter[3]));

    RETURN_NOERROR;
}



tResult cOptilm_AutonomousDriving::SetDigitialMapRegion()
{

    T_section_boundary[0].left  = 15.5;
    T_section_boundary[0].right = 18.0;
    T_section_boundary[0].top   = 1.5;
    T_section_boundary[0].bom   = -0.5;
    T_section_boundary[0].HeadingAngle = -90.0;

    T_section_boundary[1].left  = 19.5;
    T_section_boundary[1].right = 21.5;
    T_section_boundary[1].top   = 4.5;
    T_section_boundary[1].bom   = 2.5;
    T_section_boundary[1].HeadingAngle = 0.0;

    T_section_boundary[2].left  = 19.5;
    T_section_boundary[2].right = 21.5;
    T_section_boundary[2].top   = 6.5;
    T_section_boundary[2].bom   = 4.5;
    T_section_boundary[2].HeadingAngle = 0.0;

    T_section_boundary[3].left  = 15.5;
    T_section_boundary[3].right = 18.0;
    T_section_boundary[3].top   = 6.5;
    T_section_boundary[3].bom   = 4.5;
    T_section_boundary[3].HeadingAngle = 180.0;

    T_crossing_nummer = 4;




    //    avoidance_section_boundary[0].left  = 7.48;
    //    avoidance_section_boundary[0].right = 12.53;
    //    avoidance_section_boundary[0].top   = 1.5;
    //    avoidance_section_boundary[0].bom   = -0.5;

    avoidance_section_boundary[0].left  = 5.48;
    avoidance_section_boundary[0].right = 13.53;
    avoidance_section_boundary[0].top   = 1.5;
    avoidance_section_boundary[0].bom   = -0.5;

    avoidance_section_boundary[1].left  = 7.48;
    avoidance_section_boundary[1].right = 12.53;
    avoidance_section_boundary[1].top   = 4.5;
    avoidance_section_boundary[1].bom   = 2.5;


    avoidance_nummer = 1;



    emergency_vehicle_boundary[0].left  = 7.48;
    emergency_vehicle_boundary[0].right = 12.53;
    emergency_vehicle_boundary[0].top   = 4.5;
    emergency_vehicle_boundary[0].bom   = 2.5;
    emergency_vehicle_boundary[0].HeadingAngle = 0;


    emergency_vehicle_boundary[1].left  = 15.5;
    emergency_vehicle_boundary[1].right = 18.0;
    emergency_vehicle_boundary[1].top   = 3.5;
    emergency_vehicle_boundary[1].bom   = 1.5;
    emergency_vehicle_boundary[1].HeadingAngle = 90;
    emergency_vehicle_nummer = 2;



    pedestrian_section_boundary[0].left  = 2.644;
    pedestrian_section_boundary[0].right = 4.356;
    pedestrian_section_boundary[0].top   = 1.135;
    pedestrian_section_boundary[0].bom   = -0.135;

    pedestrian_section_boundary[1].left  = 2.865;
    pedestrian_section_boundary[1].right = 4.135;
    pedestrian_section_boundary[1].top   = 11.356;
    pedestrian_section_boundary[1].bom   = 9.644;

    pedestrian_nummer = 0;



    child_section_boundary[0].left  = 4.356;
    child_section_boundary[0].right = 9.356;
    child_section_boundary[0].top   = 0.985;
    child_section_boundary[0].bom   = -0.025;

    child_section_boundary[1].left  = 13.715;
    child_section_boundary[1].right = 14.715;
    child_section_boundary[1].top   = 8.644;
    child_section_boundary[1].bom   = 4.356;

    child_section_boundary[2].left  = -0.135;
    child_section_boundary[2].right = 1.135;
    child_section_boundary[2].top   = 10.644;
    child_section_boundary[2].bom   = 2.856;

    child_section_boundary[3].left  = 8.644;
    child_section_boundary[3].right = 12.644;
    child_section_boundary[3].top   = 16.135;
    child_section_boundary[3].bom   = 14.865;

    child_nummer = 0;



    low_speed_region[0].left  = -1.0;
    low_speed_region[0].right = 15.5;
    low_speed_region[0].top   = 5.0;
    low_speed_region[0].bom   = -1.0;

    low_speed_region_nummer = 0;


    //ramp up
    ramper_region[0].left  = 2.5;
    ramper_region[0].right = 6.0;
    ramper_region[0].top   = 5.0;
    ramper_region[0].bom   = 3.5;

    //ramp in
    ramper_region[1].left  = -0.5;
    ramper_region[1].right = 2.5;
    ramper_region[1].top   = 5.0;
    ramper_region[2].bom   = 1.2;

    //ramp down
    ramper_region[2].left  = -0.5;
    ramper_region[2].right = 5;
    ramper_region[2].top   = 1.2;
    ramper_region[2].bom   = -0.5;

    //close image process
    ramper_region[3].left  = 2.0;
    ramper_region[3].right = 2.5;
    ramper_region[3].top   = 5.0;
    ramper_region[3].bom   = 3.5;

    ramper_nummer = 4;



    //S curve
//    S_curve_region[0].left  = 2;
//    S_curve_region[0].right = 3;
//    S_curve_region[0].top   = 1.2;
//    S_curve_region[0].bom   = -0.2;
//    S_curve_region[0].HeadingAngle = 180;

//    S_curve_region[1].left  = 2;
//    S_curve_region[1].right = 3;
//    S_curve_region[1].top   = 4.2;
//    S_curve_region[1].bom   = 2.8;
//    S_curve_region[1].HeadingAngle   = 180;

    S_curve_region[0].left  = 10;
    S_curve_region[0].right = 10.5;
    S_curve_region[0].top   = 2.5;
    S_curve_region[0].bom   = 0.5;
    S_curve_region[0].HeadingAngle = 0;

    S_curve_region[1].left  = 15;
    S_curve_region[1].right = 18;
    S_curve_region[1].top   = 8;
    S_curve_region[1].bom   = 7;
    S_curve_region[1].HeadingAngle   = -90;

    S_curve_nummer = 2;

    RETURN_NOERROR;

}

tResult cOptilm_AutonomousDriving::SetDigitialMapRegionTestEvent()
{
    //test event regions

    T_section_boundary[0].left  = 8.5;
    T_section_boundary[0].right = 10.5;
    T_section_boundary[0].top   = 11.5;
    T_section_boundary[0].bom   = 9.5;
    T_section_boundary[0].HeadingAngle = 0;

    T_section_boundary[1].left  = 10.5;
    T_section_boundary[1].right = 12.5;
    T_section_boundary[1].top   = 9.5;
    T_section_boundary[1].bom   = 7.5;
    T_section_boundary[1].HeadingAngle = 90;

    T_section_boundary[2].left  = 12.5;
    T_section_boundary[2].right = 14.5;
    T_section_boundary[2].top   = 11.5;
    T_section_boundary[2].bom   = 9.5;
    T_section_boundary[2].HeadingAngle = 180;

    T_section_boundary[3].left  = 2.5;
    T_section_boundary[3].right = 4.5;
    T_section_boundary[3].top   = 6.5;
    T_section_boundary[3].bom   = 4.5;
    T_section_boundary[3].HeadingAngle = -90.0;

    T_section_boundary[4].left  = 5.5;
    T_section_boundary[4].right = 7.5;
    T_section_boundary[4].top   = 14.5;
    T_section_boundary[4].bom   = 12.5;
    T_section_boundary[4].HeadingAngle = 90;

    T_section_boundary[5].left  = -0.5;
    T_section_boundary[5].right = 1.5;
    T_section_boundary[5].top   = 11.5;
    T_section_boundary[5].bom   = 9.5;
    T_section_boundary[5].HeadingAngle = 180;

    T_section_boundary[6].left  = -0.5;
    T_section_boundary[6].right = 1.5;
    T_section_boundary[6].top   = 6.5;
    T_section_boundary[6].bom   = 4.5;
    T_section_boundary[6].HeadingAngle = 180;

    T_section_boundary[7].left  = 10.5;
    T_section_boundary[7].right = 12.5;
    T_section_boundary[7].top   = 5.5;
    T_section_boundary[7].bom   = 3.5;
    T_section_boundary[7].HeadingAngle = -90;

    T_section_boundary[8].left  = 2.5;
    T_section_boundary[8].right = 4.5;
    T_section_boundary[8].top   = 14.5;
    T_section_boundary[8].bom   = 12.5;
    T_section_boundary[8].HeadingAngle = 90;

    T_section_boundary[9].left  = 14.5;
    T_section_boundary[9].right = 16.5;
    T_section_boundary[9].top   = 11.5;
    T_section_boundary[9].bom   = 9.5;
    T_section_boundary[9].HeadingAngle = 0.0;

    T_section_boundary[10].left  = 16.5;
    T_section_boundary[10].right = 18.5;
    T_section_boundary[10].top   = 2.5;
    T_section_boundary[10].bom   = 0.5;
    T_section_boundary[10].HeadingAngle = -90;


    T_crossing_nummer = 11;




    avoidance_section_boundary[0].left  = 4.5;
    avoidance_section_boundary[0].right = 9;
    avoidance_section_boundary[0].top   = 2.5;
    avoidance_section_boundary[0].bom   = 0.5;

    avoidance_section_boundary[1].left  = 16.5;
    avoidance_section_boundary[1].right = 18.5;
    avoidance_section_boundary[1].top   = 11;
    avoidance_section_boundary[1].bom   = 5;

    avoidance_section_boundary[2].left  = 20;
    avoidance_section_boundary[2].right = 25;
    avoidance_section_boundary[2].top   = 14.5;
    avoidance_section_boundary[2].bom   = 12.5;


    avoidance_nummer = 3;



    pedestrian_section_boundary[0].left  = -0.5;
    pedestrian_section_boundary[0].right = 1.5;
    pedestrian_section_boundary[0].top   = 9.0;
    pedestrian_section_boundary[0].bom   = 6.0;

    pedestrian_section_boundary[1].left  = 7.0;
    pedestrian_section_boundary[1].right = 10.0;
    pedestrian_section_boundary[1].top   = 14.5;
    pedestrian_section_boundary[1].bom   = 12.5;

    pedestrian_nummer = 2;

    emergency_vehicle_boundary[0].left  = 6.0;
    emergency_vehicle_boundary[0].right = 11.0;
    emergency_vehicle_boundary[0].top   = 10.5;
    emergency_vehicle_boundary[0].bom   = 12.5;
    emergency_vehicle_boundary[0].HeadingAngle = 0;


//    emergency_vehicle_boundary[1].left  = 15.5;
//    emergency_vehicle_boundary[1].right = 18.0;
//    emergency_vehicle_boundary[1].top   = 3.5;
//    emergency_vehicle_boundary[1].bom   = 1.5;
//    emergency_vehicle_boundary[1].HeadingAngle = 90;
    emergency_vehicle_nummer = 1;




//    child_section_boundary[0].left  = 4.356;
//    child_section_boundary[0].right = 9.356;
//    child_section_boundary[0].top   = 0.985;
//    child_section_boundary[0].bom   = -0.025;

//    child_section_boundary[1].left  = 13.715;
//    child_section_boundary[1].right = 14.715;
//    child_section_boundary[1].top   = 8.644;
//    child_section_boundary[1].bom   = 4.356;

//    child_section_boundary[2].left  = -0.135;
//    child_section_boundary[2].right = 1.135;
//    child_section_boundary[2].top   = 10.644;
//    child_section_boundary[2].bom   = 2.856;

//    child_section_boundary[3].left  = 8.644;
//    child_section_boundary[3].right = 12.644;
//    child_section_boundary[3].top   = 16.135;
//    child_section_boundary[3].bom   = 14.865;

//    child_nummer = 0;



    low_speed_region[0].left  = -0.5;
    low_speed_region[0].right = 10.5;
    low_speed_region[0].top   = 14.5;
    low_speed_region[0].bom   = 4.5;

    low_speed_region_nummer = 1;


    //ramp up
    ramper_region[0].left  = 5.5;
    ramper_region[0].right = 9.5;
    ramper_region[0].top   = 5.5;
    ramper_region[0].bom   = 3.5;

    //ramp in
    ramper_region[1].left  = 2.5;
    ramper_region[1].right = 5.5;
    ramper_region[1].top   = 5.5;
    ramper_region[2].bom   = 1.5;

    //ramp down
    ramper_region[2].left  = 2.5;
    ramper_region[2].right = 7.5;
    ramper_region[2].top   = 1.5;
    ramper_region[2].bom   = -0.5;

    //close image process
    ramper_region[3].left  = 5.3;
    ramper_region[3].right = 5.8;
    ramper_region[3].top   = 5.5;
    ramper_region[3].bom   = 3.5;

    ramper_nummer = 4;



    //S curve
    S_curve_region[0].left  = 18;
    S_curve_region[0].right = 19.5;
    S_curve_region[0].top   = 2.5;
    S_curve_region[0].bom   = 0.5;
    S_curve_region[0].HeadingAngle = 0;

    S_curve_region[1].left  = 26.5;
    S_curve_region[1].right = 28.5;
    S_curve_region[1].top   = 10.5;
    S_curve_region[1].bom   = 8.5;
    S_curve_region[1].HeadingAngle   = -90;

    S_curve_nummer = 2;

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::SetDigitialMapRegionFinal()
{
    //test event regions

    T_section_boundary[0].left  = 3.5;
    T_section_boundary[0].right = 5.5;
    T_section_boundary[0].top   = 10.5;
    T_section_boundary[0].bom   = 8.5;
    T_section_boundary[0].HeadingAngle = 180;

    T_section_boundary[1].left  = 3.5;
    T_section_boundary[1].right = 5.5;
    T_section_boundary[1].top   = 12.5;
    T_section_boundary[1].bom   = 10.5;
    T_section_boundary[1].HeadingAngle = 180;

    T_section_boundary[2].left  = 8.5;
    T_section_boundary[2].right = 10.5;
    T_section_boundary[2].top   = 10.5;
    T_section_boundary[2].bom   = 8.5;
    T_section_boundary[2].HeadingAngle = -90;

    T_section_boundary[3].left  = 8.5;
    T_section_boundary[3].right = 10.5;
    T_section_boundary[3].top   = 15.5;
    T_section_boundary[3].bom   = 13.5;
    T_section_boundary[3].HeadingAngle = 90.0;

    T_section_boundary[4].left  = 12.5;
    T_section_boundary[4].right = 14.5;
    T_section_boundary[4].top   = 12.5;
    T_section_boundary[4].bom   = 10.5;
    T_section_boundary[4].HeadingAngle = 0;

    T_section_boundary[5].left  = 12.5;
    T_section_boundary[5].right = 14.5;
    T_section_boundary[5].top   = 15.5;
    T_section_boundary[5].bom   = 13.5;
    T_section_boundary[5].HeadingAngle = 90;

    T_section_boundary[6].left  = 6.5;
    T_section_boundary[6].right = 8.5;
    T_section_boundary[6].top   = 5.5;
    T_section_boundary[6].bom   = 3.5;
    T_section_boundary[6].HeadingAngle = 90;

    T_section_boundary[7].left  = 17.5;
    T_section_boundary[7].right = 19.5;
    T_section_boundary[7].top   = 8.5;
    T_section_boundary[7].bom   = 6.5;
    T_section_boundary[7].HeadingAngle = 0;


    T_crossing_nummer = 8;

    avoidance_section_boundary[0].left  = 17.5;
    avoidance_section_boundary[0].right = 19.5;
    avoidance_section_boundary[0].top   = 7.0;
    avoidance_section_boundary[0].bom   = 3.0;


    avoidance_section_boundary[1].left  = 11.0;
    avoidance_section_boundary[1].right = 15.0;
    avoidance_section_boundary[1].top   = 8.5;
    avoidance_section_boundary[1].bom   = 6.5;
    avoidance_section_boundary[1].HeadingAngle = 180;


    avoidance_nummer = 2;



    pedestrian_section_boundary[0].left  = 5;
    pedestrian_section_boundary[0].right = 8;
    pedestrian_section_boundary[0].top   = 10.5;
    pedestrian_section_boundary[0].bom   = 8.5;

    pedestrian_section_boundary[1].left  = 11.0;
    pedestrian_section_boundary[1].right = 13.0;
    pedestrian_section_boundary[1].top   = 1.5;
    pedestrian_section_boundary[1].bom   = -0.5;

    pedestrian_nummer = 2;



    child_section_boundary[0].left  = 15.0;
    child_section_boundary[0].right = 17.5;
    child_section_boundary[0].top   = 8.5;
    child_section_boundary[0].bom   = 6.5;

//    child_section_boundary[1].left  = 13.715;
//    child_section_boundary[1].right = 14.715;
//    child_section_boundary[1].top   = 8.644;
//    child_section_boundary[1].bom   = 4.356;

//    child_section_boundary[2].left  = -0.135;
//    child_section_boundary[2].right = 1.135;
//    child_section_boundary[2].top   = 10.644;
//    child_section_boundary[2].bom   = 2.856;

//    child_section_boundary[3].left  = 8.644;
//    child_section_boundary[3].right = 12.644;
//    child_section_boundary[3].top   = 16.135;
//    child_section_boundary[3].bom   = 14.865;

    child_nummer = 1;

    emergency_vehicle_boundary[0].left  = 6.0;
    emergency_vehicle_boundary[0].right = 11.0;
    emergency_vehicle_boundary[0].top   = 12.5;
    emergency_vehicle_boundary[0].bom   = 10.5;
    emergency_vehicle_boundary[0].HeadingAngle = 0;


//    emergency_vehicle_boundary[1].left  = 15.5;
//    emergency_vehicle_boundary[1].right = 18.0;
//    emergency_vehicle_boundary[1].top   = 3.5;
//    emergency_vehicle_boundary[1].bom   = 1.5;
//    emergency_vehicle_boundary[1].HeadingAngle = 90;
    emergency_vehicle_nummer = 1;



    low_speed_region[0].left  = 3.5;
    low_speed_region[0].right = 14.5;
    low_speed_region[0].top   = 15.5;
    low_speed_region[0].bom   = 8.5;

    low_speed_region_nummer = 1;


    //ramp up
    ramper_region[0].left  = 11.5;
    ramper_region[0].right = 14;
    ramper_region[0].top   = 5.5;
    ramper_region[0].bom   = 3.5;

    //ramp in
    ramper_region[1].left  = 14;
    ramper_region[1].right = 17.5;
    ramper_region[1].top   = 8.0;
    ramper_region[2].bom   = 3.5;

    //ramp down
    ramper_region[2].left  = 12.5;
    ramper_region[2].right = 17.5;
    ramper_region[2].top   = 9.5;
    ramper_region[2].bom   = 8.0;

    //close image process
    ramper_region[3].left  = 14;
    ramper_region[3].right = 14.3;
    ramper_region[3].top   = 5.5;
    ramper_region[3].bom   = 3.5;

    ramper_nummer = 4;



    //S curve
//    S_curve_region[0].left  = 18;
//    S_curve_region[0].right = 19.5;
//    S_curve_region[0].top   = 2.5;
//    S_curve_region[0].bom   = 0.5;
//    S_curve_region[0].HeadingAngle = 0;

//    S_curve_region[1].left  = 26.5;
//    S_curve_region[1].right = 28.5;
//    S_curve_region[1].top   = 10.5;
//    S_curve_region[1].bom   = 8.5;
//    S_curve_region[1].HeadingAngle   = -90;

//    S_curve_nummer = 2;

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::SetIntersectionGraph()
{
    //test event regions
    int i,j= 0;

    for(i = 0; i < 6; i++)
    {
        for(j = 0; j < 6; j++)
        {
            intersection_direction[i][j] = 0;
            intersection_adjacency_matrix[i][j] = 0;
        }

    }
    // 1 - 2, 1 - 6
    intersection_direction[0][1] = 0;
    intersection_direction[0][5] = 0;
    intersection_adjacency_matrix[0][1] = 1;
    intersection_adjacency_matrix[0][5] = 1;

    //2 - 1, 2 - 3, 2 - 4, 2 - 5, 2 - 6
    intersection_direction[1][0] = 180;
    intersection_direction[1][2] = -90;
    intersection_direction[1][3] = 0;
    intersection_direction[1][4] = -90;
    intersection_direction[1][5] = 0;
    intersection_adjacency_matrix[1][0] = 1;
    intersection_adjacency_matrix[1][2] = 1;
    intersection_adjacency_matrix[1][3] = 1;
    intersection_adjacency_matrix[1][4] = 1;
    intersection_adjacency_matrix[1][5] = 1;

    //3 - 2,  3 - 6
    intersection_direction[2][1] = -90;
    intersection_direction[2][5] = 180;
    intersection_adjacency_matrix[2][1] = 1;
    intersection_adjacency_matrix[2][5] = 1;

    //4 - 2
    intersection_direction[3][1] = 180;
    intersection_adjacency_matrix[3][1] = 1;


    //5 - 2
    intersection_direction[4][1] = 90;
    intersection_adjacency_matrix[4][1] = 1;

    //6 - 1, 6 - 2, 6 - 3
    intersection_direction[5][0] = 0;
    intersection_direction[5][1] = 0;
    intersection_direction[5][2] = 90;
    intersection_adjacency_matrix[5][0] = 1;
    intersection_adjacency_matrix[5][1] = 1;
    intersection_adjacency_matrix[5][2] = 1;

    Intersection_boundary[0].left  = 8.5;
    Intersection_boundary[0].right = 10.5;
    Intersection_boundary[0].top   = 11.5;
    Intersection_boundary[0].bom   = 9.5;


    Intersection_boundary[1].left  = 16.19;
    Intersection_boundary[1].right = 17.15;
    Intersection_boundary[1].top   = 3.92;
    Intersection_boundary[1].bom   = 2.96;
    Intersection_boundary[1].center.X = 16.67;
    Intersection_boundary[1].center.Y = 3.44;

    Intersection_boundary[2].left  = 20.222;
    Intersection_boundary[2].right = 21.182;
    Intersection_boundary[2].top   = 3.92;
    Intersection_boundary[2].bom   = 2.96;
    Intersection_boundary[2].center.X = 20.702;
    Intersection_boundary[2].center.Y = 3.44;

    Intersection_boundary[3].left  = 20.222;
    Intersection_boundary[3].right = 21.182;
    Intersection_boundary[3].top   = 3.92;
    Intersection_boundary[3].bom   = 2.96;
    Intersection_boundary[3].center.X = 20.702;
    Intersection_boundary[3].center.Y = 3.44;

    Intersection_boundary[4].left  = 16.19;
    Intersection_boundary[4].right = 17.15;
    Intersection_boundary[4].top   = 0.96;
    Intersection_boundary[4].bom   = 0;
    Intersection_boundary[4].center.X = 16.67;
    Intersection_boundary[4].center.Y = 0.48;

    Intersection_boundary[5].left  = 16.19;
    Intersection_boundary[5].right = 17.15;
    Intersection_boundary[5].top   = 0.96;
    Intersection_boundary[5].bom   = 0;
    Intersection_boundary[5].center.X = 16.67;
    Intersection_boundary[5].center.Y = 0.48;

    Intersection_nummer = 6;
    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::SetIntersectionGraphAugumented()
{
    //test event regions
    int i,j= 0;

    for(i = 0; i < 8; i++)
    {
        for(j = 0; j < 8; j++)
        {
            intersection_direction[i][j] = 0;
            intersection_adjacency_matrix[i][j] = 0;
        }

    }
    // 1 - 2, 1 - 6
    intersection_direction[0][1] = 0;
    intersection_direction[0][5] = 0;
    intersection_adjacency_matrix[0][1] = 1;
    intersection_adjacency_matrix[0][5] = 1;

    //2 - 1, 2 - 3, 2 - 4, 2 - 5, 2 - 6
    intersection_direction[1][0] = 180;
    intersection_direction[1][2] = 0;
    intersection_direction[1][3] = 90;
    intersection_direction[1][4] = -90;
    intersection_direction[1][5] = 0;
    intersection_adjacency_matrix[1][0] = 1;
    intersection_adjacency_matrix[1][2] = 1;
    intersection_adjacency_matrix[1][3] = 1;
    intersection_adjacency_matrix[1][4] = 1;
    intersection_adjacency_matrix[1][5] = 1;

    //3 - 2,  3 - 6,  3 - 8
    intersection_direction[2][1] = 180;
    intersection_direction[2][5] = 180;
    intersection_direction[2][7] = 90;
    intersection_adjacency_matrix[2][1] = 1;
    intersection_adjacency_matrix[2][5] = 1;
    intersection_adjacency_matrix[2][7] = 1;

    //4 - 2, 4 - 7, 4 - 8
    intersection_direction[3][1] = -90;
    intersection_direction[3][6] = 0;
    intersection_direction[3][7] = -90;
    intersection_adjacency_matrix[3][1] = 1;
    intersection_adjacency_matrix[3][6] = 1;
    intersection_adjacency_matrix[3][7] = 1;


    //5 - 2
    intersection_direction[4][1] = 90;
    intersection_adjacency_matrix[4][1] = 1;

    //6 - 1, 6 - 2, 6 - 3
    intersection_direction[5][0] = 0;
    intersection_direction[5][1] = 0;
    intersection_direction[5][2] = 90;
    intersection_adjacency_matrix[5][0] = 1;
    intersection_adjacency_matrix[5][1] = 1;
    intersection_adjacency_matrix[5][2] = 1;

    //7 - 4
    intersection_direction[6][3] = 180;

    intersection_adjacency_matrix[6][3] = 1;


    //8 - 4, 8 - 3
    intersection_direction[7][3] = -90;
    intersection_direction[7][2] = -90;
    intersection_adjacency_matrix[7][3] = 1;
    intersection_adjacency_matrix[7][2] = 1;

    Intersection_boundary[0].left  = 8.5;
    Intersection_boundary[0].right = 10.5;
    Intersection_boundary[0].top   = 11.5;
    Intersection_boundary[0].bom   = 9.5;


    Intersection_boundary[1].left  = 16.19;
    Intersection_boundary[1].right = 17.15;
    Intersection_boundary[1].top   = 3.92;
    Intersection_boundary[1].bom   = 2.96;
    Intersection_boundary[1].center.X = 16.67;
    Intersection_boundary[1].center.Y = 3.44;

    Intersection_boundary[2].left  = 20.222;
    Intersection_boundary[2].right = 21.182;
    Intersection_boundary[2].top   = 3.92;
    Intersection_boundary[2].bom   = 2.96;
    Intersection_boundary[2].center.X = 20.702;
    Intersection_boundary[2].center.Y = 3.44;

    Intersection_boundary[3].left  = 16.19;
    Intersection_boundary[3].right = 17.15;
    Intersection_boundary[3].top   = 5.94;
    Intersection_boundary[3].bom   = 4.98;
    Intersection_boundary[3].center.X = 16.67;
    Intersection_boundary[3].center.Y = 5.46;

    Intersection_boundary[4].left  = 16.19;
    Intersection_boundary[4].right = 17.15;
    Intersection_boundary[4].top   = 0.96;
    Intersection_boundary[4].bom   = 0;
    Intersection_boundary[4].center.X = 16.67;
    Intersection_boundary[4].center.Y = 0.48;

    Intersection_boundary[5].left  = 16.19;
    Intersection_boundary[5].right = 17.15;
    Intersection_boundary[5].top   = 0.96;
    Intersection_boundary[5].bom   = 0;
    Intersection_boundary[5].center.X = 16.67;
    Intersection_boundary[5].center.Y = 0.48;

    Intersection_boundary[6].left  = 20.222;
    Intersection_boundary[6].right = 21.182;
    Intersection_boundary[6].top   = 5.94;
    Intersection_boundary[6].bom   = 4.98;
    Intersection_boundary[6].center.X = 20.702;
    Intersection_boundary[6].center.Y = 5.46;

    Intersection_boundary[7].left  = 20.222;
    Intersection_boundary[7].right = 21.182;
    Intersection_boundary[7].top   = 5.94;
    Intersection_boundary[7].bom   = 4.98;
    Intersection_boundary[7].center.X = 20.702;
    Intersection_boundary[7].center.Y = 5.46;

    Intersection_nummer = 8;
    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::SetIntersectionGraphFinal()
{
    //test event regions
    int i,j= 0;

    for(i = 0; i < 12; i++)
    {
        for(j = 0; j < 12; j++)
        {
            intersection_direction[i][j] = 0;
            intersection_adjacency_matrix[i][j] = 0;
        }

    }
    // 1 - 3
    intersection_direction[0][2] = 0;

    intersection_adjacency_matrix[0][2] = 1;

    //2 - 3, 2 - 12
    intersection_direction[1][2] = 180;
    intersection_direction[1][11] = 0;

    intersection_adjacency_matrix[1][2] = 1;
    intersection_adjacency_matrix[1][11] = 1;


    //3 - 1, 3 - 2, 3 - 4, 3 - 11, 3 - 12, 3 - 13
    intersection_direction[2][0] = 180;
    intersection_direction[2][1] = 0;
    intersection_direction[2][3] = 90;
    intersection_direction[2][10] = 0;
    intersection_direction[2][11] = 0;
    intersection_direction[2][12] = 45;

    intersection_adjacency_matrix[2][0] = 1;
    intersection_adjacency_matrix[2][1] = 1;
    intersection_adjacency_matrix[2][3] = 1;
    intersection_adjacency_matrix[2][10] = 1;
    intersection_adjacency_matrix[2][11] = 1;
    intersection_adjacency_matrix[2][12] = 1;

    //4 - 3, 4 - 5, 4 - 6
    intersection_direction[3][2] = -90;
    intersection_direction[3][4] = 90;
    intersection_direction[3][5] = 0;

    intersection_adjacency_matrix[3][2] = 1;
    intersection_adjacency_matrix[3][4] = 1;
    intersection_adjacency_matrix[3][5] = 1;


    //5 - 4, 5 - 7, 5 - 8
    intersection_direction[4][3] = -90;
    intersection_direction[4][6] = 0;
    intersection_direction[4][7] = 0;

    intersection_adjacency_matrix[4][3] = 1;
    intersection_adjacency_matrix[4][6] = 1;
    intersection_adjacency_matrix[4][7] = 1;

    //6 - 4, 6 - 7, 6 - 9
    intersection_direction[5][3] = 180;
    intersection_direction[5][6] = 90;
    intersection_direction[5][8] = 90;

    intersection_adjacency_matrix[5][3] = 1;
    intersection_adjacency_matrix[5][6] = 1;
    intersection_adjacency_matrix[5][8] = 1;

    //7 - 5, 7 - 6, 7 - 8, 7 - 9
    intersection_direction[6][4] = 180;
    intersection_direction[6][5] = -90;
    intersection_direction[6][7] = 90;
    intersection_direction[6][8] = 0;

    intersection_adjacency_matrix[6][4] = 1;
    intersection_adjacency_matrix[6][5] = 1;
    intersection_adjacency_matrix[6][7] = 1;
    intersection_adjacency_matrix[6][8] = 1;

    //8 - 5, 8 - 7, 8 - 10
    intersection_direction[7][4] = -90;
    intersection_direction[7][6] = -90;
    intersection_direction[7][9] = 0;

    intersection_adjacency_matrix[7][4] = 1;
    intersection_adjacency_matrix[7][6] = 1;
    intersection_adjacency_matrix[7][9] = 1;

    //9 - 6, 9 - 7, 9 - 10
    intersection_direction[8][5] = 180;
    intersection_direction[8][6] = 180;
    intersection_direction[8][9] = 90;

    intersection_adjacency_matrix[8][5] = 1;
    intersection_adjacency_matrix[8][6] = 1;
    intersection_adjacency_matrix[8][9] = 1;

    //10 - 8, 10 - 9, 10 - 12
    intersection_direction[9][7] = 180;
    intersection_direction[9][8] = -90;
    intersection_direction[9][11] = -90;

    intersection_adjacency_matrix[9][7] = 1;
    intersection_adjacency_matrix[9][8] = 1;
    intersection_adjacency_matrix[9][11] = 1;

    //11 - 3, 11 - 12, 11 - 13
    intersection_direction[10][2] = 90;
    intersection_direction[10][11] = 90;
    intersection_direction[10][12] = -135;

    intersection_adjacency_matrix[10][2] = 1;
    intersection_adjacency_matrix[10][11] = 1;
    intersection_adjacency_matrix[10][12] = 1;

    //12 - 2, 12 - 3, 12 - 10, 12 - 11
    intersection_direction[11][1] = 180;
    intersection_direction[11][2] = 180;
    intersection_direction[11][9] = 180;
    intersection_direction[11][10] = 90;

    intersection_adjacency_matrix[11][1] = 1;
    intersection_adjacency_matrix[11][2] = 1;
    intersection_adjacency_matrix[11][9] = 1;
    intersection_adjacency_matrix[11][10] = 1;

    //13 - 3, 13 - 11
    intersection_direction[12][2] = 180;
    intersection_direction[12][10] = 180;

    intersection_adjacency_matrix[12][2] = 1;
    intersection_adjacency_matrix[12][10] = 1;

    Intersection_boundary[0].left  = 8.5;
    Intersection_boundary[0].right = 10.5;
    Intersection_boundary[0].top   = 11.5;
    Intersection_boundary[0].bom   = 9.5;


    Intersection_boundary[1].left  = 4;
    Intersection_boundary[1].right = 5;
    Intersection_boundary[1].top   = 8;
    Intersection_boundary[1].bom   = 7;
    Intersection_boundary[1].center.X = 4.5;
    Intersection_boundary[1].center.Y = 7.5;

    Intersection_boundary[2].left  = 4;
    Intersection_boundary[2].right = 5;
    Intersection_boundary[2].top   = 8;
    Intersection_boundary[2].bom   = 7;
    Intersection_boundary[2].center.X = 4.5;
    Intersection_boundary[2].center.Y = 7.5;

    Intersection_boundary[3].left  = 4;
    Intersection_boundary[3].right = 5;
    Intersection_boundary[3].top   = 10;
    Intersection_boundary[3].bom   = 9;
    Intersection_boundary[3].center.X = 4.5;
    Intersection_boundary[3].center.Y = 9.5;

    Intersection_boundary[4].left  = 4;
    Intersection_boundary[4].right = 5;
    Intersection_boundary[4].top   = 12;
    Intersection_boundary[4].bom   = 11;
    Intersection_boundary[4].center.X = 4.5;
    Intersection_boundary[4].center.Y = 11.5;

    Intersection_boundary[5].left  = 9;
    Intersection_boundary[5].right = 10;
    Intersection_boundary[5].top   = 10;
    Intersection_boundary[5].bom   = 9;
    Intersection_boundary[5].center.X = 9.5;
    Intersection_boundary[5].center.Y = 9.5;

    Intersection_boundary[6].left  = 9;
    Intersection_boundary[6].right = 10;
    Intersection_boundary[6].top   = 12;
    Intersection_boundary[4].bom   = 11;
    Intersection_boundary[4].center.X = 9.5;
    Intersection_boundary[4].center.Y = 11.5;

    Intersection_boundary[7].left  = 9;
    Intersection_boundary[7].right = 10;
    Intersection_boundary[7].top   = 15;
    Intersection_boundary[7].bom   = 14;
    Intersection_boundary[7].center.X = 9.5;
    Intersection_boundary[7].center.Y = 14.5;

    Intersection_boundary[8].left  = 13;
    Intersection_boundary[8].right = 14;
    Intersection_boundary[8].top   = 12;
    Intersection_boundary[8].bom   = 11;
    Intersection_boundary[8].center.X = 13.5;
    Intersection_boundary[8].center.Y = 11.5;

    Intersection_boundary[9].left  = 13;
    Intersection_boundary[9].right = 14;
    Intersection_boundary[9].top   = 15;
    Intersection_boundary[9].bom   = 14;
    Intersection_boundary[9].center.X = 13.5;
    Intersection_boundary[9].center.Y = 14.5;

    Intersection_boundary[10].left  = 7;
    Intersection_boundary[10].right = 8;
    Intersection_boundary[10].top   = 5;
    Intersection_boundary[10].bom   = 4;
    Intersection_boundary[10].center.X = 7.5;
    Intersection_boundary[10].center.Y = 4.5;

    Intersection_boundary[11].left  = 18;
    Intersection_boundary[11].right = 19;
    Intersection_boundary[11].top   = 8;
    Intersection_boundary[11].bom   = 7;
    Intersection_boundary[11].center.X = 18.5;
    Intersection_boundary[11].center.Y = 7.5;

    Intersection_boundary[12].left  = 11;
    Intersection_boundary[12].right = 12;
    Intersection_boundary[12].top   = 9;
    Intersection_boundary[12].bom   = 8;
    Intersection_boundary[12].center.X = 11.5;
    Intersection_boundary[12].center.Y = 8.5;

    Intersection_nummer = 13;

    RETURN_NOERROR;
}

tResult cOptilm_AutonomousDriving::SetIntersectionGraphDemo()
{
    //test event regions
    int i,j= 0;

    for(i = 0; i < 12; i++)
    {
        for(j = 0; j < 12; j++)
        {
            intersection_direction[i][j] = 0;
            intersection_adjacency_matrix[i][j] = 0;
        }

    }
    // 1 - 3
    intersection_direction[0][2] = 0;

    intersection_adjacency_matrix[0][2] = 1;

    //2 - 3, 2 - 12
    intersection_direction[1][2] = 180;
    intersection_direction[1][11] = 0;

    intersection_adjacency_matrix[1][2] = 1;
    intersection_adjacency_matrix[1][11] = 1;


    //3 - 1, 3 - 2, 3 - 4, 3 - 11, 3 - 12, 3 - 13
    intersection_direction[2][0] = 180;
    intersection_direction[2][1] = 0;
    intersection_direction[2][3] = 90;
//    intersection_direction[2][10] = 0;
    intersection_direction[2][11] = 0;
//    intersection_direction[2][12] = 45;

    intersection_adjacency_matrix[2][0] = 1;
    intersection_adjacency_matrix[2][1] = 1;
    intersection_adjacency_matrix[2][3] = 1;
//    intersection_adjacency_matrix[2][10] = 1;
    intersection_adjacency_matrix[2][11] = 1;
//    intersection_adjacency_matrix[2][12] = 1;

    //4 - 3, 4 - 5, 4 - 6
    intersection_direction[3][2] = -90;
    intersection_direction[3][4] = 90;
    intersection_direction[3][5] = 0;

    intersection_adjacency_matrix[3][2] = 1;
    intersection_adjacency_matrix[3][4] = 1;
    intersection_adjacency_matrix[3][5] = 1;


    //5 - 4, 5 - 7, 5 - 8
    intersection_direction[4][3] = -90;
    intersection_direction[4][6] = 0;
    intersection_direction[4][7] = 0;

    intersection_adjacency_matrix[4][3] = 1;
    intersection_adjacency_matrix[4][6] = 1;
    intersection_adjacency_matrix[4][7] = 1;

    //6 - 4, 6 - 7, 6 - 10
    intersection_direction[5][3] = 180;
    intersection_direction[5][6] = 90;
//    intersection_direction[5][8] = 90;
     intersection_direction[5][9] = 90;

    intersection_adjacency_matrix[5][3] = 1;
    intersection_adjacency_matrix[5][6] = 1;
//    intersection_adjacency_matrix[5][8] = 1;
    intersection_adjacency_matrix[5][9] = 1;
    //7 - 5, 7 - 6, 7 - 8, 7 - 9
    intersection_direction[6][4] = 180;
    intersection_direction[6][5] = -90;
    intersection_direction[6][7] = 90;
//    intersection_direction[6][8] = 0;

    intersection_adjacency_matrix[6][4] = 1;
    intersection_adjacency_matrix[6][5] = 1;
    intersection_adjacency_matrix[6][7] = 1;
//    intersection_adjacency_matrix[6][8] = 1;

    //8 - 5, 8 - 7, 8 - 10
    intersection_direction[7][4] = -90;
    intersection_direction[7][6] = -90;
    intersection_direction[7][9] = 0;

    intersection_adjacency_matrix[7][4] = 1;
    intersection_adjacency_matrix[7][6] = 1;
    intersection_adjacency_matrix[7][9] = 1;

    //9 - 6, 9 - 7, 9 - 10
//    intersection_direction[8][5] = 180;
//    intersection_direction[8][6] = 180;
//    intersection_direction[8][9] = 90;

//    intersection_adjacency_matrix[8][5] = 1;
//    intersection_adjacency_matrix[8][6] = 1;
//    intersection_adjacency_matrix[8][9] = 1;

    //10 - 8, 10 - 9, 10 - 12, 10 - 6
    intersection_direction[9][5] = 180;
    intersection_direction[9][7] = 180;
//    intersection_direction[9][8] = -90;
    intersection_direction[9][11] = -90;

    intersection_adjacency_matrix[9][5] = 1;
    intersection_adjacency_matrix[9][7] = 1;
//    intersection_adjacency_matrix[9][8] = 1;
    intersection_adjacency_matrix[9][11] = 1;

    //11 - 3, 11 - 12, 11 - 13
//    intersection_direction[10][2] = 90;
//    intersection_direction[10][11] = 90;
//    intersection_direction[10][12] = -135;

//    intersection_adjacency_matrix[10][2] = 1;
//    intersection_adjacency_matrix[10][11] = 1;
//    intersection_adjacency_matrix[10][12] = 1;

    //12 - 2, 12 - 3, 12 - 10, 12 - 11
    intersection_direction[11][1] = 180;
    intersection_direction[11][2] = 180;
    intersection_direction[11][9] = 180;
//    intersection_direction[11][10] = 90;

    intersection_adjacency_matrix[11][1] = 1;
    intersection_adjacency_matrix[11][2] = 1;
    intersection_adjacency_matrix[11][9] = 1;
//    intersection_adjacency_matrix[11][10] = 1;

    //13 - 3, 13 - 11
//    intersection_direction[12][2] = 180;
//    intersection_direction[12][10] = 180;

//    intersection_adjacency_matrix[12][2] = 1;
//    intersection_adjacency_matrix[12][10] = 1;

    Intersection_boundary[0].left  = 8.5;
    Intersection_boundary[0].right = 10.5;
    Intersection_boundary[0].top   = 11.5;
    Intersection_boundary[0].bom   = 9.5;


    Intersection_boundary[1].left  = 4;
    Intersection_boundary[1].right = 5;
    Intersection_boundary[1].top   = 8;
    Intersection_boundary[1].bom   = 7;
    Intersection_boundary[1].center.X = 4.5;
    Intersection_boundary[1].center.Y = 7.5;

    Intersection_boundary[2].left  = 4;
    Intersection_boundary[2].right = 5;
    Intersection_boundary[2].top   = 8;
    Intersection_boundary[2].bom   = 7;
    Intersection_boundary[2].center.X = 4.5;
    Intersection_boundary[2].center.Y = 7.5;

    Intersection_boundary[3].left  = 4;
    Intersection_boundary[3].right = 5;
    Intersection_boundary[3].top   = 10;
    Intersection_boundary[3].bom   = 9;
    Intersection_boundary[3].center.X = 4.5;
    Intersection_boundary[3].center.Y = 9.5;

    Intersection_boundary[4].left  = 4;
    Intersection_boundary[4].right = 5;
    Intersection_boundary[4].top   = 12;
    Intersection_boundary[4].bom   = 11;
    Intersection_boundary[4].center.X = 4.5;
    Intersection_boundary[4].center.Y = 11.5;

    Intersection_boundary[5].left  = 9;
    Intersection_boundary[5].right = 10;
    Intersection_boundary[5].top   = 10;
    Intersection_boundary[5].bom   = 9;
    Intersection_boundary[5].center.X = 9.5;
    Intersection_boundary[5].center.Y = 9.5;

    Intersection_boundary[6].left  = 9;
    Intersection_boundary[6].right = 10;
    Intersection_boundary[6].top   = 12;
    Intersection_boundary[4].bom   = 11;
    Intersection_boundary[4].center.X = 9.5;
    Intersection_boundary[4].center.Y = 11.5;

    Intersection_boundary[7].left  = 9;
    Intersection_boundary[7].right = 10;
    Intersection_boundary[7].top   = 15;
    Intersection_boundary[7].bom   = 14;
    Intersection_boundary[7].center.X = 9.5;
    Intersection_boundary[7].center.Y = 14.5;

    Intersection_boundary[8].left  = 13;
    Intersection_boundary[8].right = 14;
    Intersection_boundary[8].top   = 12;
    Intersection_boundary[8].bom   = 11;
    Intersection_boundary[8].center.X = 13.5;
    Intersection_boundary[8].center.Y = 11.5;

    Intersection_boundary[9].left  = 13;
    Intersection_boundary[9].right = 14;
    Intersection_boundary[9].top   = 15;
    Intersection_boundary[9].bom   = 14;
    Intersection_boundary[9].center.X = 13.5;
    Intersection_boundary[9].center.Y = 14.5;

    Intersection_boundary[10].left  = 7;
    Intersection_boundary[10].right = 8;
    Intersection_boundary[10].top   = 5;
    Intersection_boundary[10].bom   = 4;
    Intersection_boundary[10].center.X = 7.5;
    Intersection_boundary[10].center.Y = 4.5;

    Intersection_boundary[11].left  = 18;
    Intersection_boundary[11].right = 19;
    Intersection_boundary[11].top   = 8;
    Intersection_boundary[11].bom   = 7;
    Intersection_boundary[11].center.X = 18.5;
    Intersection_boundary[11].center.Y = 7.5;

    Intersection_boundary[12].left  = 11;
    Intersection_boundary[12].right = 12;
    Intersection_boundary[12].top   = 9;
    Intersection_boundary[12].bom   = 8;
    Intersection_boundary[12].center.X = 11.5;
    Intersection_boundary[12].center.Y = 8.5;

    Intersection_nummer = 13;

    RETURN_NOERROR;
}

double cOptilm_AutonomousDriving::myNeuralNetworkFunction(const double x1[2])
{
//  double xp1[2];
//  int k;
//  double av[2];
//  static const double b[2] = { 0.099, -0.285 };

//  double d0;
//  double d1;
//  int i0;
//  static const double a[15] = { 0.13181593253348814, 0.42134026099296096,
//    -0.13147181041304948, -0.094076694952236126, -0.53256905938993637,
//    -1.1033208673535537, -0.0031186043856376508, 0.53743171712970939,
//    0.55453562011418056, 0.15360451931790139, 0.32237327639058633,
//    -0.83858832765023272, 0.43615718730344211, 0.095951393632841275,
//    -0.27439838490515422 };

//  static const double b_a[15] = { -5.7813083897340052, -4.486262629154913,
//    -3.8275310541396697, 3.1227559036706487, -2.4460311416126119,
//    1.5467191251133763, 0.82323776058131248, 0.031395445255909246,
//    -0.83563488573646716, -1.5048858803674172, 2.2616586312350138,
//    -3.011333970499551, 3.869021142941107, 4.94699581156847, 5.4254843020370664
//  };

//  static const double c_a[30] = { 5.0338955203894011, 1.0773426839267559,
//    1.2435702499545747, -1.8025240100793161, 4.0397928503641776,
//    -2.559945642357969, -5.4103076248540134, -4.6273605940690157,
//    -4.8733407783038123, -3.0725995918995856, 3.795044018947229,
//    -3.9148977367218953, 3.1381481055257106, 4.8865034843482409,
//    0.27709286703826969, -0.56294831305685167, -5.4286275230169956,
//    -5.2803173364922218, 5.0829264786166624, -3.5159785373199126,
//    -4.7854016794315148, 0.17655449048382307, 2.885360438294394,
//    -2.0325574592125779, -4.42740104457047, 3.9771579155527759,
//    3.8076791425499006, -4.4212074648686732, 1.6361402847751738,
//    -5.4117173643031125 };

//  /* MYNEURALNETWORKFUNCTION neural network simulation function. */
//  /*  */
//  /*  Generated by Neural Network Toolbox function genFunction, 06-Nov-2018 17:26:58. */
//  /*  */
//  /*  [y1] = myNeuralNetworkFunction(x1) takes these arguments: */
//  /*    x = Qx2 matrix, input #1 */
//  /*  and returns: */
//  /*    y = Qx1 matrix, output #1 */
//  /*  where Q is the number of samples. */
//  /*  ===== NEURAL NETWORK CONSTANTS ===== */
//  /*  Input 1 */
//  /*  Layer 1 */
//  /*  Layer 2 */
//  /*  Output 1 */
//  /*  ===== SIMULATION ======== */
//  /*  Dimensions */
//  /*  samples */
//  /*  Input 1 */
//  /*  ===== MODULE FUNCTIONS ======== */
//  /*  Map Minimum and Maximum Input Processing Function */
//  for (k = 0; k < 2; k++) {
//    xp1[k] = x1[k] - b[k];
//  }

//  for (k = 0; k < 2; k++) {
//    av[k] = xp1[k] * (1.15141047783535 + -0.643280396534537 * (double)k);
//  }

//  for (k = 0; k < 2; k++) {
//    xp1[k] = av[k];
//  }

//  for (k = 0; k < 2; k++) {
//    av[k] = xp1[k] + -1.0;
//  }

//  for (k = 0; k < 2; k++) {
//    xp1[k] = av[k];
//  }

//  /*  Layer 1 */
//  /*  Sigmoid Symmetric Transfer Function */
//  /*  Layer 2 */
//  /*  Output 1 */
//  /*  Map Minimum and Maximum Output Reverse-Processing Function */
//  d0 = 0.0;
//  for (k = 0; k < 15; k++) {
//    d1 = 0.0;
//    for (i0 = 0; i0 < 2; i0++) {
//      d1 += c_a[k + 15 * i0] * xp1[i0];
//    }

//    d0 += a[k] * (2.0 / (1.0 + std::exp(-2.0 * (b_a[k] + d1))) - 1.0);
//  }

//  return ((0.16293473039924483 + d0) - -1.0) / 0.285714285714286 + 3.0;

    double xp1[2];
    int k;
    double av[2];
    static const double b[2] = { 0.218, -0.113 };

    static const double b_b[2] = { 2.75482093663912, 7.96812749003984 };

    double d0;
    double d1;
    int i0;
    static const double a[10] = { -0.60767044734289954, -0.3824366236056661,
      -0.22019234906419202, 0.39697817571281635, -0.29197686907726789,
      -0.013519388508301875, 0.030149975275006798, 0.17094741385230586,
      0.36185978415346609, -0.19415681625330106 };

    static const double b_a[10] = { -4.3401646648953092, -3.3038734751949579,
      2.5006631876754928, 1.0705927480165931, 0.00424481389026447,
      -0.46017639949308758, -1.9751012610670755, -1.6691655186247154,
      -3.4782369614714974, -4.5889910071406144 };

    static const double c_a[20] = { 2.6680245126428668, 3.2785027711324637,
      -0.90715688814991657, -3.4805392128316726, 3.3955954282220002,
      -2.3569664288153973, -0.96215707146439244, -4.5403703555656421,
      -2.8701626255408326, -3.8796420619137741, 3.5978839764269765,
      -3.075490346656709, -4.2626806826353087, -2.58013440558798,
      -2.6493772333690915, 3.6219860072361265, -4.0280143158018724,
      -0.72177943902667, -3.2624806052315365, 1.6796513731590552 };

    /* WEIGHTNETWORKFUNCTION neural network simulation function. */
    /*  */
    /*  Generated by Neural Network Toolbox function genFunction, 10-Nov-2018 15:34:50. */
    /*   */
    /*  [y1] = WeightNetworkFunction(x1) takes these arguments: */
    /*    x = 2xQ matrix, input #1 */
    /*  and returns: */
    /*    y = 1xQ matrix, output #1 */
    /*  where Q is the number of samples. */
    /*  ===== NEURAL NETWORK CONSTANTS ===== */
    /*  Input 1 */
    /*  Layer 1 */
    /*  Layer 2 */
    /*  Output 1 */
    /*  ===== SIMULATION ======== */
    /*  Dimensions */
    /*  samples */
    /*  Input 1 */
    /*  ===== MODULE FUNCTIONS ======== */
    /*  Map Minimum and Maximum Input Processing Function */
    for (k = 0; k < 2; k++) {
      xp1[k] = x1[k] - b[k];
    }

    for (k = 0; k < 2; k++) {
      av[k] = xp1[k] * b_b[k];
    }

    for (k = 0; k < 2; k++) {
      xp1[k] = av[k];
    }

    for (k = 0; k < 2; k++) {
      av[k] = xp1[k] + -1.0;
    }

    for (k = 0; k < 2; k++) {
      xp1[k] = av[k];
    }

    /*  Layer 1 */
    /*  Sigmoid Symmetric Transfer Function */
    /*  Layer 2 */
    /*  Output 1 */
    /*  Map Minimum and Maximum Output Reverse-Processing Function */
    d0 = 0.0;
    for (k = 0; k < 10; k++) {
      d1 = 0.0;
      for (i0 = 0; i0 < 2; i0++) {
        d1 += c_a[k + 10 * i0] * xp1[i0];
      }

      d0 += a[k] * (2.0 / (1.0 + std::exp(-2.0 * (b_a[k] + d1))) - 1.0);
    }

    return ((-0.40934075391087416 + d0) - -1.0) / 0.285714285714286 + 3.0;
}



double cOptilm_AutonomousDriving::SpeedNetworkFunction(double x1)
{
  double cv;
  double d0;
  int k;
  static const double a[4] = { -0.57012618324868569, -0.09337423522987591,
    -0.14100345887262816, 1.5414909044744967 };

  static const double b_a[4] = { 5.8685281368238043, -1.6038844236815069,
    4.6190738600854147, 7.1636120088026729 };

  static const double c_a[4] = { -5.2070427848785465, 5.8113001660561485,
    6.5588089002909014, 4.0800936072482505 };

  /* SPEEDNETWORKFUNCTION neural network simulation function. */
  /*  */
  /*  Generated by Neural Network Toolbox function genFunction, 07-Nov-2018 16:41:37. */
  /*   */
  /*  [y1] = SpeedNetworkFunction(x1) takes these arguments: */
  /*    x = 1xQ matrix, input #1 */
  /*  and returns: */
  /*    y = 1xQ matrix, output #1 */
  /*  where Q is the number of samples. */
  /*  ===== NEURAL NETWORK CONSTANTS ===== */
  /*  Input 1 */
  /*  Layer 1 */
  /*  Layer 2 */
  /*  Output 1 */
  /*  ===== SIMULATION ======== */
  /*  Dimensions */
  /*  samples */
  /*  Input 1 */
  /*  ===== MODULE FUNCTIONS ======== */
  /*  Map Minimum and Maximum Input Processing Function */
  cv = x1 * 6.62251655629139;

  /*  Layer 1 */
  /*  Sigmoid Symmetric Transfer Function */
  /*  Layer 2 */
  /*  Output 1 */
  /*  Map Minimum and Maximum Output Reverse-Processing Function */
  d0 = 0.0;
  for (k = 0; k < 4; k++) {
    d0 += a[k] * (2.0 / (1.0 + std::exp(-2.0 * (b_a[k] + c_a[k] * (cv + -1.0))))
                  - 1.0);
  }

  return ((-1.3457968056019702 + d0) - -1.0) / 1.35777325186694 + 0.115;
}
