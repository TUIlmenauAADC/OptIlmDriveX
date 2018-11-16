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


#include "OptIlm_ModelPredictiveController.h"
#include "Nmpc_Algorithm/SOP_NMPC.h"
#include "ADTF3_helper.h"
#include "Nmpc_Algorithm/ExtendedKalmanFilter/ExtendedKalmanFilter.h"


short emergency_break_counter = 0;
bool initial_flag = false;
bool position_input_flag = false;

NMPC_STRUCT *Nmpc;
EKF_STRUCT *Ekf;


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
                                    "OptIlm_ModelPredictiveController",
                                    cOptIlm_ModelPredictiveController,
                                    //                                    adtf::filter::timer_trigger(0))
                                    adtf::filter::thread_trigger(true))


cOptIlm_ModelPredictiveController::cOptIlm_ModelPredictiveController()
{
    SetName("Model Predictive Controller");
    /*
    RegisterPropertyVariable("1.DrivingModelTest", m_Driving_Model_Test);

    RegisterPropertyVariable("2.Lane keeping: a.weight_High", weightFact_Lanekeeping_HY);
    RegisterPropertyVariable("2.Lane keeping: b.weight_low", weightFact_Lanekeeping_LY);
    RegisterPropertyVariable("2.Lane keeping: c.weight_d", weightFact_Lanekeeping_DY);
    RegisterPropertyVariable("2.Lane keeping: d.max speed", lane_keeping_maxSpeed);
    RegisterPropertyVariable("2.Lane keeping: e.min speed", lane_keeping_minSpeed);

    RegisterPropertyVariable("3.Turn left: a.weight_x", weightFact_TurnLeft_X);
    RegisterPropertyVariable("3.Turn left: b.weight_y", weightFact_TurnLeft_Y);

    RegisterPropertyVariable("4.Turn right: a.weight_x", weightFact_TurnRight_X);
    RegisterPropertyVariable("4.Turn right: b.weight_y", weightFact_TurnRight_Y);

    RegisterPropertyVariable("5.Straight: a.weight_x", weightFact_Straight_X);
    RegisterPropertyVariable("5.Straight: b.weight_y", weightFact_Straight_Y);

    RegisterPropertyVariable("6.Pull out left: a.weight_x", weightFact_PullOutLeft_X);
    RegisterPropertyVariable("6.Pull out left: b.weight_y", weightFact_PullOutLeft_Y);

    RegisterPropertyVariable("7.Pull out right: a.weight_x", weightFact_PullOutRight_X);
    RegisterPropertyVariable("7.Pull out right: b.weight_y", weightFact_PullOutRight_Y);

    RegisterPropertyVariable("8.Parking: a.weight_x", weightFact_Parking_X);
    RegisterPropertyVariable("8.Parking: b.weight_y", weightFact_Parking_Y);

    RegisterPropertyVariable("9.Avoidance: a.weight_x", weightFact_Avoidance_X);
    RegisterPropertyVariable("9.Avoidance: b.weight_y", weightFact_Avoidance_Y);

    RegisterPropertyVariable("10.Merge left: a.weight_x", weightFact_MergeLeft_X);
    RegisterPropertyVariable("10.Merge left: b.weight_y", weightFact_MergeLeft_Y);


    */

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

    Register(m_oOutputSpeed, "Speed_Control_Signal", pTypeSignalValue);
    Register(m_oOutputSteering, "Steering_Control_Signal", pTypeSignalValue);


    Register(m_oDrivingModel, "DrivingModel" , pTypeSignalValue);
    Register(m_oVehicleSpeed, "VehicleSpeed" , pTypeSignalValue);


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
        LOG_WARNING("No mediadescription for tPosition found!");
    }
    //    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;
    //Create Pin
    Register(m_oReaderPos, "position" , pTypePositionData);
    Register(m_oOutputEKFPosition, "EKFposition" , pTypePositionData);




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
    Register(m_oInput_trajectory_point, "TrajectoryPoint", pTypeTrajectoryPoint);


    m_lastSampleTime = 0;

}


//implement the Configure function to read ALL Properties
tResult cOptIlm_ModelPredictiveController::Configure()
{

    RETURN_IF_FAILED(cTriggerFunction::Configure());


    RETURN_NOERROR;
}

tResult cOptIlm_ModelPredictiveController::Process(tTimeStamp tmTimeOfTrigger)
{
    // avoid that the method triggers itself due to the feedback loop
//    std::lock_guard<std::mutex> oGuard(m_oMutex);

    tTimeStamp m_NowSampleTime =  tmTimeOfTrigger;
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;


    ///////////////////////////////////////////////
    ///get pin data
    ///
//    if(initial_flag == true)
//    {
//        GetDrivingModel();
//        GetVehicleSpeed();
//        GetTrajectoryPoint();
//        GetPosition();
//    }


    if(f64SampleTime > 0.1)
    {
        m_lastSampleTime = m_NowSampleTime;

        //Filter sample time test for debug
        //        LOG_INFO(cString::Format("ModelPredictiveController Sample Time = %.3f S", f64SampleTime));



        ///////////////////////////////////////////////
        ///get pin data
        ///
        if(initial_flag == true)
        {
            GetDrivingModel();
            GetVehicleSpeed();
            GetTrajectoryPoint();
            GetPosition();
        }



        ////////////////////////////////////////////////
        /// program initial
        ///
        initial_flag = Initial(initial_flag);




        //for debug
        //            for(int index = 0; index < 12; index++)
        //            {
        //                Nmpc->trajectory_setpoint.X[index] = index*0.01;
        //                Nmpc->trajectory_setpoint.Y[index] = -index*0.01;
        //                Nmpc->trajectory_setpoint.Theta[index] = 0;

        //                // for debug
        //        //        LOG_INFO(adtf_util::cString::Format("Trajectory Point%d X: %f Y: %f", index, Nmpc->trajectory_setpoint.X[index], Nmpc->trajectory_setpoint.Y[index]));

        //            }

//                if(position_input_flag == true)
//                {
//                    Nmpc->car_position.X_Position   = Ekf->estimates_position.X_Position;
//                    Nmpc->car_position.Y_Position   = Ekf->estimates_position.Y_Position;
//                    Nmpc->car_position.HeadingAngle = Ekf->estimates_position.HeadingAngle;
////                    LOG_INFO(cString::Format("position x:%g y:%g heading:%g ", Nmpc->car_position.X_Position, Nmpc->car_position.Y_Position, Nmpc->car_position.HeadingAngle));

//                }



        ////////////////////////////////////
        ///Run Algorithmus(NMPC)
        ///
        if(initial_flag == true)
        {
            int ipopt_status = IPOPT_DEFAULT_STATUS;
            Nmpc->ipopt_operation_time = 0;

            if(Nmpc->driving_model_flag == CAR_STOP || Nmpc->driving_model_flag == EMERGENCY_BREAK)
            {
                Nmpc->car_control.steering = 0;
                Nmpc->car_control.speed    = 0;

            }
            else
            {
                ipopt_status = CalculateMPC(Nmpc->driving_model_flag, 1.0, Nmpc);
            }
            //for Debugs
            //        LOG_INFO(cString::Format("Driving model(NMPC Filter): %d", Nmpc->driving_model_flag));
            //                    LOG_INFO(cString::Format("Ipopt Staus = %d  Time = %.3f S", ipopt_status, Nmpc->ipopt_operation_time));
            //                    LOG_INFO(cString::Format("Control Speed = %g m/s,  Steering = %g Degree", Nmpc->car_control.speed, Nmpc->car_control.steering));
            //        if (log_file) fprintf(log_file,"%.3f\n",Nmpc->ipopt_operation_time);

        }

        ////////////////////////////////////////////////
        /// Send Control Signal
        ///

//                if(position_input_flag == true)
//                {
//                    int NowEKFTime =  adtf_util::cHighResTimer::GetTime();
//                    Ekf->steering_angle = Nmpc->car_control.steering;
////                    Ekf->currents_position.speed = Nmpc->car_control.speed;


//                    Ekf->delta_time = (double)(NowEKFTime - Ekf->last_Time) * 0.000001;       // Length of the time intervals in [s]
//                    Ekf->last_Time = NowEKFTime;

////                    LOG_INFO(cString::Format("Ekf->delta_time = %.3f S", Ekf->delta_time));
//                    RunExtendedKalmanFilter(Ekf);
//                    TransmitEKFPosition();

//                }

        SendControlSignal(Nmpc->car_control.speed, (Nmpc->car_control.steering * RADIAN_TO_DEGREES));
    }

    
    RETURN_NOERROR;
}

tResult cOptIlm_ModelPredictiveController::TransmitEKFPosition()
{
    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_PositionSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.x, Ekf->estimates_position.X_Position));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.y, Ekf->estimates_position.Y_Position));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.radius, Ekf->estimates_position.radius));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.speed, Ekf->estimates_position.speed));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionIndex.heading, Ekf->estimates_position.HeadingAngle));
    }
    m_oOutputEKFPosition << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}

tResult cOptIlm_ModelPredictiveController::TransmitSpeed(tSignalValue speed_signal)
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

tResult cOptIlm_ModelPredictiveController::TransmitSteering(tSignalValue steering_signal)
{
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, steering_signal.f32Value));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, steering_signal.ui32ArduinoTimestamp));

    }
    m_oOutputSteering << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cOptIlm_ModelPredictiveController::SendControlSignal(float speed, float steering)
{
    tSignalValue signal_value;

    signal_value.f32Value = speed;
    TransmitSpeed(signal_value);




    // Do the Processing
    tFloat32 output_servo_percent = 0;

    if(steering== 0)
        output_servo_percent = 0;
    else if(steering > 0)
    {
        output_servo_percent = steering * POSITIVE_STEERING_ANGLE_TO_PERCENT;
    }
    else if(steering < 0)
    {
        output_servo_percent = steering * NEGATIVE_STEERING_ANGLE_TO_PERCENT;

    }


    signal_value.f32Value = output_servo_percent;
    signal_value.ui32ArduinoTimestamp = steering;
    TransmitSteering(signal_value);

    //for Debug
    //        LOG_INFO(cString::Format("Send Control Signal Speed = %g m/s,  Steering = %g Degree", speed, signal_value.ui32ArduinoTimestamp));
    RETURN_NOERROR;
}

tTimeStamp cOptIlm_ModelPredictiveController::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}


bool cOptIlm_ModelPredictiveController::Initial(bool flag)
{
    if(flag == false)
    {
        Nmpc = (NMPC_STRUCT*)calloc(1,sizeof(NMPC_STRUCT));
        NmpcInitial(Nmpc, flag);
        SetIpopt();
        Nmpc->driving_model_flag = CAR_STOP;
        Nmpc->car_control.speed = 0;
        Nmpc->car_control.steering = 0;
        LOG_INFO("MPC Initialed!!!");


        Ekf = (EKF_STRUCT*)calloc(1,sizeof(EKF_STRUCT));



        return true;
    }

    return flag;
}

void cOptIlm_ModelPredictiveController::FreeMem(bool flag)
{
    if(flag == true)
    {
        NmpcFree(Nmpc);
        free(Nmpc);
    }
}


///////////////////////////////////////////////////////
/// Get Pin Data
///
///
tResult cOptIlm_ModelPredictiveController::GetDrivingModel(void)
{
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oDrivingModel.GetLastSample(pReadSample)))
    {
        float model_flag = 0;
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &model_flag));

        Nmpc->driving_model_flag = (int)model_flag;
    }

    RETURN_NOERROR;
}


tResult cOptIlm_ModelPredictiveController::GetVehicleSpeed(void)
{
    object_ptr<const ISample> pSampleFromVcc;
    tSignalValue vehicle_speed;

    if (IS_OK(m_oVehicleSpeed.GetLastSample(pSampleFromVcc)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromVcc);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &vehicle_speed.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &vehicle_speed.ui32ArduinoTimestamp));
        Nmpc->vehicle_speed = vehicle_speed.f32Value;
    }

    RETURN_NOERROR;
}

tResult cOptIlm_ModelPredictiveController::GetPosition(void)
{
    object_ptr<const ISample> pReadSampleFromPosition;
    if (IS_OK(m_oReaderPos.GetLastSample(pReadSampleFromPosition)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSampleFromPosition);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &Nmpc->car_position.X_Position));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &Nmpc->car_position.Y_Position));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &Nmpc->car_position.HeadingAngle));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &Nmpc->car_position.speed));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &Nmpc->car_position.radius));

        Nmpc->car_position.X_Position   = cos(Nmpc->car_position.HeadingAngle) * 0.18 + Nmpc->car_position.X_Position;
        Nmpc->car_position.Y_Position   = sin(Nmpc->car_position.HeadingAngle) * 0.18 + Nmpc->car_position.Y_Position;
        //show position pin data for debug
        //        LOG_INFO(cString::Format("position x:%g y:%g heading:%g speed:%g radius:g ", Nmpc->car_position.X_Position, Nmpc->car_position.Y_Position, Nmpc->car_position.HeadingAngle, Nmpc->car_position.speed, Nmpc->car_position.radius));
    }

//        if(position_input_flag == false)
//        {
//            Ekf->currents_position.X_Position   = Nmpc->car_position.X_Position;
//            Ekf->currents_position.Y_Position   = Nmpc->car_position.Y_Position;
//            Ekf->currents_position.HeadingAngle = Nmpc->car_position.HeadingAngle;
//            Ekf->currents_position.speed        = Nmpc->vehicle_speed;//Nmpc->car_position.speed;
//            Ekf->currents_position.radius       = Nmpc->car_position.radius;
//            ResetExtendedKalmanFilter(Ekf);
//            InitialExtendedKalmanFilter(Ekf);
//            Ekf->last_Time = adtf_util::cHighResTimer::GetTime();
//            position_input_flag = true;
//            LOG_INFO("Extended Kalman Filter initialed!!");
//        }
//        else
//        {
//            Ekf->currents_position.X_Position   = Nmpc->car_position.X_Position;
//            Ekf->currents_position.Y_Position   = Nmpc->car_position.Y_Position;
//            Ekf->currents_position.HeadingAngle = Nmpc->car_position.HeadingAngle;
//            Ekf->currents_position.speed        = Nmpc->vehicle_speed;//Nmpc->car_position.speed;
//            Ekf->currents_position.radius       = Nmpc->car_position.radius;
//        }

    RETURN_NOERROR;
}

tResult cOptIlm_ModelPredictiveController::GetTrajectoryPoint(void)
{
    static int last_number_of_trajectory_setpoint = 12;

    object_ptr<const ISample> pSampleFromTraPoint;
    if (IS_OK(m_oInput_trajectory_point.GetLastSample(pSampleFromTraPoint)))
    {

        auto oDecoder = m_TrajectoryPointSampleFactory.MakeDecoderFor(*pSampleFromTraPoint);

        RETURN_IF_FAILED(oDecoder.IsValid());

        const tWorldCoordinate* pCoordinates = reinterpret_cast<const tWorldCoordinate*>(oDecoder.GetElementAddress(m_ddlTrajectoryPointDataId.pointArray));
        int point_counter = 0;
        for (int index = 0; index < 20; index++)
        {
            if(pCoordinates[index].world_X == 999 && pCoordinates[index].world_Y == 999)
            {
                Nmpc->trajectory_setpoint.X[index] = 0;
                Nmpc->trajectory_setpoint.Y[index] = 0;
            }
            else
            {
                Nmpc->trajectory_setpoint.X[index] = pCoordinates[index].world_X;
                Nmpc->trajectory_setpoint.Y[index] = pCoordinates[index].world_Y;
                point_counter++;
            }
            //            Nmpc->trajectory_setpoint.Y[index] = 0;
            // for debug
//            LOG_INFO(adtf_util::cString::Format("Trajectory Point%d X: %f Y: %f", index, Nmpc->trajectory_setpoint.X[index], Nmpc->trajectory_setpoint.Y[index]));
        }

        Nmpc->number_of_trajectory_setpoint = (int)pCoordinates[20].world_X;
        Nmpc->target_speed                  = pCoordinates[20].world_Y;

        if(last_number_of_trajectory_setpoint != Nmpc->number_of_trajectory_setpoint)
        {
            NmpcInitial(Nmpc, initial_flag);
            last_number_of_trajectory_setpoint = Nmpc->number_of_trajectory_setpoint;

            //for debug
            LOG_INFO(adtf_util::cString::Format("Trajectory Point Counter %d", Nmpc->N_dt));
        }

        //for debug
//        LOG_INFO(adtf_util::cString::Format("Target Speed %g", Nmpc->target_speed));

    }

    RETURN_NOERROR;
}
