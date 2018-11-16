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

#include "Optilm_MapVisualization.h"
#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"
#include<pthread.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_OPTILM_AUTONOMOUS_DRIVING_DATA_TRIGGERED_FILTER,
                                    "Optilm MapVisualization",
                                    cOptilm_MapVisualization,
                                    adtf::filter::thread_trigger(true))

bool system_initial = false;



DATA_STRUCT line_point[18];
POINT_STRUCT parking_point[8];
DATA_STRUCT crossing_point[2];
KURVE_STRUCT kurve[4];
ROAD_SIGN road_sign[20];

cOptilm_MapVisualization::cOptilm_MapVisualization()
{
    SetName("Optilm MapVisualization");


    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register output pin
    Register(m_oVideo_Output, "Video_output", pType);



//    //DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
//    object_ptr<IStreamType> pTypeSignalValue;
//    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory))
//    {
//        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
//        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
//    }
//    else
//    {
//        LOG_ERROR("No mediadescription for tSignalValue found!");
//    }
//    //Input
//    Register(m_oInput_VehicleSpeed, "VehicleSpeed" , pTypeSignalValue);
   


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
    Register(m_oInput_Position2, "Position2" , pTypePositionData);



    system_initial = false;
    m_lastSampleTime = 0;

}


tResult cOptilm_MapVisualization::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    
    RETURN_NOERROR;
}

float temp_last_driving_model_flag = CAR_STOP;
tResult cOptilm_MapVisualization::Process(tTimeStamp tmTimeOfTrigger)
{
    tTimeStamp m_NowSampleTime =  tmTimeOfTrigger;
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.000001;


    //Get data from pin
    GetPosition();
    GetPosition2();
//    GetCarSpeed();


    if(f64SampleTime > 0.05)
    {
        m_lastSampleTime = m_NowSampleTime;

        //Filter sample time test for debug
        //        LOG_INFO(cString::Format("StateControl Sample Time = %.3f S", f64SampleTime));


        if(system_initial == false)
        {
            SystemInitial();
        }
        else
        {
            TransmitDataFusionVideo();
        }



    }
    RETURN_NOERROR;
}

tResult cOptilm_MapVisualization::SystemInitial()
{
    position_initial_flag = tFalse;
    position_initial_flag2 = tFalse;


    line_point[0].down.from.X   = 200;    line_point[0].down.to.X   = 1619;
    line_point[0].down.from.Y   = 0;      line_point[0].down.to.Y   = 0;
    line_point[0].center.from.X = 200;    line_point[0].center.to.X = 1619;
    line_point[0].center.from.Y = 48;     line_point[0].center.to.Y = 48;
    line_point[0].top.from.X    = 200;    line_point[0].top.to.X    = 1619;
    line_point[0].top.from.Y    = 96;     line_point[0].top.to.Y    = 96;

    line_point[1].down.from.X   = 200;    line_point[1].down.to.X   = 1619;
    line_point[1].down.from.Y   = 296;    line_point[1].down.to.Y   = 296;
    line_point[1].center.from.X = 200;    line_point[1].center.to.X = 1619;
    line_point[1].center.from.Y = 344;    line_point[1].center.to.Y = 344;
    line_point[1].top.from.X    = 200;    line_point[1].top.to.X    = 1619;
    line_point[1].top.from.Y    = 392;    line_point[1].top.to.Y    = 392;

    line_point[2].down.from.X   = 1619;   line_point[2].down.to.X   = 1619;
    line_point[2].down.from.Y   = 96;     line_point[2].down.to.Y   = 296;
    line_point[2].center.from.X = 1667;   line_point[2].center.to.X = 1667;
    line_point[2].center.from.Y = 96;     line_point[2].center.to.Y = 296;
    line_point[2].top.from.X    = 1715;   line_point[2].top.to.X    = 1715;
    line_point[2].top.from.Y    = 96;     line_point[2].top.to.Y    = 296;

    line_point[3].down.from.X   = 1619;   line_point[3].down.to.X   = 1619;
    line_point[3].down.from.Y   = 396;    line_point[3].down.to.Y   = 596;
    line_point[3].center.from.X = 1667;   line_point[3].center.to.X = 1667;
    line_point[3].center.from.Y = 396;    line_point[3].center.to.Y = 596;
    line_point[3].top.from.X    = 1715;   line_point[3].top.to.X    = 1715;
    line_point[3].top.from.Y    = 392;    line_point[3].top.to.Y    = 592;

    line_point[4].down.from.X   = 1715;   line_point[4].down.to.X   = 1922;
    line_point[4].down.from.Y   = 0;      line_point[4].down.to.Y   = 0;
    line_point[4].center.from.X = 1715;   line_point[4].center.to.X = 1922;
    line_point[4].center.from.Y = 48;     line_point[4].center.to.Y = 48;
    line_point[4].top.from.X    = 1715;   line_point[4].top.to.X    = 1922;
    line_point[4].top.from.Y    = 96;     line_point[4].top.to.Y    = 96;

    line_point[5].down.from.X   = 1715;   line_point[5].down.to.X   = 2022;
    line_point[5].down.from.Y   = 296;    line_point[5].down.to.Y   = 296;
    line_point[5].center.from.X = 1715;   line_point[5].center.to.X = 2022;
    line_point[5].center.from.Y = 344;    line_point[5].center.to.Y = 344;
    line_point[5].top.from.X    = 1715;   line_point[5].top.to.X    = 2022;
    line_point[5].top.from.Y    = 392;    line_point[5].top.to.Y    = 392;

    line_point[6].down.from.X   = 1815;   line_point[6].down.to.X   = 1922;
    line_point[6].down.from.Y   = 692;    line_point[6].down.to.Y   = 692;
    line_point[6].center.from.X = 1815;   line_point[6].center.to.X = 1922;
    line_point[6].center.from.Y = 740;    line_point[6].center.to.Y = 740;
    line_point[6].top.from.X    = 1815;   line_point[6].top.to.X    = 1922;
    line_point[6].top.from.Y    = 788;    line_point[6].top.to.Y    = 788;

    line_point[7].down.from.X   = 2022;   line_point[7].down.to.X   = 2022;
    line_point[7].down.from.Y   = 196;    line_point[7].down.to.Y   = 296;
    line_point[7].center.from.X = 2070;   line_point[7].center.to.X = 2070;
    line_point[7].center.from.Y = 196;    line_point[7].center.to.Y = 296;
    line_point[7].top.from.X    = 2118;   line_point[7].top.to.X    = 2118;
    line_point[7].top.from.Y    = 192;    line_point[7].top.to.Y    = 292;

    line_point[8].down.from.X   = 2022;   line_point[8].down.to.X   = 2022;
    line_point[8].down.from.Y   = 396;    line_point[8].down.to.Y   = 596;
    line_point[8].center.from.X = 2070;   line_point[8].center.to.X = 2070;
    line_point[8].center.from.Y = 396;    line_point[8].center.to.Y = 596;
    line_point[8].top.from.X    = 2118;   line_point[8].top.to.X    = 2118;
    line_point[8].top.from.Y    = 392;    line_point[8].top.to.Y    = 592;



    crossing_point[0].down.from.X   = 1619;   crossing_point[0].down.to.X   = 1715;
    crossing_point[0].down.from.Y   = 0;      crossing_point[0].down.to.Y   = 0;
    crossing_point[0].center.from.X = 1619;   crossing_point[0].center.to.X = 1715;
    crossing_point[0].center.from.Y = 48;     crossing_point[0].center.to.Y = 48;

    crossing_point[1].down.from.X   = 2118;   crossing_point[1].down.to.X   = 2118;
    crossing_point[1].down.from.Y   = 296;    crossing_point[1].down.to.Y   = 392;
    crossing_point[1].center.from.X = 2070;   crossing_point[1].center.to.X = 2070;
    crossing_point[1].center.from.Y = 296;    crossing_point[1].center.to.Y = 392;




    kurve[0].down.center.X     = 196;    kurve[0].down.center.Y    = 196;
    kurve[0].down.star_angle   = 90;     kurve[0].down.end_angle   = 270;
    kurve[0].down.radius       = 196;
    kurve[0].center.center.X   = 196;    kurve[0].center.center.Y  = 196;
    kurve[0].center.star_angle = 90;     kurve[0].center.end_angle = 270;
    kurve[0].center.radius     = 148;
    kurve[0].top.center.X      = 196;    kurve[0].top.center.Y     = 196;
    kurve[0].top.star_angle    = 90;     kurve[0].top.end_angle    = 270;
    kurve[0].top.radius        = 100;

    kurve[1].down.center.X     = 1815;    kurve[1].down.center.Y    = 592;
    kurve[1].down.star_angle   = 180;     kurve[1].down.end_angle   = 270;
    kurve[1].down.radius       = 196;
    kurve[1].center.center.X   = 1815;    kurve[1].center.center.Y  = 592;
    kurve[1].center.star_angle = 180;     kurve[1].center.end_angle = 270;
    kurve[1].center.radius     = 148;
    kurve[1].top.center.X      = 1815;    kurve[1].top.center.Y     = 592;
    kurve[1].top.star_angle    = 180;     kurve[1].top.end_angle    = 270;
    kurve[1].top.radius        = 100;

    kurve[2].down.center.X     = 1922;   kurve[2].down.center.Y    = 592;
    kurve[2].down.star_angle   = 270;    kurve[2].down.end_angle   = 360;
    kurve[2].down.radius       = 196;
    kurve[2].center.center.X   = 1922;   kurve[2].center.center.Y  = 592;
    kurve[2].center.star_angle = 270;    kurve[2].center.end_angle = 360;
    kurve[2].center.radius     = 148;
    kurve[2].top.center.X      = 1922;   kurve[2].top.center.Y     = 592;
    kurve[2].top.star_angle    = 270;    kurve[2].top.end_angle    = 360;
    kurve[2].top.radius        = 100;

    kurve[3].down.center.X     = 1922;   kurve[3].down.center.Y    = 196;
    kurve[3].down.star_angle   = 0;      kurve[3].down.end_angle   = 90;
    kurve[3].down.radius       = 196;
    kurve[3].center.center.X   = 1922;   kurve[3].center.center.Y  = 196;
    kurve[3].center.star_angle = 0;      kurve[3].center.end_angle = 90;
    kurve[3].center.radius     = 148;
    kurve[3].top.center.X      = 1922;   kurve[3].top.center.Y     = 196;
    kurve[3].top.star_angle    = 0;      kurve[3].top.end_angle    = 90;
    kurve[3].top.radius        = 100;

    parking_point[0].from.X = 522;  parking_point[0].from.Y = 217;
    parking_point[0].to.X   = 613;  parking_point[0].to.Y   = 217;

    parking_point[1].from.X = 622;  parking_point[1].from.Y = 217;
    parking_point[1].to.X   = 713;  parking_point[1].to.Y   = 217;

    parking_point[2].from.X = 522;  parking_point[2].from.Y = 217;
    parking_point[2].to.X   = 522;  parking_point[2].to.Y   = 294;

    parking_point[3].from.X = 567;  parking_point[3].from.Y = 217;
    parking_point[3].to.X   = 567;  parking_point[3].to.Y   = 294;

    parking_point[4].from.X = 613;  parking_point[4].from.Y = 217;
    parking_point[4].to.X   = 613;  parking_point[4].to.Y   = 294;

    parking_point[5].from.X = 622;  parking_point[5].from.Y = 217;
    parking_point[5].to.X   = 622;  parking_point[5].to.Y   = 294;

    parking_point[6].from.X = 667;  parking_point[6].from.Y = 217;
    parking_point[6].to.X   = 667;  parking_point[6].to.Y   = 294;

    parking_point[7].from.X = 713;  parking_point[7].from.Y = 217;
    parking_point[7].to.X   = 713;  parking_point[7].to.Y   = 294;


    road_sign[0].id = 11;             road_sign[0].direction = 90;
    road_sign[0].position.X = 693;    road_sign[0].position.Y = 416;

    road_sign[1].id = 1;              road_sign[1].direction = -90;
    road_sign[1].position.X = 1603;   road_sign[1].position.Y = 133;

    road_sign[2].id = 1;              road_sign[2].direction = 0;
    road_sign[2].position.X = 1984;   road_sign[2].position.Y = 280;

    road_sign[3].id = 3;              road_sign[3].direction = 0;
    road_sign[3].position.X = 1581;   road_sign[3].position.Y = -15;

    road_sign[4].id = 3;              road_sign[4].direction = 180;
    road_sign[4].position.X = 1752;   road_sign[4].position.Y = 407;

    road_sign[5].id = 3;              road_sign[5].direction = 90;
    road_sign[5].position.X = 2133;   road_sign[5].position.Y = 258;

    road_sign[6].id = 3;              road_sign[6].direction = 0;
    road_sign[6].position.X = 1581;   road_sign[6].position.Y = 280;

    road_sign[7].id = 3;              road_sign[7].direction = 180;
    road_sign[7].position.X = 1752;   road_sign[7].position.Y = 111;

    road_sign[8].id = 3;              road_sign[8].direction = -90;
    road_sign[8].position.X = 2006;   road_sign[8].position.Y = 429;

    road_sign[9].id = 5;              road_sign[9].direction = 90;
    road_sign[9].position.X = 1730;   road_sign[9].position.Y = 258;

    road_sign[10].id = 5;              road_sign[10].direction = -90;
    road_sign[10].position.X = 1603;   road_sign[10].position.Y = 429;

    road_sign[11].id = 10;             road_sign[11].direction = 0;
    road_sign[11].position.X = 548;    road_sign[11].position.Y = -15;

    road_sign[12].id = 10;             road_sign[12].direction = 180;
    road_sign[12].position.X = 1253;   road_sign[12].position.Y = 111;

    road_sign[13].id = 10;             road_sign[13].direction = 0;
    road_sign[13].position.X = 1253;   road_sign[13].position.Y = 280;

    road_sign[14].id = 10;             road_sign[14].direction = 180;
    road_sign[14].position.X = 348;    road_sign[14].position.Y = 407;

    road_sign[15].id = 2;              road_sign[15].direction = 0;
    road_sign[15].position.X = 381;    road_sign[15].position.Y = 280;

    road_sign[16].id = 8;              road_sign[16].direction = 180;
    road_sign[16].position.X = 348;    road_sign[16].position.Y = 111;

    road_sign[17].id = 8;              road_sign[17].direction = 180;
    road_sign[17].position.X = 1253;   road_sign[17].position.Y = 407;

    road_sign[18].id = 14;             road_sign[18].direction = 180;
    road_sign[18].position.X = 1476;   road_sign[18].position.Y = 111;

    road_sign[19].id = 14;             road_sign[19].direction = 0;
    road_sign[19].position.X = 348;    road_sign[19].position.Y = -15;




    position_data[HEADING]= 0;

    system_initial = true;
    RETURN_NOERROR;
}





//////////////////////////////////////////////////////////
/// Get Pin data
///
///
tResult cOptilm_MapVisualization::GetPosition()
{

    object_ptr<const ISample> pReadSampleFromPosition;
    while (IS_OK(m_oInput_Position.GetNextSample(pReadSampleFromPosition)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSampleFromPosition);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &position_data[X]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &position_data[Y]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &position_data[SPEED]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &position_data[HEADING]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &position_data[RADIUS]));

        position_initial_flag = tTrue;
        //show position pin data for debug
        //        LOG_INFO(cString::Format("position x:%g y:%g heading:%g speed:%g radius:g ", Nmpc->car_position.X_Position, Nmpc->car_position.Y_Position, Nmpc->car_position.HeadingAngle, Nmpc->car_position.speed, Nmpc->car_position.radius));

        position_data[X] = position_data[X] * 100;
        position_data[Y] = position_data[Y] * 100;
    }

    RETURN_NOERROR;
}

tResult cOptilm_MapVisualization::GetPosition2()
{

    object_ptr<const ISample> pReadSampleFromPosition;
    while (IS_OK(m_oInput_Position2.GetNextSample(pReadSampleFromPosition)))
    {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSampleFromPosition);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &position_data2[X]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &position_data2[Y]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &position_data2[SPEED]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &position_data2[HEADING]));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &position_data2[RADIUS]));

        position_initial_flag2 = tTrue;
        //show position pin data for debug
        //        LOG_INFO(cString::Format("position x:%g y:%g heading:%g speed:%g radius:g ", Nmpc->car_position.X_Position, Nmpc->car_position.Y_Position, Nmpc->car_position.HeadingAngle, Nmpc->car_position.speed, Nmpc->car_position.radius));

        position_data2[X] = position_data2[X] * 100;
        position_data2[Y] = position_data2[Y] * 100;
    }

    RETURN_NOERROR;
}

//tResult cOptilm_MapVisualization::GetCarSpeed()
//{

//    object_ptr<const ISample> pSampleFromVcc;


//    while (IS_OK(m_oInput_VehicleSpeed.GetLastSample(pSampleFromVcc)))
//    {
//        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromVcc);

//        RETURN_IF_FAILED(oDecoder.IsValid());

//        // retrieve the values (using convenience methods that return a variant)
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &vehicle_speed.f32Value));
//        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &vehicle_speed.ui32ArduinoTimestamp));

//    }

//    RETURN_NOERROR;
//}










/////////////////////////////////////////////////
/// Send Data
///
///
tResult cOptilm_MapVisualization::TransmitDataFusionVideo(void)
{

#define IMAGE_HEIGHT 1200
#define IMAGE_WIDTH  2300
#define IMAGE_Y_ZERO 1000
#define IMAGE_X_ZERO 100

    // new image for result
    cv::Mat outputImage;    
    outputImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    //Draw MAP range
    rectangle(outputImage, Point(0,0), Point(IMAGE_WIDTH,IMAGE_HEIGHT), Scalar(0,0,0), -1);
    rectangle(outputImage, Point(0,0), Point(IMAGE_WIDTH-3,IMAGE_HEIGHT-3), Scalar(0,255,0), 2);

    //Draw X, Y
    line(outputImage, Point(50, 1100), Point(250, 1100), Scalar(0,255,0), 3);
    line(outputImage, Point(230, 1110), Point(250, 1100), Scalar(0,255,0), 3);
    line(outputImage, Point(230, 1090), Point(250, 1100), Scalar(0,255,0), 3);
    line(outputImage, Point(50, 1100), Point(50, 900), Scalar(0,255,0), 3);
    line(outputImage, Point(60, 910), Point(50, 900), Scalar(0,255,0), 3);
    line(outputImage, Point(40, 910), Point(50, 900), Scalar(0,255,0), 3);
    putText(outputImage, "X,0", Point(260 , 1110), 0, 1, Scalar(0,255,0),3);
    putText(outputImage, "Y,90", Point(30 , 885), 0, 1, Scalar(0,255,0),3);



    //Draww Map
    for(int index = 0; index < 9; index++)
    {
        line(outputImage, Point(IMAGE_X_ZERO + line_point[index].down.from.X,IMAGE_Y_ZERO - line_point[index].down.from.Y), Point(IMAGE_X_ZERO + line_point[index].down.to.X, IMAGE_Y_ZERO - line_point[index].down.to.Y), Scalar(255,255,255), 3);
        line(outputImage, Point(IMAGE_X_ZERO + line_point[index].center.from.X,IMAGE_Y_ZERO - line_point[index].center.from.Y), Point(IMAGE_X_ZERO + line_point[index].center.to.X, IMAGE_Y_ZERO - line_point[index].center.to.Y), Scalar(128,128,128), 2);
        line(outputImage, Point(IMAGE_X_ZERO + line_point[index].top.from.X,IMAGE_Y_ZERO - line_point[index].top.from.Y), Point(IMAGE_X_ZERO + line_point[index].top.to.X, IMAGE_Y_ZERO - line_point[index].top.to.Y), Scalar(255,255,255), 3);
    }

    for(int index = 0; index < 2; index++)
    {
        line(outputImage, Point(IMAGE_X_ZERO + crossing_point[index].down.from.X,IMAGE_Y_ZERO - crossing_point[index].down.from.Y), Point(IMAGE_X_ZERO + crossing_point[index].down.to.X, IMAGE_Y_ZERO - crossing_point[index].down.to.Y), Scalar(255,255,255), 3);
        line(outputImage, Point(IMAGE_X_ZERO + crossing_point[index].center.from.X,IMAGE_Y_ZERO - crossing_point[index].center.from.Y), Point(IMAGE_X_ZERO + crossing_point[index].center.to.X, IMAGE_Y_ZERO - crossing_point[index].center.to.Y), Scalar(128,128,128), 2);
    }

    for(int index = 0; index < 4; index++)
    {
        ellipse(outputImage, Point(IMAGE_X_ZERO + kurve[index].down.center.X,IMAGE_Y_ZERO - kurve[index].down.center.Y), Size(kurve[index].down.radius,kurve[index].down.radius), 0, kurve[index].down.star_angle, kurve[index].down.end_angle, Scalar(255,255,255), 3);
        ellipse(outputImage, Point(IMAGE_X_ZERO + kurve[index].center.center.X,IMAGE_Y_ZERO - kurve[index].center.center.Y), Size(kurve[index].center.radius,kurve[index].center.radius), 0, kurve[index].center.star_angle, kurve[index].center.end_angle, Scalar(128,128,128), 2);
        ellipse(outputImage, Point(IMAGE_X_ZERO + kurve[index].top.center.X,IMAGE_Y_ZERO - kurve[index].top.center.Y), Size(kurve[index].top.radius,kurve[index].top.radius), 0, kurve[index].top.star_angle, kurve[index].top.end_angle, Scalar(255,255,255), 3);
    }


    for(int index = 0; index < 8; index++)
    {
        line(outputImage, Point(IMAGE_X_ZERO + parking_point[index].from.X,IMAGE_Y_ZERO - parking_point[index].from.Y), Point(IMAGE_X_ZERO + parking_point[index].to.X, IMAGE_Y_ZERO - parking_point[index].to.Y), Scalar(255,255,255), 3);
    }


    char text[50];
    for(int index = 0; index < 20; index++)
    {
        rectangle(outputImage, Point(IMAGE_X_ZERO + road_sign[index].position.X - 10,IMAGE_Y_ZERO - road_sign[index].position.Y + 10), Point(IMAGE_X_ZERO + road_sign[index].position.X + 10,IMAGE_Y_ZERO - road_sign[index].position.Y - 10), Scalar( 0, 255,0), 2);
        sprintf(text, "%d" ,road_sign[index].id);
        putText(outputImage, text, Point(IMAGE_X_ZERO + road_sign[index].position.X - 8,IMAGE_Y_ZERO - road_sign[index].position.Y + 8), 0, 0.6, Scalar(255,255,255),2);
    }


    //Draw Car
    int car_point[4][2];
    float l_frant = 36 + 12;
    float l_rear  = 0 + 12;
    float car_width = 15;
    int npt[] = {4};

    if(position_initial_flag)
    {

        car_point[0][X] = (int)((cos(position_data[HEADING]) * (l_frant)) - (sin(position_data[HEADING]) * -car_width ) + position_data[X]);
        car_point[0][Y] = (int)((sin(position_data[HEADING]) * (l_frant)) + (cos(position_data[HEADING]) * -car_width)  + position_data[Y]);
        car_point[1][X] = (int)((cos(position_data[HEADING]) * (-l_rear)) - (sin(position_data[HEADING]) * -car_width ) + position_data[X]);
        car_point[1][Y] = (int)((sin(position_data[HEADING]) * (-l_rear)) + (cos(position_data[HEADING]) * -car_width)  + position_data[Y]);
        car_point[2][X] = (int)((cos(position_data[HEADING]) * (-l_rear)) - (sin(position_data[HEADING]) * car_width ) + position_data[X]);
        car_point[2][Y] = (int)((sin(position_data[HEADING]) * (-l_rear)) + (cos(position_data[HEADING]) * car_width)  + position_data[Y]);
        car_point[3][X] = (int)((cos(position_data[HEADING]) * (l_frant)) - (sin(position_data[HEADING]) * car_width ) + position_data[X]);
        car_point[3][Y] = (int)((sin(position_data[HEADING]) * (l_frant)) + (cos(position_data[HEADING]) * car_width)  + position_data[Y]);

        Point points[1][4];
        points[0][0] = Point(IMAGE_X_ZERO + car_point[0][X], IMAGE_Y_ZERO - car_point[0][Y]);
        points[0][1] = Point(IMAGE_X_ZERO + car_point[1][X], IMAGE_Y_ZERO - car_point[1][Y]);
        points[0][2] = Point(IMAGE_X_ZERO + car_point[2][X], IMAGE_Y_ZERO - car_point[2][Y]);
        points[0][3] = Point(IMAGE_X_ZERO + car_point[3][X], IMAGE_Y_ZERO - car_point[3][Y]);

        const Point* ppt[1] = {points[0]};
        polylines(outputImage, ppt, npt, 1, 1, Scalar(255,0,0),2);
        circle(outputImage, Point(IMAGE_X_ZERO + position_data[X], IMAGE_Y_ZERO - position_data[Y]), 3, Scalar(255,0,0), -1);
    }

    if(position_initial_flag2)
    {
        l_frant = 18 + 12;
        l_rear  = 18 + 12;

        car_point[0][X] = (int)((cos(position_data2[HEADING]) * (l_frant)) - (sin(position_data2[HEADING]) * -car_width ) + position_data2[X]);
        car_point[0][Y] = (int)((sin(position_data2[HEADING]) * (l_frant)) + (cos(position_data2[HEADING]) * -car_width)  + position_data2[Y]);
        car_point[1][X] = (int)((cos(position_data2[HEADING]) * (-l_rear)) - (sin(position_data2[HEADING]) * -car_width ) + position_data2[X]);
        car_point[1][Y] = (int)((sin(position_data2[HEADING]) * (-l_rear)) + (cos(position_data2[HEADING]) * -car_width)  + position_data2[Y]);
        car_point[2][X] = (int)((cos(position_data2[HEADING]) * (-l_rear)) - (sin(position_data2[HEADING]) * car_width ) + position_data2[X]);
        car_point[2][Y] = (int)((sin(position_data2[HEADING]) * (-l_rear)) + (cos(position_data2[HEADING]) * car_width)  + position_data2[Y]);
        car_point[3][X] = (int)((cos(position_data2[HEADING]) * (l_frant)) - (sin(position_data2[HEADING]) * car_width ) + position_data2[X]);
        car_point[3][Y] = (int)((sin(position_data2[HEADING]) * (l_frant)) + (cos(position_data2[HEADING]) * car_width)  + position_data2[Y]);

        Point points2[1][4];
        points2[0][0] = Point(IMAGE_X_ZERO + car_point[0][X], IMAGE_Y_ZERO - car_point[0][Y]);
        points2[0][1] = Point(IMAGE_X_ZERO + car_point[1][X], IMAGE_Y_ZERO - car_point[1][Y]);
        points2[0][2] = Point(IMAGE_X_ZERO + car_point[2][X], IMAGE_Y_ZERO - car_point[2][Y]);
        points2[0][3] = Point(IMAGE_X_ZERO + car_point[3][X], IMAGE_Y_ZERO - car_point[3][Y]);

        const Point* ppt2[1] = {points2[0]};
        polylines(outputImage, ppt2, npt, 1, 1, Scalar(0,0,255),2);
        circle(outputImage, Point(IMAGE_X_ZERO + position_data2[X], IMAGE_Y_ZERO - position_data2[Y]), 3, Scalar(0,0,255), -1);
    }





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






