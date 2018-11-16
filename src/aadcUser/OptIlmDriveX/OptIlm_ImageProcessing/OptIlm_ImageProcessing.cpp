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

#include "stdafx.h"
#include "OptIlm_ImageProcessing.h"
#include "ADTF3_OpenCV_helper.h"
#include "ImageTranslate.h"


//Image Processing
#include "Algorithm/InitialVariable.h"
#include "Algorithm/FunctionType.h"



ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_COPENCVTEMPLATE_DATA_TRIGGERED_FILTER,
                                    "OptIlm ImageProcessing",
                                    cOptIlm_ImageProcessing,
//                                    adtf::filter::pin_trigger({ "video_input" }))
                                    adtf::filter::thread_trigger(true))


//Image Processing
ITS *image_processing;
IMAGE_TRANSLATE *im_T;  	    //Image source down sample Struct (all Image source to type YUV)
IMAGE_BUFFER *VinSource;       //Orginales Bild aus Kamera


char image_algorithm_initial_flag = 0;



// new image for result
Mat outputImage;
Mat outputDebugImage;

float lane_data[8];

cOptIlm_ImageProcessing::cOptIlm_ImageProcessing()
{

    SetName("Image Processing");

    //register properties

    RegisterPropertyVariable("1.Show processed video", m_processed_video);
    RegisterPropertyVariable("2.Show debug video", m_debug_video);


    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "video_input", pType);
    //Register output pin
    Register(m_oWriter, "video_output", pType);
    Register(m_oImageDebug, "video_debug", pType);


    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
    });


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
        LOG_WARNING("No mediadescription for tLaneCurveData found!");
    }
//    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;
    //Create Pin
    Register(m_oLaneData, "LaneData" , pTypeLaneData);


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
    Register(m_oInput_steering_angle, "SteeringAngle" , pTypeSignalValue);
    Register(m_oInput_control_mode, "ImageControlMode" , pTypeSignalValue);






    im_T = (IMAGE_TRANSLATE*)calloc(1,sizeof(IMAGE_TRANSLATE));
    VinSource = (IMAGE_BUFFER*)calloc(1,sizeof(IMAGE_BUFFER));


    if(image_algorithm_initial_flag == 0)
    {
        image_processing = (ITS*)calloc(1,sizeof(ITS));
        SetITSBuffer(image_processing);
        ResetConstant(image_processing);
        image_algorithm_initial_flag = 1;
//            image_processing->traindata = fopen("TrainData.txt","w");

    }

    image_processing->YImg      = &VinSource->Y[0];
    image_processing->UImg      = &VinSource->U[0];
    image_processing->VImg      = &VinSource->V[0];
    image_processing->Showimage = &VinSource->Y[0];
    image_processing->ShowUImg  = &VinSource->U[0];
    image_processing->ShowVImg  = &VinSource->V[0];

    outputImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    outputDebugImage.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    image_processing->function_switch.input_flag |= LANE_DETECTION;
//    image_processing->function_switch.input_flag |= STOP_LINE_DETECTION;
//    image_processing->function_switch.input_flag |= ADULT_DETECTION;
//    image_processing->function_switch.input_flag |= CHILD_DETECTION;


    m_lastSampleTime = 0;
}

//cOptIlm_ImageProcessing::~cOptIlm_ImageProcessing()
//{
//    FreeMemory(image_processing);
//    free(image_processing);
//    free(VinSource);
//    free(im_T);

//}

tResult cOptIlm_ImageProcessing::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));


    RETURN_NOERROR;
}
bool process_image_flag = false;
tResult cOptIlm_ImageProcessing::Process(tTimeStamp tmTimeOfTrigger)
{
    // avoid that the method triggers itself due to the feedback loop
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    tTimeStamp m_NowSampleTime =  tmTimeOfTrigger;
    f64SampleTime = (tFloat64)(m_NowSampleTime - m_lastSampleTime);
    f64SampleTime *= 0.001;


//    GetSteeringAngle();
//    GetImageMode();


//    object_ptr<const ISample> pReadSample;
//    while (IS_OK(m_oReader.GetNextSample(pReadSample)) && process_image_flag == false)
//    {
//        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
//        //lock read buffer
//        if (IS_OK(pReadSample->Lock(pReadBuffer)))
//        {
//            //create a opencv matrix from the media sample buffer
//            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
//                                 CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

//            //Do the image processing and copy to destination image buffer
//            //            Canny(inputImage, outputDebugImage, 100, 200);// Detect Edges


//            ImageBufferDownsamplingBGR_to_YUY2(IMAGE_WIDTH, IMAGE_HEIGHT, inputImage, 0);
//            ITSLANE_MAIN(image_processing);
//            Transfer_YUY2_to_BGR(IMAGE_WIDTH, IMAGE_HEIGHT, outputImage);
//            DrawDebugImage(IMAGE_WIDTH, IMAGE_HEIGHT, outputDebugImage);

//        }
//    }


    if(f64SampleTime > 33)
    {
        m_lastSampleTime = m_NowSampleTime;
        process_image_flag = true;
        //Filter sample time test for debug
//        LOG_INFO(cString::Format("Image Processing Frame rate Time = %.0f fs", (1000/f64SampleTime)));

        GetSteeringAngle();
        GetImageMode();

        object_ptr<const ISample> pReadSample;
        if (IS_OK(m_oReader.GetLastSample(pReadSample)))
        {
            object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
            //lock read buffer
            if (IS_OK(pReadSample->Lock(pReadBuffer)))
            {
                //create a opencv matrix from the media sample buffer
                Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                     CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));


//                tTimeStamp process_Time_befor =  adtf_util::cHighResTimer::GetTime();

                ImageBufferDownsamplingBGR_to_YUY2(IMAGE_WIDTH, IMAGE_HEIGHT, inputImage, 0);

                ITSLANE_MAIN(image_processing);

                TransmitLaneData();

//                tTimeStamp process_Time_after = adtf_util::cHighResTimer::GetTime();

//                tFloat64 process_Time = (tFloat64)(process_Time_after - process_Time_befor);
//                process_Time *= 0.001;
//                LOG_INFO(cString::Format("process_Time: %.f ", process_Time));
            }
        }








//        LOG_INFO(cString::Format("iTS->SL_LaneModel.M: %g   %g ",image_processing->SL_LaneModel.M, image_processing->SL_LaneModel.m));
//        LOG_INFO(cString::Format("iTS->L_SL_PointGroup.mark_width: %g ", image_processing->L_SL_PointGroup.mark_width));
//        LOG_INFO(cString::Format("L_evRodSlp: %g ", image_processing->L_evRodSlp));


        if(m_processed_video == tTrue)
            Transfer_YUY2_to_BGR(IMAGE_WIDTH, IMAGE_HEIGHT, outputImage);

        if(m_debug_video == tTrue)
            DrawDebugImage(IMAGE_WIDTH, IMAGE_HEIGHT, outputDebugImage);


        //Write processed Image to Output Pin
        if (!outputImage.empty() && m_processed_video == tTrue)
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriter, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
        }

        //Write processed Image to Output Pin
        if (!outputDebugImage.empty() && m_debug_video == tTrue)
        {
            //update output format if matrix size does not fit to
            if (outputDebugImage.total() * outputDebugImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oImageDebug, outputDebugImage);
            }
            // write to pin
            writeMatToPin(m_oImageDebug, outputDebugImage, m_pClock->GetStreamTime());
        }

        process_image_flag = false;
    }

    RETURN_NOERROR;
}

tResult cOptIlm_ImageProcessing::GetSteeringAngle()
{
    tSignalValue steering_angle;
    steering_angle.f32Value = 0;
    steering_angle.ui32ArduinoTimestamp = 0;
    object_ptr<const ISample> pSampleFromSteering;

    if (IS_OK(m_oInput_steering_angle.GetLastSample(pSampleFromSteering)))
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

tResult cOptIlm_ImageProcessing::GetImageMode()
{
    tSignalValue image_mode;
    object_ptr<const ISample> pSampleFromImageMode;

    if (IS_OK(m_oInput_control_mode.GetLastSample(pSampleFromImageMode)))
    {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleFromImageMode);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &image_mode.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp, &image_mode.ui32ArduinoTimestamp));

    }


    image_processing->function_switch.input_flag = (char)image_mode.f32Value;

//    image_processing->function_switch.input_flag &= ~LANE_DETECTION;
//    image_processing->function_switch.input_flag |= LANE_DETECTION;
//    image_processing->function_switch.input_flag |= STOP_LINE_DETECTION;
//    image_processing->function_switch.input_flag |= ADULT_DETECTION;
//    image_processing->function_switch.input_flag |= CHILD_DETECTION;



    //for debug
//    LOG_INFO(cString::Format("Steering Angle: %g ", steering_angle.f32Value));

    RETURN_NOERROR;
}

tResult cOptIlm_ImageProcessing::TransmitLaneData(void)
{


    lane_data[0] = (tFloat)image_processing->L_DetectMode;
    if(image_processing->L_DetectMode == LTRACE && image_processing->L_StbCtr == 15)
    {
        lane_data[0] = (tFloat)image_processing->L_DetectMode;
        lane_data[1] = image_processing->k;
        lane_data[2] = image_processing->m;
        lane_data[3] = image_processing->bm;

    }
    else if(image_processing->L_DetectMode == SL_TRACE && image_processing->SL_LaneModel.L_StbCtr == 25)
    {
        lane_data[0] = (tFloat)image_processing->L_DetectMode;
        lane_data[1] = image_processing->SL_LaneModel.k;
        lane_data[2] = image_processing->SL_LaneModel.m;
        lane_data[3] = image_processing->SL_LaneModel.bm;
    }
    else
    {
        lane_data[0] = LSEARCH;
//        lane_data[1] = 0;
//        lane_data[2] = 0;
//        lane_data[3] = 0;
    }
    lane_data[4] = (tFloat)image_processing->L_WAvg;
    lane_data[5] = (tFloat)image_processing->SL_LaneModel.L_SL_LorR;

    if(image_processing->adult.mode == TRACE && image_processing->adult.stable_counter > 6 && image_processing->adult.direction == 1)
        lane_data[6] = 2;
    else if(image_processing->adult.mode == TRACE && image_processing->adult.stable_counter > 6 && image_processing->adult.direction == 0)
        lane_data[6] = 1;
    else
        lane_data[6] = 0;

    if(image_processing->child.mode == TRACE && image_processing->child.stable_counter > 8)
        lane_data[7] = 1;
    else
        lane_data[7] = 0;


    object_ptr<ISample> pWriteSample;
    if (IS_OK(alloc_sample(pWriteSample)))
    {

        auto oCodec = m_LaneDataStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.laneDetectMode, lane_data[0]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.parameter_k, lane_data[1]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.parameter_m, lane_data[2]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.parameter_b, lane_data[3]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.laneWidth, lane_data[4]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.leftOrRightLine, lane_data[5]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.adult_flag, lane_data[6]));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneDataId.child_flag, lane_data[7]));

    }
    m_oLaneData << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}



void cOptIlm_ImageProcessing::CalculateVehiclePath(float steering)
{
    if(steering == 0)
    {
        image_processing->default_k = 0;
        image_processing->default_m = 0;
        image_processing->default_b = 0;
    }
    else if(steering > 0)
    {
        image_processing->default_k = -steering * 0.0000795;
        image_processing->default_m = -steering * 0.00338;
        image_processing->default_b = 0;
    }
    else if(steering < 0)
    {
        image_processing->default_k = -steering * 0.0001662;
        image_processing->default_m = -steering * -0.004729;
        image_processing->default_b = 0;
    }


}


/***********************************************************************************************************************************/
tResult cOptIlm_ImageProcessing::ImageBufferDownsamplingBGR_to_YUY2(int Im_width, int Im_height, const cv::Mat& image, char type)
{
    int row, col;
    int red = 0;
    int green = 0;
    int blue = 0;
    int index = 0;
    int index2 = 0;
    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            red  = (unsigned char)image.at<Vec3b>((row * 2),(col * 2))[0];
            green = (unsigned char)image.at<Vec3b>((row * 2),(col * 2))[1];
            blue   = (unsigned char)image.at<Vec3b>((row * 2),(col * 2))[2];

            VinSource->Y[index] = uCharLimitSet((int)((0.299*(float)red) + (0.587*(float)green) + (0.114*(float)blue)));
            index++;
            if(index % 2 == 0)
            {
                if(type == 0){
                    //es ist schneller aber unklar
                    VinSource->U[index2] = uCharLimitSet(((-43*red-85*green+128*blue)>> 8)+128);
                    VinSource->V[index2] = uCharLimitSet(((128*red-107*green-21*blue)>> 8)+128);
                }
                else if(type == 1){
                    //es ist langsamer aber kl?rer
                    VinSource->U[index2] = uCharLimitSet(-0.169*red-0.331*green+0.499*blue+128);
                    VinSource->V[index2] = uCharLimitSet(0.499*red-0.418*green-0.0813*blue+128);
                }
//                VinSource->U[index2] = 128;
//                VinSource->V[index2] = 128;
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult cOptIlm_ImageProcessing::Transfer_YUV_to_YUY2(int Im_width, int Im_height, const cv::Mat& image)
{
    int row, col;
    int index = 0;
    int index2 = 0;
    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            VinSource->Y[index] = (unsigned char)image.at<Vec3b>(row,col)[0];
            index++;
            if(index % 2 == 0)
            {
                VinSource->U[index2] = (unsigned char)image.at<Vec3b>(row,col)[1];
                VinSource->V[index2] = (unsigned char)image.at<Vec3b>(row,col)[2];
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult cOptIlm_ImageProcessing::Transfer_YUY2_to_YUV(int Im_width, int Im_height, cv::Mat& image)
{
    int row, col;
    int index = 0;
    int index2 = 0;


    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            image.at<Vec3b>(row,col)[0] = VinSource->Y[index];

            image.at<Vec3b>(row,col)[1] = VinSource->U[index2];
            image.at<Vec3b>(row,col)[2] = VinSource->V[index2];


            index++;
            if(index % 2 == 0)
            {
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult cOptIlm_ImageProcessing::Transfer_YUY2_to_BGR(int Im_width, int Im_height, cv::Mat& image)
{
    int row, col;
    int index = 0;
    int index2 = 0;


    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            image.at<Vec3b>(row,col)[2] = uCharLimitSet((298 * (VinSource->Y[index] - 16) + 516 * (VinSource->U[index2]- 128) + 128) >> 8);
            image.at<Vec3b>(row,col)[1] = uCharLimitSet((298 * (VinSource->Y[index] - 16) - 100 * (VinSource->U[index2] - 128) - 208 * (VinSource->V[index2] - 128) + 128) >> 8);
            image.at<Vec3b>(row,col)[0] = uCharLimitSet((298 * (VinSource->Y[index] - 16) + 409 * (VinSource->V[index2] - 128) + 128) >> 8);

            index++;
            if(index % 2 == 0)
            {
                index2++;
            }
        }
    }
    RETURN_NOERROR;
}

tResult cOptIlm_ImageProcessing::DrawDebugImage(int Im_width, int Im_height, cv::Mat& image)
{
    int row, col;
    int red = 0;
    int green = 0;
    int blue = 0;
    int index = 0;


    for(row = 0; row < Im_height; row++)
    {
        for(col = 0; col < Im_width; col++)
        {
            red = green = blue = 0;

//            if(image_processing->YImg[index] > image_processing->O_MarkLightTH)
//            {
//                red = green = blue = 255;
//            }
//            else
//            {
//                red = green = blue = 0;
//            }


            if((image_processing->O_InfoPlane[index] & HEVEINFO) == HEVEINFO)
            {
                red = 255;
                green = blue = 0;
            }
            else if((image_processing->O_InfoPlane[index] & VEINFO) == VEINFO)
            {
                green = 255;
                blue = red = 0;
            }
            else if((image_processing->O_InfoPlane[index] & HEINFO) == HEINFO)
            {
                blue = 255;
                green = red = 0;
            }

//            if((image_processing->O_InfoPlane[index] & RSDHEVEINFO) == RSDHEVEINFO)
//            {
//                red = 255;
//                green = blue = 0;
//            }


//            int Y = (image_processing->Hog_InfoPlane[index] * 7);
//            int Y = (image_processing->Gxy_InfoPlane[index]);
//            int U = 128;
//            int V = 128;

//            red   = uCharLimitSet((298 * (Y - 16) + 409 * (V - 128) + 128) >> 8);
//            green = uCharLimitSet((298 * (Y - 16) - 100 * (U - 128) - 208 * (V - 128) + 128) >> 8);
//            blue  = uCharLimitSet((298 * (Y - 16) + 516 * (U- 128) + 128) >> 8);


//            if(image_processing->O_P_InfoPlane[index] == 1)
//            {
//                red = 255;
//                green = blue = 0;
//            }

//            switch (image_processing->Axy_InfoPlane[index])
//            {
//                case 1:
//                    red   = 255;
//                    green = 0;
//                    blue  = 0;
//                    break;
//                case 2:
//                    red   = 255;
//                    green = 128;
//                    blue  = 0;
//                    break;
//                case 3:
//                    red   = 255;
//                    green = 128;
//                    blue  = 128;
//                    break;
//                case 4:
//                    red   = 0;
//                    green = 255;
//                    blue  = 0;
//                    break;
//                case 5:
//                    red   = 128;
//                    green = 255;
//                    blue  = 0;
//                    break;
//                case 6:
//                    red   = 128;
//                    green = 255;
//                    blue  = 128;
//                    break;
//                case 7:
//                    red   = 0;
//                    green = 0;
//                    blue  = 255;
//                    break;
//                case 8:
//                    red   = 0;
//                    green = 128;
//                    blue  = 255;
//                    break;
//                case 9:
//                    red   = 128;
//                    green = 128;
//                    blue  = 255;
//                    break;
//                case 0:
//                    red   = 0;
//                    green = 0;
//                    blue  = 0;
//                    break;
//                default:
//                    red   = 0;
//                    green = 0;
//                    blue  = 0;
//                    break;
//            }

//            if((image_processing->L_ColProjection[index] & VEINFO) == VEINFO)
//            {
//                green = 255;
//                blue = red = 0;
//            }
//            else
//            {
//                red = green = blue = 0;
//            }

            image.at<Vec3b>(row,col)[0] = blue;      //B
            image.at<Vec3b>(row,col)[1] = green;     //G
            image.at<Vec3b>(row,col)[2] = red;     //R

            index++;
        }
    }
    RETURN_NOERROR;
}
/***********************************************************************************************************************************/
