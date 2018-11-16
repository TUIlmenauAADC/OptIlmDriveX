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

#define CID_SIGNALVALUEGENERATOR_STREAMING_SOURCE "OptIlm_signal_value_generator.streaming_source.user.aadc.cid"
#define LABEL_SIGNALVALUEGENERATOR_STREAMING_SOURCE "OptIlm Signal Value Generator"


/*! the main class for the signal value generator. */
class OptIlm_SignalValueGenerator : public adtf::streaming::cSampleStreamingSource
{
public:
	ADTF_CLASS_ID_NAME(OptIlm_SignalValueGenerator,
        CID_SIGNALVALUEGENERATOR_STREAMING_SOURCE,
        LABEL_SIGNALVALUEGENERATOR_STREAMING_SOURCE);

	ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IReferenceClock), REQUIRE_INTERFACE(adtf::services::IKernel));

private:
	
	/*! The output pin */
    cSampleWriter m_SteeringOut;
    cSampleWriter m_SpeedOut;

    /*! The frame delay */
    property_variable<tInt64> m_timerInterval = 100;
    
    /*! The value to transmit */
    property_variable<tFloat32> m_value = 0.0f;



    /*! Media Descriptions ids. */
    struct ddlSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    
	/*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_signalValueSampleFactory;
	
	/*! The timer */
	kernel_timer m_oTimer;
	
    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;




    FILE *log_file; // debug file
public:
    /*! Default constructor. */
    OptIlm_SignalValueGenerator();

	tResult Construct() override;
	tResult StartStreaming() override;
	tResult StopStreaming() override;

private:

    /*!
     * Timer function which is called each cycle
     *
     * \return  A tVoid.
     */
	tVoid TimerFunc();

    /*!
     * Transmit new sample.
     *
     * \return  Standard Result Code.
     */
    tResult TransmitNewSample(float steering, float speed);


    long int timer_counter;
    int timer_index;
    int max_index;

    float *timer_array;
    float *steering_array;
    float *speed_array;

};


//ASCII Hex
#define SPACE     0x20
#define RETURN    0x0d		//CR
#define NEW_LINE  0x0a		//LF
#define COMMA     0x2c
