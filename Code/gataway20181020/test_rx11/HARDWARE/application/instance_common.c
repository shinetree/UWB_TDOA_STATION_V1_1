/*! ----------------------------------------------------------------------------
 *  @file    instance_common.c
 *  @brief   DecaWave application level common instance functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"

#include "instance.h"
#include "led.h"
// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------

double inst_tdist[MAX_TAG_LIST_SIZE] ;
double inst_idist[MAX_ANCHOR_LIST_SIZE] ;
double inst_idistraw[MAX_ANCHOR_LIST_SIZE] ;
instance_data_t instance_data[NUM_INST] ;

void ancprepareresponse(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame, uint32 uTimeStamp);
void ancprepareresponse2(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame);

//int eventOutcount = 0;
//int eventIncount = 0;

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------
// convert microseconds to device time
uint64 convertmicrosectodevicetimeu (double microsecu)
{
    uint64 dt;
    long double dtime;

    dtime = (microsecu / (double) DWT_TIME_UNITS) / 1e6 ;

    dt =  (uint64) (dtime) ;

    return dt;
}

double convertdevicetimetosec(int32 dt)
{
    double f = 0;

    f =  dt * DWT_TIME_UNITS ;  // seconds #define TIME_UNITS          (1.0/499.2e6/128.0) = 15.65e-12

    return f ;
}

 
int reportTOF(int idx, uint32 tofx)
{
        double distance ;
        double distance_to_correct;
        double tof ;
        int32 tofi ;

        // check for negative results and accept them making them proper negative integers
        tofi = (int32) tofx ; // make it signed
        if (tofi > 0x7FFFFFFF)  // close up TOF may be negative
        {
            tofi -= 0x80000000 ;  //
        }

        // convert to seconds (as floating point)
        tof = convertdevicetimetosec(tofi) ;          //this is divided by 4 to get single time of flight
        inst_idistraw[idx] = distance = tof * SPEED_OF_LIGHT;

#if (CORRECT_RANGE_BIAS == 1)
        //for the 6.81Mb data rate we assume gating gain of 6dB is used,
        //thus a different range bias needs to be applied
        //if(inst->configData.dataRate == DWT_BR_6M8)
        if(instance_data[0].configData.smartPowerEn)
        {
        	//1.31 for channel 2 and 1.51 for channel 5
        	if(instance_data[0].configData.chan == 5)
        	{
        		distance_to_correct = distance/1.51;
        	}
        	else //channel 2
        	{
        		distance_to_correct = distance/1.31;
			}
        }
        else
        {
        	distance_to_correct = distance;
        }

        distance = distance - dwt_getrangebias(instance_data[0].configData.chan, (float) distance_to_correct, instance_data[0].configData.prf);
#endif

        if ((distance < 0) || (distance > 20000.000))    // discard any results less than <0 cm or >20 km
            return 0;

        inst_idist[idx] = distance;

        instance_data[0].longTermRangeCount++ ;                          // for computing a long term average

    return 1;
}// end of reportTOF

void setTagDist(int tidx, int aidx)
{
	inst_tdist[tidx] = inst_idist[aidx];
}

double getTagDist(int idx)
{
	return inst_tdist[idx];
}

void clearDistTable(int idx)
{
	inst_idistraw[idx] = 0;
	inst_idist[idx] = 0;
}

void instancecleardisttableall(void)
{
	int i;

	for(i=0; i<MAX_ANCHOR_LIST_SIZE; i++)
	{
		inst_idistraw[i] = 0xffff;
		inst_idist[i] = 0xffff;
	}
}

// -------------------------------------------------------------------------------------------------------------------
#if NUM_INST != 1
#error These functions assume one instance only
#else


// -------------------------------------------------------------------------------------------------------------------
// Set this instance role as the Tag, Anchor or Listener
void instancesetrole(int inst_mode)
{
    // assume instance 0, for this
    instance_data[0].mode =  inst_mode;                   // set the role
}

int instancegetrole(void)
{
    return instance_data[0].mode;
}

int instancenewrange(void)
{
	int x = instance_data[0].newRange;
    instance_data[0].newRange = TOF_REPORT_NUL;
    return x;
}

int instancenewrangeancadd(void)
{
    return instance_data[0].newRangeAncAddress;
}

int instancenewrangetagadd(void)
{
    return instance_data[0].newRangeTagAddress;
}

int instancenewrangetim(void)
{
    return instance_data[0].newRangeTime;
}

// -------------------------------------------------------------------------------------------------------------------
// function to clear counts/averages/range values
//
void instanceclearcounts(void)
{
    int instance = 0 ;
    int i= 0 ;

    instance_data[instance].rxTimeouts = 0 ;

    dwt_configeventcounters(1); //enable and clear - NOTE: the counters are not preserved when in DEEP SLEEP

    instance_data[instance].frameSN = 0;

    instance_data[instance].longTermRangeCount  = 0;


    for(i=0; i<MAX_ANCHOR_LIST_SIZE; i++)
	{
    	instance_data[instance].tofArray[i] = INVALID_TOF;
	}

    for(i=0; i<MAX_TAG_LIST_SIZE; i++)
	{
		instance_data[instance].tof[i] = INVALID_TOF;
	}

} // end instanceclearcounts()


// -------------------------------------------------------------------------------------------------------------------
// function to initialise instance structures
//
// Returns 0 on success and -1 on error
int instance_init(void)
{
    int instance = 0 ;
    int i;
    int result;

    instance_data[instance].mode =  ANCHOR;                                // assume listener,
    instance_data[instance].testAppState = TA_INIT ;
    instance_data[instance].instToSleep = FALSE;


    // Reset the IC (might be needed if not getting here from POWER ON)
    // ARM code: Remove soft reset here as using hard reset in the inittestapplication() in the main.c file
    //dwt_softreset();

	//this initialises DW1000 and uses specified configurations from OTP/ROM
    result = dwt_initialise(DWT_LOADUCODE | DWT_LOADLDOTUNE | DWT_LOADTXCONFIG | DWT_LOADANTDLY| DWT_LOADXTALTRIM) ;

    //this is platform dependent - only program if DW EVK/EVB
    dwt_setleds(3) ; //configure the GPIOs which control the leds on EVBs

    if (DWT_SUCCESS != result)
    {
        return (-1) ;   // device initialise has failed
    }


    instanceclearcounts() ;

    instance_data[instance].panID = 0xdeca ;

    instance_data[instance].wait4ack = 0;
    instance_data[instance].stopTimer = 0;
    instance_data[instance].instanceTimerEn = 0;

    instance_clearevents();

    //dwt_geteui(instance_data[instance].eui64);
    memset(instance_data[instance].eui64, 0, ADDR_BYTE_SIZE_L);

    instance_data[instance].tagSleepCorrection = 0;

    dwt_setautorxreenable(0); //disable auto RX re-enable
    dwt_setdblrxbuffmode(0); //disable double RX buffer

    // if using auto CRC check (DWT_INT_RFCG and DWT_INT_RFCE) are used instead of DWT_INT_RDFR flag
    // other errors which need to be checked (as they disable receiver) are
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_SFDT | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);

    dwt_setcallbacks(instance_txcallback, instance_rxcallback);

    instance_data[instance].monitor = 0;

    instance_data[instance].lateTX = 0;
    instance_data[instance].lateRX = 0;

    instance_data[instance].responseTO = -1; //initialise
    for(i=0; i<256; i++)
    {
    	instance_data[instance].rxResps[i] = -10;
    }

    instance_data[instance].delayedReplyTime = 0;

    return 0 ;
}

// -------------------------------------------------------------------------------------------------------------------
//
// Return the Device ID register value, enables higher level validation of physical device presence
//

uint32 instancereaddeviceid(void)
{
    return dwt_readdevid() ;
}


// -------------------------------------------------------------------------------------------------------------------
//
// function to allow application configuration be passed into instance and affect underlying device operation
//
void instance_config(instanceConfig_t *config, sfConfig_t *sfConfig)
{
    int instance = 0 ;
    uint32 power = 0;
    uint8 otprev ;

    instance_data[instance].configData.chan = config->channelNumber ;
    instance_data[instance].configData.rxCode =  config->preambleCode ;
    instance_data[instance].configData.txCode = config->preambleCode ;
    instance_data[instance].configData.prf = config->pulseRepFreq ;
    instance_data[instance].configData.dataRate = config->dataRate ;
    instance_data[instance].configData.txPreambLength = config->preambleLen ;
    instance_data[instance].configData.rxPAC = config->pacSize ;
    instance_data[instance].configData.nsSFD = config->nsSFD ;
    instance_data[instance].configData.phrMode = DWT_PHRMODE_STD ;
    instance_data[instance].configData.sfdTO = config->sfdTO;

    //the DW1000 will automatically use gating gain for frames < 1ms duration (i.e. 6.81Mbps data rate)
    //smartPowerEn should be set based on the frame length, but we can also use dtaa rate.
    if(instance_data[instance].configData.dataRate == DWT_BR_6M8)
    {
        instance_data[instance].configData.smartPowerEn = 1;
    }
    else
    {
        instance_data[instance].configData.smartPowerEn = 0;
    }

    //configure the channel parameters
    dwt_configure(&instance_data[instance].configData, DWT_LOADXTALTRIM) ;

    instance_data[instance].configTX.PGdly = txSpectrumConfig[config->channelNumber].PGdelay ;

    //firstly check if there are calibrated TX power value in the DW1000 OTP
    power = dwt_getotptxpower(config->pulseRepFreq, instance_data[instance].configData.chan);

    if((power == 0x0) || (power == 0xFFFFFFFF)) //if there are no calibrated values... need to use defaults
    {
        power = txSpectrumConfig[config->channelNumber].txPwr[config->pulseRepFreq- DWT_PRF_16M];
    }

    //Configure TX power
    instance_data[instance].configTX.power = power;

    //configure the tx spectrum parameters (power and PG delay)
    dwt_configuretxrf(&instance_data[instance].configTX);

    otprev = dwt_otprevision() ;  // this revision tells us how OTP is programmed.

	if ((2 == otprev) || (3 == otprev))  // board is calibrated with TREK1000 with antenna delays set for each use case)
	{
		uint8 mode = (instance_data[instance].mode == ANCHOR ? 1 : 0);
		uint8 chanindex = 0;

		instance_data[instance].txAntennaDelay
										= dwt_getTREKOTPantennadelay(mode,
												instance_data[instance].configData.chan,
												instance_data[instance].configData.dataRate) ;

		// if nothing was actually programmed then set a reasonable value anyway
		if ((instance_data[instance].txAntennaDelay == 0)
				|| (instance_data[instance].txAntennaDelay == 0xffff))
		{
			if(instance_data[instance].configData.chan == 5)
			{
				chanindex = 1;
			}

			instance_data[instance].txAntennaDelay = rfDelaysTREK[chanindex];
		}

	}
	else // assume it is older EVK1000 programming.
	{
		//get the antenna delay that was read from the OTP calibration area
		instance_data[instance].txAntennaDelay = dwt_readantennadelay(config->pulseRepFreq) >> 1;

		// if nothing was actually programmed then set a reasonable value anyway
		if ((instance_data[instance].txAntennaDelay == 0)
				|| (instance_data[instance].txAntennaDelay == 0xffff))
		{
			instance_data[instance].txAntennaDelay = rfDelays[config->pulseRepFreq - DWT_PRF_16M];
		}
	}

	// -------------------------------------------------------------------------------------------------------------------
	// set the antenna delay, we assume that the RX is the same as TX.
	dwt_setrxantennadelay(instance_data[instance].txAntennaDelay);
	dwt_settxantennadelay(instance_data[instance].txAntennaDelay);

    instance_data[instance].rxAntennaDelay = instance_data[instance].txAntennaDelay;

    if(config->preambleLen == DWT_PLEN_64) //if preamble length is 64
	{
    	SPI_ConfigFastRate(SPI_BaudRatePrescaler_32); //reduce SPI to < 3MHz

		dwt_loadopsettabfromotp(0);

		SPI_ConfigFastRate(SPI_BaudRatePrescaler_4); //increase SPI to max
    }


	instancesettagsleepdelay(sfConfig->pollSleepDly); //set the Tag sleep time
	instance_data[instance].sframePeriod = sfConfig->sfPeriod;
	instance_data[instance].slotPeriod = sfConfig->slotPeriod;
	instance_data[instance].tagSleepRnd = sfConfig->slotPeriod;
	instance_data[instance].numSlots = sfConfig->numSlots;

	//last two slots are used for anchor to anchor ranging
	instance_data[instance].a0SlotTime = (instance_data[instance].numSlots-2) * instance_data[instance].slotPeriod;

	//set the default response delays
	instancesetreplydelay(sfConfig->replyDly);

}

// -------------------------------------------------------------------------------------------------------------------
// function to set the tag sleep time (in ms)
//
void instancesettagsleepdelay(int sleepdelay) //sleep in ms
{
    int instance = 0 ;
    instance_data[instance].tagSleepTime_ms = sleepdelay; //subtract the micro system delays (time it takes to switch states etc.)
}


int instancegetrnum(void) //get ranging number
{
	return instance_data[0].rangeNum;
}

int instancegetrnuma(int idx) //get ranging number
{
	return instance_data[0].rangeNumA[idx];
}

int instancegetrnumanc(int idx) //get ranging number
{
	return instance_data[0].rangeNumAAnc[idx];
}

int instancegetlcount(void) //get count of ranges used for calculation of lt avg
{
    int x = instance_data[0].longTermRangeCount;

    return (x);
}

double instancegetidist(int idx) //get instantaneous range
{
    double x ;

    idx &= (MAX_ANCHOR_LIST_SIZE - 1);

    x = inst_idist[idx];

    return (x);
}

double instancegetidistraw(int idx) //get instantaneous range (uncorrected)
{
    double x ;

    idx &= (MAX_ANCHOR_LIST_SIZE - 1);

    x = inst_idistraw[idx];

    return (x);
}

int instancegetidist_mm(int idx) //get instantaneous range
{
    int x ;

    idx &= (MAX_ANCHOR_LIST_SIZE - 1);

    x = (int)(inst_idist[idx]*1000);

    return (x);
}

int instancegetidistraw_mm(int idx) //get instantaneous range (uncorrected)
{
    int x ;

    idx &= (MAX_ANCHOR_LIST_SIZE - 1);

    x = (int)(inst_idistraw[idx]*1000);

    return (x);
}

void instance_backtoanchor(instance_data_t *inst)
{
	//stay in RX and behave as anchor
	inst->testAppState = TA_RXE_WAIT ;
	inst->mode = ANCHOR ;
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
	dwt_setrxaftertxdelay(0);
}


 
void inst_processrxtimeout(instance_data_t *inst)
{

	//inst->responseTimeouts ++ ;
    inst->rxTimeouts ++ ;
    inst->done = INST_NOT_DONE_YET;

    if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
    {
		//only enable receiver when not using double buffering
		inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
		dwt_setrxtimeout(0);

    }
	else if(inst->mode == TAG)
    {
		//if tag times out - no response (check if we are to send a final)
		//send the final only if it has received response from anchor 0
        if((inst->previousState == TA_TXPOLL_WAIT_SEND) && ((inst->rxResponseMask & 0x1) == 0))
        {
       		inst->instToSleep = TRUE ; //set sleep to TRUE so that tag will go to DEEP SLEEP before next ranging attempt
			inst->testAppState = TA_TXE_WAIT ;
			inst->nextState = TA_TXPOLL_WAIT_SEND ;
        }
        else if (inst->previousState == TA_TXFINAL_WAIT_SEND) //got here from main (error sending final - handle as timeout)
        {
        	dwt_forcetrxoff();	//this will clear all events
       		inst->instToSleep = TRUE ;
       		// initiate the re-transmission of the poll that was not responded to
			inst->testAppState = TA_TXE_WAIT ;
			inst->nextState = TA_TXPOLL_WAIT_SEND ;
        }
        else //send the final
        {
        	// initiate the re-transmission of the poll that was not responded to
			inst->testAppState = TA_TXE_WAIT ;
			inst->nextState = TA_TXFINAL_WAIT_SEND ;
		}

    }
	else //ANCHOR_RNG
	{
		//no Response form the other anchor
        if(
        	((inst->previousState == TA_TXPOLL_WAIT_SEND)
        		&& ((A1_ANCHOR_ADDR == inst->instanceAddress16) && ((inst->rxResponseMaskAnc & 0x4) == 0)))
				||
		    ((inst->previousState == TA_TXPOLL_WAIT_SEND)
				&& ((GATEWAY_ANCHOR_ADDR == inst->instanceAddress16) && ((inst->rxResponseMaskAnc & 0x2) == 0)))
		   )
        {
        	instance_backtoanchor(inst);
        }
        else if (inst->previousState == TA_TXFINAL_WAIT_SEND) //got here from main (error ending final - handle as timeout)
        {
        	instance_backtoanchor(inst);
        }
        else //send the final
        {
        	// initiate the re-transmission of the poll that was not responded to
			inst->testAppState = TA_TXE_WAIT ;
			inst->nextState = TA_TXFINAL_WAIT_SEND ;
		}
	}

    //timeout - disable the radio (if using SW timeout the rx will not be off)
    //dwt_forcetrxoff() ;
}

//
// NB: This function is called from the (TX) interrupt handler
//
 
void instance_txcallback(const dwt_callback_data_t *txd)
{
	int instance = 0;
	uint8 txTimeStamp[5] = {0, 0, 0, 0, 0};
	uint8 txevent = txd->event;
	event_data_t dw_event;

	dw_event.uTimeStamp = portGetTickCount();

	if(txevent == DWT_SIG_TX_DONE)
	{
		//uint64 txtimestamp = 0;

		//NOTE - we can only get TX good (done) while here
		//dwt_readtxtimestamp((uint8*) &instance_data[instance].txu.txTimeStamp);

		dwt_readtxtimestamp(txTimeStamp) ;
		dw_event.timeStamp32l = (uint32)txTimeStamp[0] + ((uint32)txTimeStamp[1] << 8) + ((uint32)txTimeStamp[2] << 16) + ((uint32)txTimeStamp[3] << 24);
		dw_event.timeStamp = txTimeStamp[4];
	    dw_event.timeStamp <<= 32;
		dw_event.timeStamp += dw_event.timeStamp32l;
		dw_event.timeStamp32h = ((uint32)txTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

		instance_data[instance].stopTimer = 0;

		dw_event.rxLength = instance_data[instance].psduLength;
		dw_event.type =  0;
		dw_event.type_pend =  0;
		dw_event.type_save = DWT_SIG_TX_DONE;

		memcpy((uint8 *)&dw_event.msgu.frame[0], (uint8 *)&instance_data[instance].msg_f, instance_data[instance].psduLength);

		instance_putevent(dw_event, DWT_SIG_TX_DONE);

		instance_data[instance].txMsgCount++;
	}
	else if(txevent == DWT_SIG_TX_AA_DONE)
	{
		//auto ACK confirmation
		dw_event.rxLength = 0;
		dw_event.type =  0;
		dw_event.type_save = DWT_SIG_TX_AA_DONE;

		instance_putevent(dw_event, DWT_SIG_TX_AA_DONE);

		//printf("TX AA time %f ecount %d\n",convertdevicetimetosecu(instance_data[instance].txu.txTimeStamp), instance_data[instance].dweventCnt);
	}

	instance_data[instance].monitor = 0;
}

/**
 * @brief function to re-enable the receiver and also adjust the timeout before sending the final message
 * if it is time so send the final message, the callback will notify the application, else the receiver is
 * automatically re-enabled
 *
 * this function is only used for tag when ranging to other anchors
 */
uint8 tagrxreenable(uint16 sourceAddress)
{
	uint8 type_pend = DWT_SIG_DW_IDLE;
	uint8 anc = sourceAddress & 0x3;
	int instance = 0;

	switch(anc)
	{
		//if we got Response from anchor 3 - this is the last expected response - send the final
		case 3:
			type_pend = DWT_SIG_DW_IDLE;
			break;

		//if we got Response from anchor 0, 1, or 2 - go back to wait for next anchor's response
		case 0:
		case 1:
		case 2:
		default:
			if(instance_data[instance].responseTO > 0) //can get here as result of error frame so need to check
			{
				dwt_setrxtimeout((uint16)instance_data[instance].fwtoTime_sy * instance_data[instance].responseTO); //reconfigure the timeout
				dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
				type_pend = DWT_SIG_RX_PENDING ;
			}
			else //last response was not received (got error/frame was corrupt)
			{
				type_pend = DWT_SIG_DW_IDLE; //report timeout - send the final
			}
			break;
	}

	return type_pend;
}

/**
 * @brief function to re-enable the receiver and also adjust the timeout before sending the final message
 * if it is time so send the final message, the callback will notify the application, else the receiver is
 * automatically re-enabled
 *
 * this function is only used for anchors (having a role of ANCHOR_RNG) when ranging to other anchors
 */
uint8 ancsendfinalorrxreenable(uint16 sourceAddress)
{
	uint8 type_pend = DWT_SIG_DW_IDLE;
	uint8 anc = sourceAddress & 0x3;
	int instance = 0;

	if(instance_data[instance].instanceAddress16 == GATEWAY_ANCHOR_ADDR)
	{
		switch(anc)
		{
			//if we got Response from anchor 1 - go back to wait for next anchor's response
			case 1:
				dwt_setrxtimeout((uint16)instance_data[instance].fwtoTime_sy); //reconfigure the timeout
				dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
				type_pend = DWT_SIG_RX_PENDING ;
				break;

			//if we got Response from anchor 2 - this is the last expected response - send the final
			case 2:
			default:
				type_pend = DWT_SIG_DW_IDLE;
				break;
		}
	}

	if(instance_data[instance].instanceAddress16 == A1_ANCHOR_ADDR)
	{
		switch(anc)
		{
			//if we got Response from anchor 2 - this is the last expected response - send the final
			case 2:
			default:
				type_pend = DWT_SIG_DW_IDLE;
				break;
		}
	}
	return type_pend;
}

/**
 * @brief this function either enables the receiver (delayed)
 *
 **/
void ancenablerx(void)
{
	int instance = 0;
	//subtract preamble length
	dwt_setdelayedtrxtime(instance_data[instance].delayedReplyTime - instance_data[instance].fixedReplyDelayAncP) ;
	if(dwt_rxenable(DWT_START_RX_DELAYED)) //delayed rx
	{
		//if the delayed RX failed - time has passed - do immediate enable
		//led_on(LED_PC9);
		dwt_setrxtimeout((uint16)instance_data[instance].fwtoTimeAnc_sy*2); //reconfigure the timeout before enable
		//longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_setrxtimeout((uint16)instance_data[instance].fwtoTimeAnc_sy); //restore the timeout for next RX enable
		instance_data[instance].lateRX++;
		//led_off(LED_PC9);
	}

}

/**
 * @brief this function either re-enables the receiver (delayed or immediate) or transmits the response frame
 *
 * @param the sourceAddress is the address of the sender of the current received frame
 * @param ancToAncTWR == 1 means that the anchor is ranging to another anchor, if == 0 then ranging to a tag
 *
 */
#pragma GCC optimize ("O0")
uint8 anctxorrxreenable(uint16 sourceAddress, int ancToAncTWR)
{
	uint8 type_pend = DWT_SIG_DW_IDLE;
	int sendResp = 0;
	int instance = 0;

	if(instance_data[instance].responseTO == 0) //go back to RX without TO - ranging has finished. (wait for Final but no TO)
	{
		dwt_setrxtimeout(0); //reconfigure the timeout
		dwt_setpreambledetecttimeout(0);
	}

	if((ancToAncTWR & 1) == 1)
	{
		if(instance_data[instance].responseTO == 1) //if one response left to receive (send a response now)
		{
			sendResp = 1;
		}
		//if A0 or A3 go back to RX as they do not send any responses when Anchor to Anchor ranging
		if((instance_data[instance].gatewayAnchor)
			|| (instance_data[instance].shortAdd_idx == 3)) //if this is anchor ID 3 do not reply to anchor poll
		{
			dwt_setrxtimeout(0);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			return DWT_SIG_RX_PENDING ;
		}
	}

	//configure delayed reply time (this is incremented for each received frame) it is timed from Poll rx time
	instance_data[instance].delayedReplyTime += (instance_data[instance].fixedReplyDelayAnc >> 8);

	//this checks if to send a frame
	if((((ancToAncTWR & 1) == 0) && ((instance_data[instance].responseTO + instance_data[instance].shortAdd_idx) == NUM_EXPECTED_RESPONSES)) //it's our turn to tx
		|| (sendResp == 1))
	{
		//led_on(LED_PC9);
		//response is expected
		instance_data[instance].wait4ack = DWT_RESPONSE_EXPECTED; //re has/will be re-enabled

		dwt_setdelayedtrxtime(instance_data[instance].delayedReplyTime) ;
		if(dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED))
		{
			//if TX has failed - we need to re-enable RX for the next response or final reception...
			dwt_setrxaftertxdelay(0);
			instance_data[instance].wait4ack = 0; //clear the flag as the TX has failed the TRX is off
			instance_data[instance].lateTX++;
			instance_data[instance].delayedReplyTime += 2*(instance_data[instance].fixedReplyDelayAnc >> 8); //to take into account W4R
			ancenablerx();
			type_pend = DWT_SIG_RX_PENDING ;
		}
		else
		{
			instance_data[instance].delayedReplyTime += (instance_data[instance].fixedReplyDelayAnc >> 8); //to take into account W4R
			type_pend = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
			instance_data[instance].timeofTx = portGetTickCnt();
			instance_data[instance].monitor = 1;
		}
		//led_off(LED_PC9);
	}
	else //stay in receive
	{
		if(sourceAddress == 0) //we got here after RX error, as we don't need to TX, we just enable RX
		{
			dwt_setrxtimeout(0);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		else
		{
			//led_on(LED_PC9);
			ancenablerx();
			//led_off(LED_PC9);
		}

		type_pend = DWT_SIG_RX_PENDING ;
	}
	//if time to send a response

	return type_pend;
}

/**
 * @brief this function handles frame error event, it will either signal TO or re-enable the receiver
 */
void handle_error_unknownframe(event_data_t dw_event)
{
	int instance = 0;
	//re-enable the receiver (after error frames as we are not using auto re-enable
	//for ranging application rx error frame is same as TO - as we are not going to get the expected frame
	if(instance_data[instance].mode == ANCHOR)
	{
		//if we are participating in the ranging (i.e. Poll was received)
		//and we get an rx error (in one of the responses)
		//need to consider this as a timeout as we could be sending our response next and
		//the applications needs to know to change the state
		//
		if(instance_data[instance].responseTO > 0)
		{
			instance_data[instance].responseTO--;

			//send a response or re-enable rx
			dw_event.type_pend = anctxorrxreenable(0, 0);
			dw_event.type = 0;
			dw_event.type_save = 0x40 | DWT_SIG_RX_TIMEOUT;
			dw_event.rxLength = 0;

			instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
		}
		else
		{
			dwt_setrxtimeout(0); //reconfigure the timeout
			dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
		}
	}
	else if(instance_data[instance].mode == LISTENER)
	{
		dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
	}
	else
	{
		instance_data[instance].responseTO--; //got something (need to reduce timeout (for remaining responses))

		dw_event.type_pend = tagrxreenable(0); //check if receiver will be re-enabled or it's time to send the final
		dw_event.type = 0;
		dw_event.type_save = 0x40 | DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;

		instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
	}
}


/**
 * @brief this function prepares and writes the anchor to anchor response frame into the TX buffer
 * it is called after anchor receives a Poll from an anchor
 */
void ancprepareresponse2(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame)
{
	uint16 frameLength = 0;
	uint8 tof_idx = (sourceAddress) & 0x3 ;
	int instance = 0;

	instance_data[instance].psduLength = frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
	//set the destination address (copy source as this is a reply)
	memcpy(&instance_data[instance].msg_f.destAddr[0], &frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
	instance_data[instance].msg_f.sourceAddr[0] = instance_data[instance].eui64[0];
	instance_data[instance].msg_f.sourceAddr[1] = instance_data[instance].eui64[1];
	// Write calculated TOF into response message (get the previous ToF+range number from that anchor)
	memcpy(&(instance_data[instance].msg_f.messageData[TOFR]), &instance_data[instance].tofAnc[tof_idx], 4);
	instance_data[instance].msg_f.messageData[TOFRN] = instance_data[instance].rangeNumAAnc[tof_idx]; //get the previous range number

	instance_data[instance].rangeNumAAnc[tof_idx] = 0; //clear the entry
	instance_data[instance].rangeNumAnc = frame[POLL_RNUM + fcode_index] ;
	instance_data[instance].msg_f.seqNum = instance_data[instance].frameSN++;

	//set the delayed rx on time (the final message will be sent after this delay)
	dwt_setrxaftertxdelay(instance_data[instance].ancRespRxDelay);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

	instance_data[instance].tagSleepCorrection = 0;
	instance_data[instance].msg_f.messageData[RES_TAG_SLP0] = 0 ;
	instance_data[instance].msg_f.messageData[RES_TAG_SLP1] = 0 ;

	instance_data[instance].msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP2; //message function code (specifies if message is a poll, response or other...)

	//write the TX data
	dwt_writetxfctrl(frameLength, 0);
	dwt_writetxdata(frameLength, (uint8 *)  &instance_data[instance].msg_f, 0) ;	// write the frame data

}

/**
 * @brief this function prepares and writes the anchor to tag response frame into the TX buffer
 * it is called after anchor receives a Poll from a tag
 */
void ancprepareresponse(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame, uint32 uTimeStamp)
{
	uint16 frameLength = 0;
	uint8 tof_idx = (sourceAddress) & 0x7 ;
	int instance = 0;

	instance_data[instance].psduLength = frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
	memcpy(&instance_data[instance].msg_f.destAddr[0], &frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
	instance_data[instance].msg_f.sourceAddr[0] = instance_data[instance].eui64[0];
	instance_data[instance].msg_f.sourceAddr[1] = instance_data[instance].eui64[1];
	// Write calculated TOF into response message (get the previous ToF+range number from that tag)
	memcpy(&(instance_data[instance].msg_f.messageData[TOFR]), &instance_data[instance].tof[tof_idx], 4);
	instance_data[instance].msg_f.messageData[TOFRN] = instance_data[instance].rangeNumA[tof_idx]; //get the previous range number

	instance_data[instance].rangeNumA[tof_idx] = 0; //clear after copy above...
	instance_data[instance].rangeNum = frame[POLL_RNUM+fcode_index] ;
	instance_data[instance].msg_f.seqNum = instance_data[instance].frameSN++;

	//we have our range - update the own mask entry...
	if(instance_data[instance].tof[tof_idx] != INVALID_TOF) //check the last ToF entry is valid and copy into the current array
	{
		instance_data[instance].rxResponseMask = (0x1 << instance_data[instance].shortAdd_idx);
		instance_data[instance].tofArray[instance_data[instance].shortAdd_idx] = instance_data[instance].tof[tof_idx];
	}
	else	//reset response mask
	{
		instance_data[instance].tofArray[instance_data[instance].shortAdd_idx] = INVALID_TOF ;
		instance_data[instance].rxResponseMask = 0;	//reset the mask of received responses when rx poll
	}
	//set the delayed rx on time (the final message will be sent after this delay)
	dwt_setrxaftertxdelay(instance_data[instance].ancRespRxDelay);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

	//if this is gateway anchor - calculate the slot period correction
	if(instance_data[instance].gatewayAnchor)
	{
		int error = 0;
		int currentSlotTime = 0;
		int expectedSlotTime = 0;
		//find the time in the current superframe
		currentSlotTime = uTimeStamp % instance_data[instance].sframePeriod;

		//this is the slot time the poll should be received in (Mask 0x07 for the 8 MAX tags we support in TREK)
		expectedSlotTime = (sourceAddress&0xFF) * instance_data[instance].slotPeriod; //

		//error = expectedSlotTime - currentSlotTime
		error = expectedSlotTime - currentSlotTime;

		if(error < (-(instance_data[instance].sframePeriod>>1))) //if error is more negative than 0.5 period, add whole period to give up to 1.5 period sleep
		{
			instance_data[instance].tagSleepCorrection = (instance_data[instance].sframePeriod + error);
		}
		else //the minimum Sleep time will be 0.5 period
		{
			instance_data[instance].tagSleepCorrection = error;
		}
		instance_data[instance].msg_f.messageData[RES_TAG_SLP0] = instance_data[instance].tagSleepCorrection & 0xFF ;
		instance_data[instance].msg_f.messageData[RES_TAG_SLP1] = (instance_data[instance].tagSleepCorrection >> 8) & 0xFF;
	}
	else
	{
		instance_data[instance].tagSleepCorrection = 0;
		instance_data[instance].msg_f.messageData[RES_TAG_SLP0] = 0 ;
		instance_data[instance].msg_f.messageData[RES_TAG_SLP1] = 0 ;
	}
	instance_data[instance].msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)

	//write the TX data
	dwt_writetxfctrl(frameLength, 0);
	dwt_writetxdata(frameLength, (uint8 *)  &instance_data[instance].msg_f, 0) ;	// write the frame data

}

/**
 * @brief this is the receive event callback handler, the received event is processed and the instance either
 * responds by sending a response frame or re-enables the receiver to await the next frame
 * once the immediate action is taken care of the event is queued up for application to process
 */
 
void instance_rxcallback(const dwt_callback_data_t *rxd)
{
	int instance = 0;
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	uint8 srcAddr_index = 0;
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCount();

    //if we got a frame with a good CRC - RX OK
    if(rxd->event == DWT_SIG_RX_OKAY)
	{
 		dw_event.rxLength = rxd->datalength;

		//need to process the frame control bytes to figure out what type of frame we have received
		if(((rxd->fctrl[0] == 0x41) || (rxd->fctrl[0] == 0x61))
				&&
				((rxd->fctrl[1] & 0xCC) == 0x88)) //short address
		{

			fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
			srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
			rxd_event = DWT_SIG_RX_OKAY;
		}
		else
		{
			rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
		}

        //read RX timestamp
        dwt_readrxtimestamp(rxTimeStamp) ;
        dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
    	dw_event.timeStamp32l =  (uint32)rxTimeStamp[0] + ((uint32)rxTimeStamp[1] << 8) + ((uint32)rxTimeStamp[2] << 16) + ((uint32)rxTimeStamp[3] << 24);
    	dw_event.timeStamp = rxTimeStamp[4];
    	dw_event.timeStamp <<= 32;
    	dw_event.timeStamp += dw_event.timeStamp32l;
    	dw_event.timeStamp32h = ((uint32)rxTimeStamp[4] << 24) + (dw_event.timeStamp32l >> 8);

        dw_event.type = 0; //type will be added as part of adding to event queue
		dw_event.type_save = rxd_event;
		dw_event.type_pend = DWT_SIG_DW_IDLE;

		//if Listener then just report the received frame to the instance (application)
		if(rxd_event == DWT_SIG_RX_OKAY) //Process good/known frame types
		{
			uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];

			if(instance_data[instance].mode != LISTENER)
			{
				if(instance_data[instance].mode == TAG) //if tag got a good frame - this is probably a response, but could also be some other non-ranging frame
					//(although due to frame filtering this is limited as non-addressed frames are filtered out)
				{
					instance_data[instance].responseTO--; //got 1 more response or other RX frame - need to reduce timeout (for next response)
				}

				//check if this is a TWR message (and also which one)
				switch(dw_event.msgu.frame[fcode_index])
				{
					//poll message from an anchor
					case RTLS_DEMO_MSG_ANCH_POLL:
					{
						//the anchor to anchor ranging poll frames are ignored by A0 and A3
						if(instance_data[instance].gatewayAnchor || (instance_data[instance].instanceAddress16 > A2_ANCHOR_ADDR))
						{
							//ignore poll from anchor 1 if gateway or anchor 3
							//anchors 2 and 3 will never send polls
							dw_event.type_pend = DWT_SIG_DW_IDLE ;
							break;
						}

						if(instance_data[instance].mode == TAG)  //tag should ignore any other Polls from anchors
						{
							instance_data[instance].responseTO++; //as will be decremented in the function and was also decremented above
							handle_error_unknownframe(dw_event);
							instance_data[instance].stopTimer = 1;
							instance_data[instance].rxMsgCount++;
							return;
						}

						//update the response index and number of responses received tables
						instance_data[instance].rxRespsIdx = (uint8) ((dw_event.msgu.frame[POLL_RNUM+fcode_index] & 0xf)
																+ (((sourceAddress&0x3) + 8) << 4));
						instance_data[instance].rxResps[instance_data[instance].rxRespsIdx] = 0;
						//debug LED on
						led_on(LED_PC9);

						//prepare the response and write it to the tx buffer
						ancprepareresponse2(sourceAddress, srcAddr_index, fcode_index, &dw_event.msgu.frame[0]);

						//A2 does not need timeout if ranging to A1
						if(sourceAddress != A1_ANCHOR_ADDR)
						{
							dwt_setrxtimeout((uint16)instance_data[instance].fwtoTimeAnc_sy); //reconfigure the timeout for response
						}

						//set the bast reply time (the actual will be Poll rx time + instance_data[0].fixedReplyDelayAnc)
						instance_data[instance].delayedReplyTime = dw_event.timeStamp32h ;
						instance_data[instance].responseTO = (instance_data[instance].instanceAddress16 - sourceAddress) & 0x3; //set number of expected responses

						dw_event.type_pend = anctxorrxreenable(instance_data[instance].instanceAddress16, 2+1);

						instance_data[instance].tofAnc[sourceAddress & 0x3] = INVALID_TOF; //clear ToF ..
						//debug LED off
						led_off(LED_PC9);
						break;
					}

					case RTLS_DEMO_MSG_TAG_POLL:
					{
						if(instance_data[instance].mode == TAG) //tag should ignore any other Polls from tags
						{
							instance_data[instance].responseTO++; //as will be decremented in the function and was also decremented above
							handle_error_unknownframe(dw_event);
							instance_data[instance].stopTimer = 1;
							instance_data[instance].rxMsgCount++;
							return;
						}
						instance_data[instance].rxRespsIdx = (int8) ((dw_event.msgu.frame[POLL_RNUM+fcode_index] & 0xf)
																+ ((sourceAddress&0x7) << 4));
						instance_data[instance].rxResps[instance_data[instance].rxRespsIdx] = 0;
						//debug LED on
						led_on(LED_PC9);
						//prepare the response and write it to the tx buffer
						ancprepareresponse(sourceAddress, srcAddr_index, fcode_index, &dw_event.msgu.frame[0], dw_event.uTimeStamp);

						dwt_setrxtimeout((uint16)instance_data[instance].fwtoTimeAnc_sy); //reconfigure the timeout for response

						instance_data[0].delayedReplyTime = dw_event.timeStamp32h /*+ (instance_data[0].fixedReplyDelayAnc >> 8)*/ ;
						instance_data[instance].responseTO = NUM_EXPECTED_RESPONSES; //set number of expected responses to 3 (from other anchors)

						dw_event.type_pend = anctxorrxreenable(instance_data[instance].instanceAddress16, 2+0);

						instance_data[instance].tof[sourceAddress & 0x7] = INVALID_TOF; //clear ToF ..
						//debug LED off
						led_off(LED_PC9);
					}
					break;

					//we got a response from a "responder" (anchor)
					case RTLS_DEMO_MSG_ANCH_RESP:
					case RTLS_DEMO_MSG_ANCH_RESP2:
					{
						//we are a tag
					    if(instance_data[instance].mode == TAG)
					    {
							uint8 index ;
							instance_data[instance].rxResps[instance_data[instance].rangeNum]++;
							dw_event.type_pend = tagrxreenable(sourceAddress); //responseTO decremented above...
							index = RRXT0 + 5*(sourceAddress & 0x3);

							instance_data[instance].rxResponseMask |= (0x1 << (sourceAddress & 0x3)); //add anchor ID to the mask
							// Write Response RX time field of Final message
							memcpy(&(instance_data[instance].msg_f.messageData[index]), rxTimeStamp, 5);

						}
						else if (instance_data[instance].mode == ANCHOR_RNG) //A0 and A1 only when ranging to other anchors
						{
							uint8 index ;
							instance_data[instance].rxResps[instance_data[instance].rangeNumAnc]++;
							dw_event.type_pend = ancsendfinalorrxreenable(sourceAddress);
							index = RRXT0 + 5*(sourceAddress & 0x3);

							instance_data[instance].rxResponseMaskAnc |= (0x1 << (sourceAddress & 0x3)); //add anchor ID to the mask
							// Write Response RX time field of Final message
							memcpy(&(instance_data[instance].msg_f.messageData[index]), rxTimeStamp, 5);
						}
						else //normal anchor mode
						{
							//got a response... (check if we got a Poll with the same range number as in this response)
							if(RTLS_DEMO_MSG_ANCH_RESP == dw_event.msgu.frame[fcode_index])
							{
								if((instance_data[instance].rxResps[instance_data[instance].rxRespsIdx] >= 0) //we got the poll else ignore this response
									&& (instance_data[instance].responseTO > 0)	) //if responseTO == 0 we have already received all of the responses - meaning should not be here => error
								{
									instance_data[instance].rxResps[instance_data[instance].rxRespsIdx]++; //increment the number of responses received
									instance_data[instance].responseTO--;

									//send a response or re-enable rx
									dw_event.type_pend = anctxorrxreenable(sourceAddress, 4+0);
								}
								else //like a timeout (error) ...
								{
									led_on(LED_PC9);
									//send a response or re-enable rx
									dwt_setrxtimeout(0); //reconfigure the timeout
									dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
									dw_event.type_pend = DWT_SIG_RX_PENDING ;
									led_off(LED_PC9);
								}
							}
							else //in anchor mode and got RTLS_DEMO_MSG_ANCH_RESP2
							{
								if((instance_data[instance].gatewayAnchor) &&
									(instance_data[instance].rxResps[instance_data[instance].rangeNumAnc]) == 2)
								{//got two responses A1 and A2 this is third (A2's to A1)
									instance_data[instance].rxResps[instance_data[instance].rangeNumAnc]++;
									instance_data[instance].rxResponseMaskAnc |= 0x8 ;

									dw_event.type_pend = anctxorrxreenable(sourceAddress, 4+1); //re-enable the RX
								}
								//A2 got A1's response to A0 - send A2 response (but only if we got the Poll from A0)
								else if((instance_data[instance].instanceAddress16 == A2_ANCHOR_ADDR) &&
										(instance_data[instance].rxResps[instance_data[instance].rxRespsIdx] >= 0) )
								{
									instance_data[instance].rxResps[instance_data[instance].rxRespsIdx]++;
									instance_data[instance].responseTO--;

									dwt_setrxtimeout(0);
									dwt_setrxaftertxdelay(0); //clear rx on delay as Final will come sooner than if we were waiting for next Response
									dw_event.type_pend = anctxorrxreenable(sourceAddress, 1);
								}
								else // if other anchor A1, A2, A3 .. ignore these responses when in ANCHOR mode
								{
									dwt_setrxtimeout(0); //reconfigure the timeout
									dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
									dw_event.type_pend = DWT_SIG_RX_PENDING ;
								}
							}
						}

					}
					break;

					case RTLS_DEMO_MSG_TAG_FINAL:
					case RTLS_DEMO_MSG_ANCH_FINAL:
						if(instance_data[instance].mode == TAG) //tag should ignore any other Final from anchors
						{
							instance_data[instance].responseTO++; //as will be decremented in the function and was also decremented above
							handle_error_unknownframe(dw_event);
							instance_data[instance].stopTimer = 1;
							instance_data[instance].rxMsgCount++;
							return;
						}
					//if anchor fall into case below and process the frame
					default:  //process rx frame
					{
						dw_event.type_pend = DWT_SIG_DW_IDLE;
					}
					break;

				}
			}//end of if not Listener mode
	    	instance_data[instance].stopTimer = 1;

            instance_putevent(dw_event, rxd_event);

			instance_data[instance].rxMsgCount++;
		}
		else //if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx (got unknown frame type)
		{
			handle_error_unknownframe(dw_event);
		}
	}
	else if (rxd->event == DWT_SIG_RX_TIMEOUT) //if tag and got TO, then did not get any or some responses - check if need to send final.
	{
		dw_event.type_pend = DWT_SIG_DW_IDLE;

		if(instance_data[instance].mode == ANCHOR)
		{
			//check if anchor has received all of the responses from other anchors (it could have received only 1 or 2)
			//it's timed out (re-enable rx or tx response)
			if(instance_data[instance].responseTO > 0)
			{
				instance_data[instance].responseTO--;
				//send a response or re-enable rx
				dw_event.type_pend = anctxorrxreenable(instance_data[instance].instanceAddress16, 6+0);
			}
		}
		dw_event.type = 0;
		dw_event.type_save = DWT_SIG_RX_TIMEOUT;
		dw_event.rxLength = 0;
		dw_event.timeStamp = 0;
		dw_event.timeStamp32l = 0;
		dw_event.timeStamp32h = 0;

		instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
		//printf("RX timeout while in %d\n", instance_data[instance].testAppState);
	}
	else //assume other events are errors
	{
		//led_on(LED_PC9);
		handle_error_unknownframe(dw_event);
		//led_off(LED_PC9);
	}
}

 
int instance_peekevent(void)
{
	int instance = 0;
    return instance_data[instance].dwevent[instance_data[instance].dweventPeek].type; //return the type of event that is in front of the queue
}

 
void instance_saveevent(event_data_t newevent, uint8 etype)
{
	int instance = 0;

	instance_data[instance].saved_dwevent = newevent;
	instance_data[instance].saved_dwevent.type = etype;
}

 
event_data_t instance_getsavedevent(void)
{
	int instance = 0;

	return instance_data[instance].saved_dwevent;
}

 
void instance_putevent(event_data_t newevent, uint8 etype)
{
	int instance = 0;
	//newevent.eventtime = portGetTickCount();
	newevent.gotit = 0 ; //newevent.eventtimeclr = 0;

	//copy event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn] = newevent;

	//set type - this makes it a new event (making sure the event data is copied before event is set as new)
	//to make sure that the get event function does not get an incomplete event
	instance_data[instance].dwevent[instance_data[instance].dweventIdxIn].type = etype;

	instance_data[instance].dweventIdxIn++;

	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxIn)
	{
		instance_data[instance].dweventIdxIn = 0;
	}
	//eventIncount++;

	//printf("put %d - in %d out %d @ %d\n", newevent.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);
}

event_data_t dw_event_g;

 
event_data_t* instance_getevent(int x)
{
	int instance = 0;
	int indexOut = instance_data[instance].dweventIdxOut;

	//dw_event_g = instance_data[instance].dwevent[instance_data[instance].dweventCntOut]; //this holds any TX/RX events

	//memcpy(&dw_event_g, &instance_data[instance].dwevent[instance_data[instance].dweventCntOut], sizeof(event_data_t));

	if(instance_data[instance].dwevent[indexOut].type == 0) //exit with "no event"
	{
		dw_event_g.type = 0;
		dw_event_g.type_save = 0;
		return &dw_event_g;
	}

	//copy the event
	dw_event_g.type_save = instance_data[instance].dwevent[indexOut].type_save ;
	dw_event_g.type_pend = instance_data[instance].dwevent[indexOut].type_pend ;
	dw_event_g.rxLength = instance_data[instance].dwevent[indexOut].rxLength ;
	dw_event_g.timeStamp = instance_data[instance].dwevent[indexOut].timeStamp ;
	dw_event_g.timeStamp32l = instance_data[instance].dwevent[indexOut].timeStamp32l ;
	dw_event_g.timeStamp32h = instance_data[instance].dwevent[indexOut].timeStamp32h ;
	dw_event_g.uTimeStamp = instance_data[instance].dwevent[indexOut].uTimeStamp ;
	//dw_event_g.eventtime = instance_data[instance].dwevent[indexOut].eventtime ;
	//dw_event_g.eventtimeclr = instance_data[instance].dwevent[indexOut].eventtimeclr ;
	//dw_event_g.gotit = instance_data[instance].dwevent[indexOut].gotit ;

	memcpy(&dw_event_g.msgu, &instance_data[instance].dwevent[indexOut].msgu, sizeof(instance_data[instance].dwevent[indexOut].msgu));

	dw_event_g.type = instance_data[instance].dwevent[indexOut].type ;


	instance_data[instance].dwevent[indexOut].gotit = x;

	//instance_data[instance].dwevent[indexOut].eventtimeclr = portGetTickCount();

	instance_data[instance].dwevent[indexOut].type = 0; //clear the event

	instance_data[instance].dweventIdxOut++;
	if(MAX_EVENT_NUMBER == instance_data[instance].dweventIdxOut) //wrap the counter
	{
		instance_data[instance].dweventIdxOut = 0;
	}
	instance_data[instance].dweventPeek = instance_data[instance].dweventIdxOut; //set the new peek value

	//if(dw_event.type) printf("get %d - in %d out %d @ %d\n", dw_event.type, instance_data[instance].dweventCntIn, instance_data[instance].dweventCntOut, ptime);

	//eventOutcount++;


	return &dw_event_g;
}

void instance_clearevents(void)
{
	int i = 0;
	int instance = 0;

	for(i=0; i<MAX_EVENT_NUMBER; i++)
	{
        memset(&instance_data[instance].dwevent[i], 0, sizeof(event_data_t));
	}

	instance_data[instance].dweventIdxIn = 0;
	instance_data[instance].dweventIdxOut = 0;
	instance_data[instance].dweventPeek = 0;

	//eventOutcount = 0;
	//eventIncount = 0;
}

// -------------------------------------------------------------------------------------------------------------------
 
int instance_run(void)
{
    int instance = 0 ;
    int done = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR


	while(done == INST_NOT_DONE_YET)
	{
		//int state = instance_data[instance].testAppState;
		done = testapprun(&instance_data[instance], message) ;                                               // run the communications application

		//we've processed message
		message = 0;
	}



    if(done == INST_DONE_WAIT_FOR_NEXT_EVENT_TO) //we are in RX and need to timeout (Tag needs to send another poll if no Rx frame)
    {
        if(instance_data[instance].mode == TAG) //Tag (is either in RX or sleeping)
        {
        	int32 nextPeriod ;

        	// next period will be a positive number because correction is -0.5 to +1.5 periods, (and tagSleepTime_ms is the period)
        	nextPeriod = instance_data[instance].tagSleepRnd + instance_data[instance].tagSleepTime_ms + instance_data[instance].tagSleepCorrection;

        	instance_data[instance].nextSleepPeriod = (uint32) nextPeriod ; //set timeout time, CAST the positive period to UINT for correct wrapping.
        	instance_data[instance].tagSleepCorrection2 = instance_data[instance].tagSleepCorrection;
        	instance_data[instance].tagSleepCorrection = 0; //clear the correction
            instance_data[instance].instanceTimerEn = 1; //start timer
        }
        instance_data[instance].stopTimer = 0 ; //clear the flag - timer can run if instancetimer_en set (set above)
        instance_data[instance].done = INST_NOT_DONE_YET;
    }

    //check if timer has expired
    if((instance_data[instance].instanceTimerEn == 1) && (instance_data[instance].stopTimer == 0))
    {
        if(instance_data[instance].mode == TAG)
        {
			if((portGetTickCount() - instance_data[instance].instanceWakeTime) > instance_data[instance].nextSleepPeriod)
			{
				event_data_t dw_event;
				instance_data[instance].instanceTimerEn = 0;
				dw_event.rxLength = 0;
				dw_event.type = 0;
				dw_event.type_save = 0x80 | DWT_SIG_RX_TIMEOUT;
				//printf("PC timeout DWT_SIG_RX_TIMEOUT\n");
				instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
			}
        }
#if (ANCTOANCTWR == 1) //allow anchor to anchor ranging
        else if(instance_data[instance].mode == ANCHOR)
        {
        	uint32_t slotTime = portGetTickCount() % instance_data[instance].sframePeriod;

        	if(instance_data[instance].gatewayAnchor)
        	{
				//if we are in the last slot - then A0 ranges to A1 and A2
				if( slotTime >= instance_data[instance].a0SlotTime)
				{
					port_DisableEXT_IRQ(); //enable ScenSor IRQ before starting
					//anchor0 sends poll to anchor1
					instance_data[instance].mode = ANCHOR_RNG; //change to ranging initiator
					dwt_forcetrxoff(); //disable DW1000
					instance_clearevents(); //clear any events
					//change state to send a Poll
					instance_data[instance].testAppState = TA_TXPOLL_WAIT_SEND ;
					instance_data[instance].msg_f.destAddr[0] = 0x1 ;
					instance_data[instance].msg_f.destAddr[1] = (GATEWAY_ANCHOR_ADDR >> 8);
					instance_data[instance].instanceTimerEn = 0;
					instance_data[instance].rangeNumAnc++;
					port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting
				}
        	}
			else if (instance_data[instance].instanceAddress16 == A1_ANCHOR_ADDR) //A1 ranges to A2 in the 2nd half of last slot
			{
				if(portGetTickCount() >= instance_data[instance].a1SlotTime)
				{
					port_DisableEXT_IRQ(); //enable ScenSor IRQ before starting
					//anchor1 sends poll to anchor2
					instance_data[instance].mode = ANCHOR_RNG; //change to ranging initiator
					dwt_forcetrxoff(); //disable DW1000
					instance_clearevents(); //clear any events
					//change state to send a Poll
					instance_data[instance].testAppState = TA_TXPOLL_WAIT_SEND ;
					instance_data[instance].msg_f.destAddr[0] = 0x2 ;
					instance_data[instance].msg_f.destAddr[1] = (GATEWAY_ANCHOR_ADDR >> 8);

					instance_data[instance].instanceTimerEn = 0;
					//instance_data[instance].a1SlotTime = 0;
					port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting
				}
			}
		}
#endif
    }

#if (ANCTOANCTWR == 1) //allow anchor to anchor ranging
    else if (instance_data[instance].instanceTimerEn == 0)
    {
        if((instance_data[instance].mode == ANCHOR) && (instance_data[instance].gatewayAnchor))
        {
        	uint32_t slotTime = portGetTickCount() % instance_data[instance].sframePeriod;
        	//enable the timer in 1st slot
        	if(slotTime < instance_data[instance].slotPeriod)
        	{
        		instance_data[instance].instanceTimerEn = 1;
        	}
        }
    }
#endif
    return 0 ;
}


void instance_close(void)
{
    //wake up device from low power mode
    //NOTE - in the ARM  code just drop chip select for 200us
    port_SPIx_clear_chip_select();  //CS low
    Sleep(1);   //200 us to wake up then waits 5ms for DW1000 XTAL to stabilise
    port_SPIx_set_chip_select();  //CS high
    Sleep(5);
    dwt_entersleepaftertx(0); // clear the "enter deep sleep after tx" bit

    dwt_setinterrupt(0xFFFFFFFF, 0); //don't allow any interrupts

}


void instance_notify_DW1000_inIDLE(int idle)
{
	instance_data[0].dwIDLE = idle;
}

void instanceconfigtxpower(uint32 txpower)
{
	instance_data[0].txPower = txpower ;

	instance_data[0].txPowerChanged = 1;

}

void instancesettxpower(void)
{
	if(instance_data[0].txPowerChanged == 1)
	{
	    //Configure TX power
	    dwt_write32bitreg(0x1E, instance_data[0].txPower);

		instance_data[0].txPowerChanged = 0;
	}
}

void instanceconfigantennadelays(uint16 tx, uint16 rx)
{
	instance_data[0].txAntennaDelay = tx ;
	instance_data[0].rxAntennaDelay = rx ;

	instance_data[0].antennaDelayChanged = 1;
}

void instancesetantennadelays(void)
{
	if(instance_data[0].antennaDelayChanged == 1)
	{
		dwt_setrxantennadelay(instance_data[0].rxAntennaDelay);
		dwt_settxantennadelay(instance_data[0].txAntennaDelay);

		instance_data[0].antennaDelayChanged = 0;
	}
}


uint16 instancetxantdly(void)
{
	return instance_data[0].txAntennaDelay;
}

uint16 instancerxantdly(void)
{
	return instance_data[0].rxAntennaDelay;
}

uint8 instancevalidranges(void)
{
	uint8 x = instance_data[0].rxResponseMaskReport;
	instance_data[0].rxResponseMaskReport = 0; //reset mask as we have printed out the ToFs
	return x;
}
#endif


/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
