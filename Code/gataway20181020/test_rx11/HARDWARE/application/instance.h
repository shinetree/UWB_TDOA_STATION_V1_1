/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "deca_types.h"
#include "deca_device_api.h"

/******************************************************************************************************************
********************* NOTES on DW (MP) features/options ***********************************************************
*******************************************************************************************************************/
#define DEEP_SLEEP (1) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode:
// when the Anchor is sending the range report back to the Tag, the Tag will enter sleep after a ranging exchange is finished
// once it receives a report or times out, before the next poll message is sent (before next ranging exchange is started).

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power


#define ANCTOANCTWR (1) //if set to 1 then anchor to anchor TWR will be done in the last slot
/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

#define SIG_RX_UNKNOWN			99		// Received an unknown frame

//DecaRTLS frame function codes
#define RTLS_DEMO_MSG_TAG_POLL              (0x81)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x70)          // Anchor response to poll
#define RTLS_DEMO_MSG_ANCH_POLL				(0x71)			// Anchor to anchor poll message
#define RTLS_DEMO_MSG_ANCH_RESP2            (0x72)          // Anchor response to poll from anchor
#define RTLS_DEMO_MSG_ANCH_FINAL            (0x73)          // Anchor final massage back to Anchor
#define RTLS_DEMO_MSG_TAG_FINAL             (0x82)          // Tag final massage back to Anchor


//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    2				// FunctionCode(1), Range Num (1)
#define ANCH_RESPONSE_MSG_LEN               8               // FunctionCode(1), Sleep Correction Time (2), Measured_TOF_Time(4), Range Num (1) (previous)
#define TAG_FINAL_MSG_LEN                   33              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
															// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Final_TxTime(5), Valid Response Mask (1)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127
//#define STANDARD_FRAME_SIZE         80

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

#define MAX_TAG_LIST_SIZE				(8)
#define MAX_ANCHOR_LIST_SIZE			(4) //this is limited to 4 in this application
#define NUM_EXPECTED_RESPONSES			(3) //e.g. MAX_ANCHOR_LIST_SIZE - 1
#define NUM_EXPECTED_RESPONSES_ANC		(1) //anchors A0, A1 and A2 are involved in anchor to anchor ranging
#define NUM_EXPECTED_RESPONSES_ANC0		(2) //anchor A0 expects response from A1 and A2

#define GATEWAY_ANCHOR_ADDR				(0x8000)
#define A1_ANCHOR_ADDR					(0x8001)
#define A2_ANCHOR_ADDR					(0x8002)
#define A3_ANCHOR_ADDR					(0x8003)


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT   	1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        		//which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               	0   //this signifies that the instance is still processing the current event

//application data message byte offsets
#define FCODE                               0               // Function code is 1st byte of messageData
#define PTXT                                2				// Poll TX time
#define RRXT0                               7				// A0 Response RX time
#define RRXT1                               12				// A1 Response RX time
#define RRXT2                               17				// A2 Response RX time
#define RRXT3                               22				// A3 Response RX time
#define FTXT                                27				// Final TX time
#define VRESP                               32				// Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
#define RES_TAG_SLP0                        1               // Response tag sleep correction LSB
#define RES_TAG_SLP1                        2               // Response tag sleep correction MSB
#define TOFR                                3				// ToF (n-1) 4 bytes
#define TOFRN								7				// range number 1 byte
#define POLL_RNUM                           1               // Poll message range number

//this it the delay used for configuring the receiver on delay (wait for response delay)
//NOTE: this RX_RESPONSE_TURNAROUND is dependent on the microprocessor and code optimisations
#define RX_RESPONSEX_TURNAROUND (50) //takes about 100 us for response to come back
#define RX_RESPONSE1_TURNAROUND (200) //takes about 200 us for the 1st response to come back (from A0)
#define RX_RESPONSE1_TURNAROUND_6M81 (300) //takes about 100 us for response to come back
#define RX_RESPONSE1_TURNAROUND_110K (300) //takes about 100 us for response to come back

//Tag will range to 3 or 4 anchors
//Each ranging exchange will consist of minimum of 3 messages (Poll, Response, Final)
//and a maximum of 6 messages (Poll, Response x 4, Final)
//Thus the ranging exchange will take either 28 ms for 110 kbps and 5 ms for 6.81 Mbps.
//NOTE: the above times are for 110k rate with 64 symb non-standard SFD and 1024 preamble length

typedef enum instanceModes{LISTENER, TAG, ANCHOR, ANCHOR_RNG, NUM_MODES} INST_MODE;

//Listener = in this mode, the instance only receives frames, does not respond
//Tag = Exchanges DecaRanging messages (Poll-Response-Final) with Anchor and enabling Anchor to calculate the range between the two instances
//Anchor = see above
//Anchor_Rng = the anchor (assumes a tag function) and ranges to another anchor - used in Anchor to Anchor TWR for auto positioning function

#define TOF_REPORT_NUL 0
#define TOF_REPORT_T2A 1
#define TOF_REPORT_A2A 2

#define INVALID_TOF (0xABCDFFFF)

typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1 - state in which the instance will enter sleep (if ranging finished) or proceed to transmit a message
    TA_TXPOLL_WAIT_SEND,        //2 - configuration and sending of Poll message
    TA_TXFINAL_WAIT_SEND,       //3 - configuration and sending of Final message
    TA_TXRESPONSE_WAIT_SEND,    //4 - a place holder - response is sent from call back
    TA_TX_WAIT_CONF,            //6 - confirmation of TX done message

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP_DONE,               //9
    TA_TXRESPONSE_SENT_POLLRX,    //10
    TA_TXRESPONSE_SENT_RESPRX,    //11
    TA_TXRESPONSE_SENT_TORX		  //12

} INST_STATES;


// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

typedef struct
{
    uint8 channelNumber ;       // valid range is 1 to 11
    uint8 preambleCode ;        // 00 = use NS code, 1 to 24 selects code
    uint8 pulseRepFreq ;        // NOMINAL_4M, NOMINAL_16M, or NOMINAL_64M
    uint8 dataRate ;            // DATA_RATE_1 (110K), DATA_RATE_2 (850K), DATA_RATE_3 (6M81)
    uint8 preambleLen ;         // values expected are 64, (128), (256), (512), 1024, (2048), and 4096
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO;  //!< SFD timeout value (in symbols) e.g. preamble length (128) + SFD(8) - PAC + some margin ~ 135us... DWT_SFDTOC_DEF; //default value
} instanceConfig_t ;

typedef struct
{
    uint16 slotPeriod ; //slot period (time for 1 tag to range to 4 anchors)
    uint16 numSlots ; // number of slots in one superframe (number of tags supported)
    uint16 sfPeriod ; // superframe period in ms

    uint16 pollSleepDly ; // the minimum SLEEP time should be FRAME PERIOD so that tags don't interfere
    uint16 replyDly ; //response delay time (Tag or Anchor when sending Final/Response messages respectively)
} sfConfig_t ;

/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

//size of the event queue, in this application there should be at most 2 unprocessed events,
//i.e. if there is a transmission with wait for response then the TX callback followed by RX callback could be executed
//in turn and the event queued up before the instance processed the TX event.
#define MAX_EVENT_NUMBER (6)

typedef struct
{
	uint8  type;			// event type - if 0 there is no event in the queue
	uint8  type_save;		// holds the event type - does not clear (used to show what event has been processed)
	uint8  type_pend;	    // set if there is a pending event
	uint16 rxLength ;		// length of RX data (does not apply to TX events)

	uint64 timeStamp ;		// last timestamp (Tx or Rx) - 40 bit DW1000 time

	uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits of the 40 bit DW1000 time
	uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits of the 40 bit DW1000 time

	uint32 uTimeStamp ;			  //32 bit system counter (ms) - STM32 tick time (at time of IRQ)

	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
	}msgu;

	//uint32 eventtime ;
	//uint32 eventtimeclr ;
	uint8 gotit;			//stores the instance function which processed the event (used for debug)
}event_data_t ;

// TX power and PG delay configuration structure
typedef struct {
                uint8 PGdelay;

                //TX POWER
                //31:24     BOOST_0.125ms_PWR
                //23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
                //15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
                //7:0       DEFAULT_PWR-TX_DATA_PWR
                uint32 txPwr[2]; //
}tx_struct;

typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)

    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state
    int done ;					//done with the current event/wait for next event to arrive

	//configuration structures
	dwt_config_t    configData ;	//DW1000 channel configuration
	dwt_txconfig_t  configTX ;		//DW1000 TX power configuration
	uint16			txAntennaDelay ; //DW1000 TX antenna delay
	uint16			rxAntennaDelay ; //DW1000 RX antenna delay
	uint32 			txPower ;		 //DW1000 TX power
	uint8 txPowerChanged ;			//power has been changed - update the register on next TWR exchange
	uint8 antennaDelayChanged;		//antenna delay has been changed - update the register on next TWR exchange

	uint16 instanceAddress16; //contains tag/anchor address

	//timeouts and delays
	int32 tagSleepTime_ms; //in milliseconds - defines the nominal Tag sleep time period
	int32 tagSleepRnd; //add an extra slot duration to sleep time to avoid collision before getting synced by anchor 0

	//this is the delay used for the delayed transmit
	uint64 pollTx2FinalTxDelay ; //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit)
	uint64 pollTx2FinalTxDelayAnc ; //this is delay from Poll Tx time to Final Tx time in DW1000 units (40-bit) for Anchor to Anchor ranging
	uint64 fixedReplyDelayAnc ;
	uint32 fixedReplyDelayAncP ;
	int ancRespRxDelay ;

	int fwtoTime_sy ;	//this is final message duration (longest out of ranging messages)
	int fwtoTimeAnc_sy ;
	uint32 delayedReplyTime;		// delayed reply time of ranging-init/response/final message

    uint32 rxTimeouts ;

    //message structure used for holding the data of the frame to transmit before it is written to the DW1000
	srd_msg_dsss msg_f ; // ranging message frame with 16-bit addresses

	//Tag function address/message configuration
	uint8   shortAdd_idx ;				// device's 16-bit address low byte (used as index into arrays [0 - 3])
	uint8   eui64[8];				// device's EUI 64-bit address
	uint16  psduLength ;			// used for storing the TX frame length
    uint8   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmission
	uint16  panID ;					// panid used in the frames

	//64 bit timestamps
	//union of TX timestamps
	union {
		uint64 txTimeStamp ;		   // last tx timestamp
		uint64 tagPollTxTime ;		   // tag's poll tx timestamp
	    uint64 anchorRespTxTime ;	   // anchor's reponse tx timestamp
	}txu;
	uint64 tagPollRxTime ;          // receive time of poll message


	//application control parameters
	uint8	wait4ack ;				// if this is set to DWT_RESPONSE_EXPECTED, then the receiver will turn on automatically after TX completion

    int8   responseTO ;
	uint8   instToSleep;			// if set the instance will go to sleep before sending the blink/poll message
	uint8	stopTimer;				// stop/disable an active timer
    uint8	instanceTimerEn;		// enable/start a timer
    uint32	instanceWakeTime;			//
    uint32  nextSleepPeriod;

	uint8	gotTO;					// got timeout event

	uint8   rxResponseMaskAnc;
	uint8   rxResponseMask;			// bit mask - bit 0 = received response from anchor ID = 0, bit 1 from anchor ID = 1 etc...
	uint8   rxResponseMaskReport;
	uint8	rangeNum;				// incremented for each sequence of ranges (each slot)
	uint8	rangeNumA[MAX_TAG_LIST_SIZE];				// array which holds last range number from each tag
	uint8	rangeNumAnc;			// incremented for each sequence of ranges (each slot) - anchor to anchor ranging
	uint8	rangeNumAAnc[MAX_ANCHOR_LIST_SIZE]; //array which holds last range number for each anchor
	uint16  sframePeriod;
	uint16  slotPeriod;
	uint16  numSlots;
	uint32  a0SlotTime;
	uint32  a1SlotTime;
	int32   tagSleepCorrection;
	int32   tagSleepCorrection2;

    //diagnostic counters/data, results and logging
    uint32 tof[MAX_TAG_LIST_SIZE]; //this is an array which holds last ToF from particular tag (ID 0-7)

    //this is an array which holds last ToF to each anchor it should
    uint32 tofArray[MAX_ANCHOR_LIST_SIZE]; //contain 4 ToF to 4 anchors all relating to same range number sequence

    uint32 tofAnc[MAX_ANCHOR_LIST_SIZE]; //this is an array which holds last ToFs from particular anchors (0, 0-1, 0-2, 1-2)

    //this is an array which holds last ToFs of the Anchor to Anchor ranging
    uint32 tofArrayAnc[MAX_ANCHOR_LIST_SIZE]; //it contains 3 ToFs relating to same range number sequence (0, 0-1, 0-2, 1-2)

    int txMsgCount; //number of transmitted messages
	int	rxMsgCount; //number of received messages
	int lateTX; //number of "LATE" TX events
	int lateRX; //number of "LATE" RX events

    int longTermRangeCount ; //total number of ranges

    int newRange;			//flag set when there is a new range to report TOF_REPORT_A2A or TOF_REPORT_T2A
    int newRangeAncAddress; //last 4 bytes of anchor address - used for printing/range output display
    int newRangeTagAddress; //last 4 bytes of tag address - used for printing/range output display
    int newRangeTime;

    uint8 gatewayAnchor ; //set to TRUE = 1 if anchor address == GATEWAY_ANCHOR_ADDR

	//event queue - used to store DW1000 events as they are processed by the dw_isr/callback functions
    event_data_t dwevent[MAX_EVENT_NUMBER]; //this holds any TX/RX events and associated message data
	event_data_t saved_dwevent; //holds an RX event while the ACK is being sent
    uint8 dweventIdxOut;
    uint8 dweventIdxIn;
	uint8 dweventPeek;
	uint8 monitor;
	uint32 timeofTx ;

	uint8 rxRespsIdx; //index into the array below (current tag (4bits)/seq number(4bits))
	int8 rxResps[256];

	int dwIDLE; //set to 1 when the RST goes high after wake up (it is set in process_dwRSTn_irq)

} instance_data_t ;

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------

// function to calculate and report the Time of Flight to the GUI/display
int reportTOF(int idx, uint32 tofx);
void clearDistTable(int idx);
void setTagDist(int tidx, int aidx);
double getTagDist(int idx);

// clear the status/ranging data 
void instanceclearcounts(void) ;

//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in driving/controlling the ranging application
//
//-------------------------------------------------------------------------------------------------------------

void instance_close(void);
// Call init, then call config, then call run. call close when finished
// initialise the instance (application) structures and DW1000 device
int instance_init(void);
// configure the instance and DW1000 device
void instance_config(instanceConfig_t *config, sfConfig_t *sfconfig) ;

// configure the MAC address
void instancesetaddresses(uint16 address) ;

void inst_processrxtimeout(instance_data_t *inst);

// called (periodically or from and interrupt) to process any outstanding TX/RX events and to drive the ranging application
int instance_run(void) ;       // returns indication of status report change
int testapprun(instance_data_t *inst, int message);

// calls the DW1000 interrupt handler
#define instance_process_irq(x) 	dwt_isr()  //call device interrupt handler
// configure TX/RX callback functions that are called from DW1000 ISR
void instance_rxcallback(const dwt_callback_data_t *rxd);
void instance_txcallback(const dwt_callback_data_t *txd);

// sets the Tag sleep delay time (the time Tag "sleeps" between each ranging attempt)
void instancesettagsleepdelay(int rangingsleep);
void instancesetreplydelay(int delayms);

// set/get the instance roles e.g. Tag/Anchor/Listener
void instancesetrole(int mode) ;                // 
int instancegetrole(void) ;
// get the DW1000 device ID (e.g. 0xDECA0130 for DW1000)
uint32 instancereaddeviceid(void) ;                                 // Return Device ID reg, enables validation of physical device presence

double instancegetidist(int idx);
double instancegetidistraw(int idx);
int instancegetidist_mm(int idx);
int instancegetidistraw_mm(int idx);
uint8 instancevalidranges(void);
void instancecleardisttableall(void);

void instance_backtoanchor(instance_data_t *inst);
int instancesenddlypacket(instance_data_t *inst, int delayedTx);

int instancegetrnum(void);
int instancegetrnuma(int idx);
int instancegetrnumanc(int idx);
int instancegetlcount(void);

int instancenewrangeancadd(void);
int instancenewrangetagadd(void);
int instancenewrangepolltim(void);
int instancenewrange(void);
int instancenewrangetim(void);

uint64 convertmicrosectodevicetimeu (double microsecu);
double convertdevicetimetosec(int32 dt);

#define DWT_PRF_64M_RFDLY   (514.462f)
#define DWT_PRF_16M_RFDLY   (513.9067f)
extern const uint16 rfDelays[2];
extern const uint16 rfDelaysTREK[2];
extern const tx_struct txSpectrumConfig[8];

extern instance_data_t instance_data[NUM_INST] ;


int instance_peekevent(void);

void instance_saveevent(event_data_t newevent, uint8 etype);

event_data_t instance_getsavedevent(void);

void instance_putevent(event_data_t newevent, uint8 etype);

event_data_t* instance_getevent(int x);

void instance_clearevents(void);

void instance_notify_DW1000_inIDLE(int idle);

// configure TX power
void instanceconfigtxpower(uint32 txpower);
void instancesettxpower(void);

// configure the antenna delays
void instanceconfigantennadelays(uint16 tx, uint16 rx);
void instancesetantennadelays(void);
uint16 instancetxantdly(void);
uint16 instancerxantdly(void);

int instance_starttxtest(int framePeriod);
#ifdef __cplusplus
}
#endif

#endif
