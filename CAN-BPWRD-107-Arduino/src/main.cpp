



/*
uncommenting this CAN_PRINT_TEST define will cause the program to print CAN message ID's and Data to
Serial2 and cease sending them to the CAN tranceivers. This lets you easily check that
everything before CAN transmission is working. 

It will also disable the critical sec in the node.spinsome scope of the main loop, so 
you may experience other bugs / crashes / problems etc.
*/
//#define CAN_PRINT_TEST



#include <Arduino.h>

#include "107-Arduino-CriticalSection.h"
#include "107-Arduino-Cyphal.h"


// ---------------- can interface ------------------ 
#include "SimpleCan.h"

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void init_CAN(void);

SimpleCan can1(0, 0);
SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);
FDCAN_TxHeaderTypeDef TxHeader;
//uint8_t TxData[8];


//temp flag and global test
bool new_rx = false;
uint32_t new_rx_id = 0;
uint32_t new_rx_len = 0;

//--------------------------------------------------



//---------------- cyphal parts --------------------
using namespace uavcan::node;

bool transmitCyphalFrame(CanardFrame const & frame);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), 
						node_heap.size(), 
						micros, 
						[] (CanardFrame const & frame) { return transmitCyphalFrame(frame); },
						3,
						cyphal::Node::DEFAULT_TX_QUEUE_SIZE,
						cyphal::Node::DEFAULT_RX_QUEUE_SIZE,
						CANARD_MTU_CAN_FD
						);



cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
(1*1000*1000UL /* = 1 sec in usecs. */);

void onHeartbeat_1_0_Received(Heartbeat_1_0 const & msg);
cyphal::Subscription heartbeat_subscription = node_hdl.create_subscription<Heartbeat_1_0>(onHeartbeat_1_0_Received);
//--------------------------------------------------


void setup() {

	

  	Serial2.begin(921600);

  	pinMode(PA9, OUTPUT);
	pinMode(PA10, OUTPUT);
	digitalWrite(PA9, LOW);
	digitalWrite(PA10, LOW);

	pinMode(PC6, OUTPUT);


	delay(100);
	init_CAN();
  
}



void loop() {

  {
	#ifndef CAN_PRINT_TEST
    CriticalSection crit_sec;
	#endif

    node_hdl.spinSome();
  }

  /* Publish the heartbeat once/second */
  static uint32_t prev = 0;
  static uint32_t ctr = 0;
  uint32_t now = millis();
  if(now - prev > 1000)
  {
	//Serial2.println("");
    //Serial2.print("heartbeat no#: ");
	
    prev = now;

    uavcan::node::Heartbeat_1_0 msg;
	ctr++;

	//Serial2.println(ctr);

    msg.uptime = now / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);
  }


//   if (new_rx == true)
//   {
// 	new_rx = false;
// 	Serial2.print("Received packet, id=0x");
// 	Serial2.print(new_rx_id, HEX);
// 	Serial2.print(", length=");
// 	Serial2.print(new_rx_len);
//   }

}



static void init_CAN()
{
	Serial2.println(can1.init(CanSpeed::Mbit1) == HAL_OK
					   ? "CAN: initialized."
					   : "CAN: error when initializing.");

	can1.activateNotification(&can1RxHandler);

	Serial2.println(can1.start() == HAL_OK
					   ? "CAN: started."
					   : "CAN: error when starting.");
}



int dlcToLength(uint32_t dlc)
{
	//Serial2.println("DLC");
	int length = dlc >> 16;
	if (length >= 13)
	{
		return 32 + (13 - length) * 16;
	}
	else if (length == 12)
	{
		return 24;
	}
	else if (length >= 9)
	{
		return 12 + (9 - length) * 4;
	}
	return length;
}



static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{

	CanardFrame frame;
	frame.extended_can_id = rxHeader.Identifier;
	frame.payload = rxData;
	frame.payload_size = CanardCANDLCToLength[rxHeader.DataLength];

	node_hdl.onCanFrameReceived(frame);
	digitalToggle(PC6);


	Serial2.print(rxHeader.Identifier, HEX);
	Serial2.print(" DLC: ");
	Serial2.println(rxHeader.DataLength);

	uint8_t data;
	for (int i = 0; i<frame.payload_size; i++)
	{
		data = *rxData;
		Serial2.print(data, HEX);
		Serial2.print(" ");
		rxData++;
	}

}

void onHeartbeat_1_0_Received(Heartbeat_1_0 const & msg)
{
	//digitalToggle(PC6);
}

bool transmitCyphalFrame(CanardFrame const & frame)
/*
This cannot have serial.prints in it, they make use of
interrupts on both tx and rx. The Critical Sections in this code
block interrupts. This function is is called within a Critical
Section. If there is a serial print in here, the program will 
crash / hang.

For (unsafe) testing, comment out the critical section in the
loop() coe just beofre the node_hdl.spinSome()
*/

{

	#ifdef CAN_PRINT_TEST
	
	
	//NOTE : SERIAL PRINTS WILL CRASH THE CRIT_SEC AS INTERRUPTS ARE NEEDED
	Serial2.print("Transmitting..   ID: ");
	Serial2.print(frame.extended_can_id, HEX);
	Serial2.print("  payload size: ");
	Serial2.print(frame.payload_size);
	Serial2.print("  DATA: ");
	const uint8_t * mydata;
	mydata = reinterpret_cast<uint8_t const *>(frame.payload);
	uint8_t data;
	for (int i = 0; i<frame.payload_size; i++)
	{
		data = *mydata;
		Serial2.print(data, HEX);
		Serial2.print(" ");
		mydata++;
	}
	Serial2.println("");
	return true;

	

	#else

	// TxHeader.Identifier = 0x321;
	TxHeader.Identifier = frame.extended_can_id;

	//TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.IdType = FDCAN_EXTENDED_ID;

	TxHeader.TxFrameType = FDCAN_DATA_FRAME;

	TxHeader.DataLength = CanardCANLengthToDLC[frame.payload_size];


	//TxHeader.DataLength = FDCAN_DLC_BYTES_7;
	// switch (frame.payload_size)
	// {
	// 	case 0:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	// 	break;

	// 	case 1:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_1;
	// 	break;

	// 	case 2:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	// 	break;

	// 	case 3:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_3;
	// 	break;

	// 	case 4:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_4;
	// 	break;

	// 	case 5:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_5;
	// 	break;
		
	// 	case 6:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_6;
	// 	break;

	// 	case 7:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_7;
	// 	break;

	// 	case 8:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	// 	break;

	// 	case 12:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_12;
	// 	break;

	// 	case 16:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_16;
	// 	break;

	// 	case 20:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_20;
	// 	break;

	// 	case 24:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_24;
	// 	break;

	// 	case 32:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_32;
	// 	break;

	// 	case 48:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_48;
	// 	break;

	// 	case 64:
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_64;
	// 	break;

	// 	default:
	// 	//error?
	// 	TxHeader.DataLength = FDCAN_DLC_BYTES_7;
	// 	break;
	// }
	
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;

	//TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.FDFormat = FDCAN_FD_CAN;

	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	// TxData[0] = press_count;
	// TxData[1] = 0xAD;

	// Serial2.print("CAN: sending message: ");
	// Serial2.print(TxData[0]);
	// Serial2.print(" ");
	// Serial2.print(TxData[1]);
	// Serial2.println(can1.addMessageToTxFifoQ(&TxHeader, TxData) == HAL_OK
	// 				   ? " [was ok.]"
	// 				   : " [failed.]");
	const uint8_t * mydata;
	mydata = reinterpret_cast<uint8_t const *>(frame.payload);

	//this is a bit of a hack and not very efficient, but it takes us from const uint8_t * to 
	//uint8_t, which we can then give to addMessageToTxFifoQ as a uint8_t *
	uint8_t data[64] = {};
	Serial2.print("tx: ");
	Serial2.println(frame.payload_size);
	for (int i = 0; i<frame.payload_size; i++)
	{
		data[i] = *mydata;
		mydata++;
	}


	//uint8_t * can_payload_nc;
	//can_payload_nc = can_payload;

	can1.addMessageToTxFifoQ(&TxHeader, data);

	return true;

	#endif



  
  
}
