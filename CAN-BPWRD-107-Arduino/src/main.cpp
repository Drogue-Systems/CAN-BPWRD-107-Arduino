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
uint8_t TxData[8];

//--------------------------------------------------



//---------------- cyphal parts --------------------
using namespace uavcan::node;

bool transmitCyphalFrame(CanardFrame const & frame);

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return transmitCyphalFrame(frame); });

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
(1*1000*1000UL /* = 1 sec in usecs. */);
//--------------------------------------------------


void setup() {
  Serial2.begin(115200);

  	pinMode(PA9, OUTPUT);
	pinMode(PA10, OUTPUT);
	digitalWrite(PA9, HIGH);
	digitalWrite(PA10, HIGH);

	// //pinMode(LED_BUILTIN, OUTPUT);
	// //attachInterrupt(digitalPinToInterrupt(BUTTON), Button_Down, LOW);

	delay(100);
	init_CAN();
  
}



void loop() {
  // put your main code here, to run repeatedly:
  {
    //CriticalSection crit_sec; //SEE COMMENT IN transmitCyphalFrame() FUCNTION!!!!
    node_hdl.spinSome();
  }

  /* Publish the heartbeat once/second */
  static uint32_t prev = 0;
  static uint32_t ctr = 0;
  uint32_t now = millis();
  if(now - prev > 2000)
  {
	Serial2.println("");
    Serial2.print("heartbeat no#: ");
	
    prev = now;

    uavcan::node::Heartbeat_1_0 msg;
	ctr++;

	Serial2.println(ctr);

    msg.uptime = now / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);
  }

}

// void loop()
// {
// 	{
// 		CriticalSection crit_sec;
		
// 		static uint32_t prev = 0;
// 		static uint32_t ctr = 0;
// 		uint32_t now = millis();
// 		if(now - prev > 1000)
// 		{
// 			//Serial2.println("inside critsec?");
// 		}
// 	}

// 	static uint32_t prev = 0;
// 	static uint32_t ctr = 0;
// 	uint32_t now = millis();
// 	if(now - prev > 1000)
// 	{
// 	prev = now;
// 	Serial2.print("Working? Now: ");
// 	Serial2.println(now);
// 	}

// }



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
	Serial2.println("DLC");
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
	int byte_length = dlcToLength(rxHeader.DataLength);

	Serial2.print("Received packet, id=0x");
	Serial2.print(rxHeader.Identifier, HEX);
	Serial2.print(", length=");
	Serial2.print(byte_length);
	for (int byte_index = 0; byte_index < byte_length; byte_index++)
	{
		Serial2.print(" byte[");
		Serial2.print(byte_index);
		Serial2.print("]=");
		Serial2.print(rxData[byte_index]);
		Serial2.print(" ");
	}
	Serial2.println();

	//digitalToggle(LED_BUILTIN);
}


bool transmitCyphalFrame(CanardFrame const & frame)
/*
This cannot have serial.prints in it, they make use of
interrupts on both tx and rx. The Critical Sections in this code
block interrupts. This functiins is called within a Critical
Section. If there is a serial print in here, the program will 
crash / hang.

For (unsafe) testing, comment out the critical section in the
loop() coe just beofre the node_hdl.spinSome()
*/

{
  bool test = true;
  if (test)
  {
	//NOTE : SERIAL PRINTS WILL CRASH THE CRIT_SEC AS INTERRUPTS ARE NEEDED
    Serial2.print("Transmitting..     ID: ");
	Serial2.print(frame.extended_can_id, HEX);
	// Serial2.print("   payload size: ");
	// Serial2.println(frame.payload_size);
	Serial2.print("     DATA: ");
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
  }

  
  
}