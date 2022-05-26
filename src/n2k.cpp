//============================================================================
// Name        : n2k.cpp
// Author      : Khalid
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C, Ansi-style
//============================================================================

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/sysinfo.h>

#include <net/if.h>
#include <netinet/ip.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string.h>
#include <pthread.h>
#include <map>
#include <list>
#include <algorithm>

/* include N2k lib */
#include "NMEA2000.h"
#include "N2kMessages.h"

/* inc json parser lib */
#include "frozen.h"

/************ PARSER SETTINGS ******************/
#define UDP_PORT 8002

/* threads */
pthread_t udpThreadHandle;
pthread_t keepAliveTimerThread;

/* network socket */
int udpSock;
struct sockaddr_in clientAddr;
socklen_t clientAddrkLen;
int connected;

/* daemon flag */
int runAsDaemon;
int debugMode;


struct subscriber {
	struct sockaddr_in ip;
	unsigned long lastKeepAliveTime;
};

std::list<subscriber> subs;

/* table to store pgn subscribers */
std::map<unsigned long, std::list<subscriber *>> pgn_subs;

void publishN2kMessage(unsigned long pgn, char *msg);
void parse_n2k(const tN2kMsg &N2kMsg);
void add_pgn_subscriber(unsigned long pgn, struct sockaddr_in *client);

class n2kCAN : public tNMEA2000 {
private:
	int can_sock;
public:
	n2kCAN() {}
	bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent=true)
	{
		struct can_frame frame;
//		printf("CAN send frame \n");

		frame.can_id = id;
		frame.can_dlc = len;
		memcpy(frame.data, buf, len);

		write(can_sock, &frame, sizeof(frame));

		return true;
	}
	bool CANOpen()
	{
		struct sockaddr_can addr;
		//	    	struct can_frame frame;
		struct ifreq ifr;

		const char *ifname = "can1";

		if((can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
			perror("Error while opening socket");
			return false;
		}

		strcpy(ifr.ifr_name, ifname);
		ioctl(can_sock, SIOCGIFINDEX, &ifr);

		addr.can_family  = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;

		printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

		if(bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("Error in socket bind");
			return false;
		}


		return true;
	}
	bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
	{
		struct can_frame frame;

		if (debugMode) printf("CANFrame: recv frame \n");

		if (recv(can_sock, &frame, sizeof(frame), 0) > 0)
		{
			id = frame.can_id;
			len = frame.can_dlc;
			memcpy(buf, frame.data, len);

			return true;
		}
		return false;
	}
};

// Application execution delay. Must be implemented by application.
void delay(uint32_t ms)
{
	usleep(ms * 1000);
    return;
}

// Current uptime in milliseconds. Must be implemented by application.
uint32_t millis()
{
    struct sysinfo s_info;
    sysinfo(&s_info);

	return s_info.uptime * 1000;
}


static void parse_pgns(const char *str, int len, void *client) {
  struct json_token t;
  int i;
  for (i = 0; json_scanf_array_elem(str, len, "", i, &t) > 0; i++) {
	  unsigned long pgn = atol(t.ptr);
	  add_pgn_subscriber(pgn, (struct sockaddr_in *)client);
  }
}


void add_pgn_subscriber(unsigned long pgn, struct sockaddr_in *client)
{
	bool clientExists = false;
	printf("sub request to PGN %lu \n", pgn);
	/* */
	std::list<subscriber>::iterator subs_it;

	for (subs_it = subs.begin(); subs_it != subs.end(); subs_it++) {
		struct subscriber &sub = *subs_it;
		if (memcmp(&sub.ip, client, sizeof(*client)) == 0)
		{
			struct subscriber *subPtr = &(*subs_it);

			/* check if subscriber already subbed to this PGN */
			std::list<subscriber *> &pgn_sub_list = pgn_subs[pgn];

			bool found = (std::find(pgn_sub_list.begin(), pgn_sub_list.end(), subPtr) != pgn_sub_list.end());
			if (!found) {
				printf("add_pgn_subscriber: client exists but not subbed, adding %lu\n", pgn);
				pgn_sub_list.push_back(subPtr);
			} else
				printf("add_pgn_subscriber: already subbed \n");

			if (debugMode)
				printf("Adding PGN to existing client %lu\n", pgn);
			clientExists = true;
			break;
		}
	}

	if (!clientExists) {
		/* client does not exists, create new sub and then add it to pgn list*/
		struct subscriber newsub;
		memcpy(&newsub, client, sizeof(*client));
		newsub.lastKeepAliveTime = millis();
		subs.push_back(newsub);

		/* add client to pgn sublist */
		printf("Addeding new sub to PGN %lu\n", pgn);
		std::list<subscriber>::iterator subs_it;
		for (subs_it = subs.begin(); subs_it != subs.end(); subs_it++) {
			struct subscriber &sub = *subs_it;

			if (memcmp(&sub.ip, client, sizeof(*client)) == 0)
			{
				struct subscriber *subPtr = &(*subs_it);
				std::list<subscriber *> &pgn_sub_list = pgn_subs[pgn];
				pgn_sub_list.push_back(subPtr);
				break;

			}
		}

	}
}

void _n2k_msg_handler(const tN2kMsg &N2kMsg)
{

	double _engSpeed;
	double _flevel;

	if (debugMode)
		printf("recvied PGN : %lu\n", N2kMsg.PGN);

	/* convert to json and publish */
	parse_n2k(N2kMsg);

}

void publishN2kMessage(unsigned long pgn, char *msg)
{
	if (debugMode)
		printf(msg);

	std::list<subscriber *> subs_list = pgn_subs[pgn];
	std::list<subscriber *>::iterator subs_it;
	size_t msg_len = strlen(msg);
	for (subs_it = subs_list.begin(); subs_it != subs_list.end(); subs_it++)
	{
		struct subscriber *sub = *subs_it;
		if (debugMode)
			printf("Send to %s\n", inet_ntoa(sub->ip.sin_addr));
		sendto(udpSock, msg, msg_len, 0, (struct sockaddr *)&sub->ip, sizeof(sub->ip));

	}

}


void parse_n2k(const tN2kMsg &N2kMsg)
{

	char buf[512];
	struct json_out json_buf = JSON_OUT_BUF(buf, 512);
	bool pgnParsed = false;

	switch (N2kMsg.PGN) {
	case 126992:
	{
	    unsigned char SID;
	    uint16_t SystemDate;
	    double SystemTime;
	    tN2kTimeSource TimeSource;
	    ParseN2kPGN126992(N2kMsg, SID, SystemDate, SystemTime, TimeSource);
	    json_printf(&json_buf, "{ pgn: 126992, values: { SID: %d, SystemDate: %d, SystemTime: %.10f} }\n", SID, SystemDate, SystemTime);
	    pgnParsed = true;
	}
	break;
	case 127245:
	{
	    double RudderPosition;
	    unsigned char Instance;
	    tN2kRudderDirectionOrder RudderDirectionOrder;
	    double AngleOrder;
	    ParseN2kPGN127245(N2kMsg, RudderPosition, Instance, RudderDirectionOrder, AngleOrder);
	    json_printf(&json_buf, "{ pgn: 127245, values: { RudderPosition: %.10f, Instance: %d, AngleOrder: %.10f} }\n", RudderPosition, Instance, AngleOrder);
	    pgnParsed = true;
	}
	break;
	case 127250:
	{
	    unsigned char SID;
	    double Heading;
	    double Deviation;
	    double Variation;
	    tN2kHeadingReference ref;
	    ParseN2kPGN127250(N2kMsg, SID, Heading, Deviation, Variation, ref);
	    json_printf(&json_buf, "{ pgn: 127250, values: { SID: %d, Heading: %.10f, Deviation: %.10f, Variation: %.10f, Ref: %d} }\n", SID, Heading, Deviation, Variation, ref);
	    pgnParsed = true;
	}
	break;
	case 127251:
	{
	    unsigned char SID;
	    double RateOfTurn;
	    ParseN2kPGN127251(N2kMsg, SID, RateOfTurn);
	    json_printf(&json_buf, "{ pgn: 127251, values: { SID: %d, RateOfTurn: %.10f} }\n", SID, RateOfTurn);
	    pgnParsed = true;
	}
	break;
	case 127257:
	{
	    unsigned char SID;
	    double Yaw;
	    double Pitch;
	    double Roll;
	    ParseN2kPGN127257(N2kMsg, SID, Yaw, Pitch, Roll);
	    json_printf(&json_buf, "{ pgn: 127257, values: { SID: %d, Yaw: %.10f, Pitch: %.10f, Roll: %.10f} }\n", SID, Yaw, Pitch, Roll);
	    pgnParsed = true;
	}
	break;
	case 127258:
	{
	    unsigned char SID;
	    tN2kMagneticVariation Source;
	    uint16_t DaysSince1970;
	    double Variation;
	    ParseN2kPGN127258(N2kMsg, SID, Source, DaysSince1970, Variation);
	    json_printf(&json_buf, "{ pgn: 127258, values: { SID: %d, DaysSince1970: %d, Variation: %.10f} }\n", SID, DaysSince1970, Variation);
	    pgnParsed = true;
	}
	break;
	case 127488:
	{
	    unsigned char EngineInstance;
	    double EngineSpeed;
	    double EngineBoostPressure;
	    int8_t EngineTiltTrim;
	    ParseN2kPGN127488(N2kMsg, EngineInstance, EngineSpeed, EngineBoostPressure, EngineTiltTrim);
	    json_printf(&json_buf, "{ pgn: 127488, values: { EngineInstance: %d, EngineSpeed: %.10f, EngineBoostPressure: %.10f, EngineTiltTrim: %d} }\n", EngineInstance, EngineSpeed, EngineBoostPressure, EngineTiltTrim);
	    pgnParsed = true;
	}
	break;
	case 127489:
	{
	    unsigned char EngineInstance;
	    double EngineOilPress;
	    double EngineOilTemp;
	    double EngineCoolantTemp;
	    double AltenatorVoltage;
	    double FuelRate;
	    double EngineHours;
	    double EngineCoolantPress;
	    double EngineFuelPress;
	    int8_t EngineLoad;
	    int8_t EngineTorque;
	    ParseN2kPGN127489(N2kMsg, EngineInstance, EngineOilPress, EngineOilTemp, EngineCoolantTemp, AltenatorVoltage, FuelRate, EngineHours, EngineCoolantPress, EngineFuelPress, EngineLoad, EngineTorque);
	    json_printf(&json_buf, "{ pgn: 127489, values: { EngineInstance: %d, EngineOilPress: %.10f, EngineOilTemp: %.10f, EngineCoolantTemp: %.10f, AltenatorVoltage: %.10f, FuelRate: %.10f, EngineHours: %.10f, EngineCoolantPress: %.10f, EngineFuelPress: %.10f, EngineLoad: %d, EngineTorque: %d} }\n", EngineInstance, EngineOilPress, EngineOilTemp, EngineCoolantTemp, AltenatorVoltage, FuelRate, EngineHours, EngineCoolantPress, EngineFuelPress, EngineLoad, EngineTorque);
	    pgnParsed = true;
	}
	break;
	case 127493:
	{
	    unsigned char EngineInstance;
	    tN2kTransmissionGear TransmissionGear;
	    double OilPressure;
	    double OilTemperature;
	    unsigned char DiscreteStatus1;
	    ParseN2kPGN127493(N2kMsg, EngineInstance, TransmissionGear, OilPressure, OilTemperature, DiscreteStatus1);
	    json_printf(&json_buf, "{ pgn: 127493, values: { EngineInstance: %d, OilPressure: %.10f, OilTemperature: %.10f, DiscreteStatus1: %d} }\n", EngineInstance, OilPressure, OilTemperature, DiscreteStatus1);
	    pgnParsed = true;
	}
	break;
	case 127497:
	{
	    unsigned char EngineInstance;
	    double TripFuelUsed;
	    double FuelRateAverage;
	    double FuelRateEconomy;
	    double InstantaneousFuelEconomy;
	    ParseN2kPGN127497(N2kMsg, EngineInstance, TripFuelUsed, FuelRateAverage, FuelRateEconomy, InstantaneousFuelEconomy);
	    json_printf(&json_buf, "{ pgn: 127497, values: { EngineInstance: %d, TripFuelUsed: %.10f, FuelRateAverage: %.10f, FuelRateEconomy: %.10f, InstantaneousFuelEconomy: %.10f} }\n", EngineInstance, TripFuelUsed, FuelRateAverage, FuelRateEconomy, InstantaneousFuelEconomy);
	    pgnParsed = true;
	}
	break;
	case 127501:
	{
	    unsigned char DeviceBankInstance;
	    tN2kOnOff Status1;
	    tN2kOnOff Status2;
	    tN2kOnOff Status3;
	    tN2kOnOff Status4;
	    ParseN2kPGN127501(N2kMsg, DeviceBankInstance, Status1, Status2, Status3, Status4);
	    json_printf(&json_buf, "{ pgn: 127501, values: { DeviceBankInstance: %d} }\n", DeviceBankInstance);
	    pgnParsed = true;
	}
	break;
	case 127505:
	{
	    unsigned char Instance;
	    tN2kFluidType FluidType;
	    double Level;
	    double Capacity;
	    ParseN2kPGN127505(N2kMsg, Instance, FluidType, Level, Capacity);
	    json_printf(&json_buf, "{ pgn: 127505, values: { Instance: %d, Level: %.10f, Capacity: %.10f} }\n", Instance, Level, Capacity);
	    pgnParsed = true;
	}
	break;
	case 127506:
	{
	    unsigned char SID;
	    unsigned char DCInstance;
	    tN2kDCType DCType;
	    uint8_t StateOfCharge;
	    uint8_t StateOfHealth;
	    double TimeRemaining;
	    double RippleVoltage;
	    ParseN2kPGN127506(N2kMsg, SID, DCInstance, DCType, StateOfCharge, StateOfHealth, TimeRemaining, RippleVoltage);
	    json_printf(&json_buf, "{ pgn: 127506, values: { SID: %d, DCInstance: %d, StateOfCharge: %d, StateOfHealth: %d, TimeRemaining: %.10f, RippleVoltage: %.10f} }\n", SID, DCInstance, StateOfCharge, StateOfHealth, TimeRemaining, RippleVoltage);
	    pgnParsed = true;
	}
	break;
	case 127507:
	{
	    unsigned char Instance;
	    unsigned char BatteryInstance;
	    tN2kChargeState ChargeState;
	    tN2kChargerMode ChargerMode;
	    tN2kOnOff Enabled;
	    tN2kOnOff EqualizationPending;
	    double EqualizationTimeRemaining;
	    ParseN2kPGN127507(N2kMsg, Instance, BatteryInstance, ChargeState, ChargerMode, Enabled, EqualizationPending, EqualizationTimeRemaining);
	    json_printf(&json_buf, "{ pgn: 127507, values: { Instance: %d, BatteryInstance: %d, EqualizationTimeRemaining: %.10f} }\n", Instance, BatteryInstance, EqualizationTimeRemaining);
	    pgnParsed = true;
	}
	break;
	case 127508:
	{
	    unsigned char BatteryInstance;
	    double BatteryVoltage;
	    double BatteryCurrent;
	    double BatteryTemperature;
	    unsigned char SID;
	    ParseN2kPGN127508(N2kMsg, BatteryInstance, BatteryVoltage, BatteryCurrent, BatteryTemperature, SID);
	    json_printf(&json_buf, "{ pgn: 127508, values: { BatteryInstance: %d, BatteryVoltage: %.10f, BatteryCurrent: %.10f, BatteryTemperature: %.10f, SID: %d} }\n", BatteryInstance, BatteryVoltage, BatteryCurrent, BatteryTemperature, SID);
	    pgnParsed = true;
	}
	break;
	case 127513:
	{
	    unsigned char BatInstance;
	    tN2kBatType BatType;
	    tN2kBatEqSupport SupportsEqual;
	    tN2kBatNomVolt BatNominalVoltage;
	    tN2kBatChem BatChemistry;
	    double BatCapacity;
	    int8_t BatTemperatureCoefficient;
	    double PeukertExponent;
	    int8_t ChargeEfficiencyFactor;
	    ParseN2kPGN127513(N2kMsg, BatInstance, BatType, SupportsEqual, BatNominalVoltage, BatChemistry, BatCapacity, BatTemperatureCoefficient, PeukertExponent, ChargeEfficiencyFactor);
	    json_printf(&json_buf, "{ pgn: 127513, values: { BatInstance: %d, BatCapacity: %.10f, BatTemperatureCoefficient: %d, PeukertExponent: %.10f, ChargeEfficiencyFactor: %d} }\n", BatInstance, BatCapacity, BatTemperatureCoefficient, PeukertExponent, ChargeEfficiencyFactor);
	    pgnParsed = true;
	}
	break;
	case 128000:
	{
	    unsigned char SID;
	    double Leeway;
	    ParseN2kPGN128000(N2kMsg, SID, Leeway);
	    json_printf(&json_buf, "{ pgn: 128000, values: { SID: %d, Leeway: %.10f} }\n", SID, Leeway);
	    pgnParsed = true;
	}
	break;
	case 128259:
	{
	    unsigned char SID;
	    double WaterReferenced;
	    double GroundReferenced;
	    tN2kSpeedWaterReferenceType SWRT;
	    ParseN2kPGN128259(N2kMsg, SID, WaterReferenced, GroundReferenced, SWRT);
	    json_printf(&json_buf, "{ pgn: 128259, values: { SID: %d, WaterReferenced: %.10f, GroundReferenced: %.10f} }\n", SID, WaterReferenced, GroundReferenced);
	    pgnParsed = true;
	}
	break;
	case 128267:
	{
	    unsigned char SID;
	    double DepthBelowTransducer;
	    double Offset;
	    double Range;
	    ParseN2kPGN128267(N2kMsg, SID, DepthBelowTransducer, Offset, Range);
	    json_printf(&json_buf, "{ pgn: 128267, values: { SID: %d, DepthBelowTransducer: %.10f, Offset: %.10f, Range: %.10f} }\n", SID, DepthBelowTransducer, Offset, Range);
	    pgnParsed = true;
	}
	break;
	case 128275:
	{
	    uint16_t DaysSince1970;
	    double SecondsSinceMidnight;
	    uint32_t Log;
	    uint32_t TripLog;
	    ParseN2kPGN128275(N2kMsg, DaysSince1970, SecondsSinceMidnight, Log, TripLog);
	    json_printf(&json_buf, "{ pgn: 128275, values: { DaysSince1970: %d, SecondsSinceMidnight: %.10f, Log: %d, TripLog: %d} }\n", DaysSince1970, SecondsSinceMidnight, Log, TripLog);
	    pgnParsed = true;
	}
	break;
	case 129025:
	{
	    double Latitude;
	    double Longitude;
	    ParseN2kPGN129025(N2kMsg, Latitude, Longitude);
	    json_printf(&json_buf, "{ pgn: 129025, values: { Latitude: %.10f, Longitude: %.10f} }\n", Latitude, Longitude);
	    pgnParsed = true;
	}
	break;
	case 129026:
	{
	    unsigned char SID;
	    tN2kHeadingReference ref;
	    double COG;
	    double SOG;
	    ParseN2kPGN129026(N2kMsg, SID, ref, COG, SOG);
	    json_printf(&json_buf, "{ pgn: 129026, values: { SID: %d, COG: %.10f, SOG: %.10f} }\n", SID, COG, SOG);
	    pgnParsed = true;
	}
	break;
	case 129028:
	{
	    unsigned char SID;
	    double TimeDelta;
	    uint8_t GNSSQuality;
	    uint8_t Direction;
	    double COG;
	    double AltitudeDelta;
	    ParseN2kPGN129028(N2kMsg, SID, TimeDelta, GNSSQuality, Direction, COG, AltitudeDelta);
	    json_printf(&json_buf, "{ pgn: 129028, values: { SID: %d, TimeDelta: %.10f, GNSSQuality: %d, Direction: %d, COG: %.10f, AltitudeDelta: %.10f} }\n", SID, TimeDelta, GNSSQuality, Direction, COG, AltitudeDelta);
	    pgnParsed = true;
	}
	break;
	case 129029:
	{
	    unsigned char SID;
	    uint16_t DaysSince1970;
	    double SecondsSinceMidnight;
	    double Latitude;
	    double Longitude;
	    double Altitude;
	    tN2kGNSStype GNSStype;
	    tN2kGNSSmethod GNSSmethod;
	    uint8_t nSatellites;
	    double HDOP;
	    double PDOP;
	    double GeoidalSeparation;
	    uint8_t nReferenceStations;
	    tN2kGNSStype ReferenceStationType;
	    uint16_t ReferenceSationID;
	    double AgeOfCorrection;
	    ParseN2kPGN129029(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight, Latitude, Longitude, Altitude, GNSStype, GNSSmethod, nSatellites, HDOP, PDOP, GeoidalSeparation, nReferenceStations, ReferenceStationType, ReferenceSationID, AgeOfCorrection);
	    json_printf(&json_buf, "{ pgn: 129029, values: { SID: %d, DaysSince1970: %d, SecondsSinceMidnight: %.10f, Latitude: %.10f, Longitude: %.10f, Altitude: %.10f, GNSStype: %d, GNSSmethod: %d, nSatellites: %d, HDOP: %.10f, PDOP: %.10f, GeoidalSeparation: %.10f, nReferenceStations: %d, ReferenceStationType: %d, ReferenceSationID: %d, AgeOfCorrection: %.10f} }\n", SID, DaysSince1970, SecondsSinceMidnight, Latitude, Longitude, Altitude, GNSStype, GNSSmethod, nSatellites, HDOP, PDOP, GeoidalSeparation, nReferenceStations, ReferenceStationType, ReferenceSationID, AgeOfCorrection);
	    pgnParsed = true;
	}
	break;
	case 129033:
	{
	    uint16_t DaysSince1970;
	    double SecondsSinceMidnight;
	    int16_t LocalOffset;
	    ParseN2kPGN129033(N2kMsg, DaysSince1970, SecondsSinceMidnight, LocalOffset);
	    json_printf(&json_buf, "{ pgn: 129033, values: { DaysSince1970: %d, SecondsSinceMidnight: %.10f, LocalOffset: %d} }\n", DaysSince1970, SecondsSinceMidnight, LocalOffset);
	    pgnParsed = true;
	}
	break;
	case 129539:
	{
	    unsigned char SID;
	    tN2kGNSSDOPmode DesiredMode;
	    tN2kGNSSDOPmode ActualMode;
	    double HDOP;
	    double VDOP;
	    double TDOP;
	    ParseN2kPGN129539(N2kMsg, SID, DesiredMode, ActualMode, HDOP, VDOP, TDOP);
	    json_printf(&json_buf, "{ pgn: 129539, values: { SID: %d, HDOP: %.10f, VDOP: %.10f, TDOP: %.10f} }\n", SID, HDOP, VDOP, TDOP);
	    pgnParsed = true;
	}
	break;
	case 129038:
	{
	    uint8_t MessageID;
	    tN2kAISRepeat Repeat;
	    uint32_t UserID;
	    double Latitude;
	    double Longitude;
	    bool Accuracy;
	    bool RAIM;
	    uint8_t Seconds;
	    double COG;
	    double SOG;
	    double Heading;
	    double ROT;
	    tN2kAISNavStatus NavStatus;
	    ParseN2kPGN129038(N2kMsg, MessageID, Repeat, UserID, Latitude, Longitude, Accuracy, RAIM, Seconds, COG, SOG, Heading, ROT, NavStatus);
	    json_printf(&json_buf, "{ pgn: 129038, values: { MessageID: %d, UserID: %d, Latitude: %.10f, Longitude: %.10f, Accuracy: %B, RAIM: %B, Seconds: %d, COG: %.10f, SOG: %.10f, Heading: %.10f, ROT: %.10f} }\n", MessageID, UserID, Latitude, Longitude, Accuracy, RAIM, Seconds, COG, SOG, Heading, ROT);
	    pgnParsed = true;
	}
	break;
	case 129039:
	{
	    uint8_t MessageID;
	    tN2kAISRepeat Repeat;
	    uint32_t UserID;
	    double Latitude;
	    double Longitude;
	    bool Accuracy;
	    bool RAIM;
	    uint8_t Seconds;
	    double COG;
	    double SOG;
	    double Heading;
	    tN2kAISUnit Unit;
	    bool Display;
	    bool DSC;
	    bool Band;
	    bool Msg22;
	    tN2kAISMode Mode;
	    bool State;
	    ParseN2kPGN129039(N2kMsg, MessageID, Repeat, UserID, Latitude, Longitude, Accuracy, RAIM, Seconds, COG, SOG, Heading, Unit, Display, DSC, Band, Msg22, Mode, State);
	    json_printf(&json_buf, "{ pgn: 129039, values: { MessageID: %d, UserID: %d, Latitude: %.10f, Longitude: %.10f, Accuracy: %B, RAIM: %B, Seconds: %d, COG: %.10f, SOG: %.10f, Heading: %.10f, Display: %B, DSC: %B, Band: %B, Msg22: %B, State: %B} }\n", MessageID, UserID, Latitude, Longitude, Accuracy, RAIM, Seconds, COG, SOG, Heading, Display, DSC, Band, Msg22, State);
	    pgnParsed = true;
	}
	break;
	case 129283:
	{
	    unsigned char SID;
	    tN2kXTEMode XTEMode;
	    bool NavigationTerminated;
	    double XTE;
	    ParseN2kPGN129283(N2kMsg, SID, XTEMode, NavigationTerminated, XTE);
	    json_printf(&json_buf, "{ pgn: 129283, values: { SID: %d, NavigationTerminated: %B, XTE: %.10f} }\n", SID, NavigationTerminated, XTE);
	    pgnParsed = true;
	}
	break;
	case 129284:
	{
	    unsigned char SID;
	    double DistanceToWaypoint;
	    tN2kHeadingReference BearingReference;
	    bool PerpendicularCrossed;
	    bool ArrivalCircleEntered;
	    tN2kDistanceCalculationType CalculationType;
	    double ETATime;
	    int16_t ETADate;
	    double BearingOriginToDestinationWaypoint;
	    double BearingPositionToDestinationWaypoint;
	    uint8_t OriginWaypointNumber;
	    uint8_t DestinationWaypointNumber;
	    double DestinationLatitude;
	    double DestinationLongitude;
	    double WaypointClosingVelocity;
	    ParseN2kPGN129284(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType, ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint, OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity);
	    json_printf(&json_buf, "{ pgn: 129284, values: { SID: %d, DistanceToWaypoint: %.10f, PerpendicularCrossed: %B, ArrivalCircleEntered: %B, ETATime: %.10f, ETADate: %d, BearingOriginToDestinationWaypoint: %.10f, BearingPositionToDestinationWaypoint: %.10f, OriginWaypointNumber: %d, DestinationWaypointNumber: %d, DestinationLatitude: %.10f, DestinationLongitude: %.10f, WaypointClosingVelocity: %.10f} }\n", SID, DistanceToWaypoint, PerpendicularCrossed, ArrivalCircleEntered, ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint, OriginWaypointNumber, DestinationWaypointNumber, DestinationLatitude, DestinationLongitude, WaypointClosingVelocity);
	    pgnParsed = true;
	}
	break;
	case 129794:
	{
	    uint8_t MessageID;
	    tN2kAISRepeat Repeat;
	    uint32_t UserID;
	    uint32_t IMOnumber;
	    char Callsign[56];
	    char Name[56];
	    uint8_t VesselType;
	    double Length;
	    double Beam;
	    double PosRefStbd;
	    double PosRefBow;
	    uint16_t ETAdate;
	    double ETAtime;
	    double Draught;
	    char Destination[56];
	    tN2kAISVersion AISversion;
	    tN2kGNSStype GNSStype;
	    tN2kAISDTE DTE;
	    tN2kAISTranceiverInfo AISinfo;
	    ParseN2kPGN129794(N2kMsg, MessageID, Repeat, UserID, IMOnumber, Callsign, Name, VesselType, Length, Beam, PosRefStbd, PosRefBow, ETAdate, ETAtime, Draught, Destination, AISversion, GNSStype, DTE, AISinfo);
	    json_printf(&json_buf, "{ pgn: 129794, values: { MessageID: %d, UserID: %d, IMOnumber: %d, Callsign: %Q, Name: %Q, VesselType: %d, Length: %.10f, Beam: %.10f, PosRefStbd: %.10f, PosRefBow: %.10f, ETAdate: %d, ETAtime: %.10f, Draught: %.10f, Destination: %Q, GNSStype: %d} }\n", MessageID, UserID, IMOnumber, Callsign, Name, VesselType, Length, Beam, PosRefStbd, PosRefBow, ETAdate, ETAtime, Draught, Destination, GNSStype);
	    pgnParsed = true;
	}
	break;
	case 129809:
	{
	    uint8_t MessageID;
	    tN2kAISRepeat Repeat;
	    uint32_t UserID;
	    char Name[56];
	    ParseN2kPGN129809(N2kMsg, MessageID, Repeat, UserID, Name);
	    json_printf(&json_buf, "{ pgn: 129809, values: { MessageID: %d, UserID: %d, Name: %Q} }\n", MessageID, UserID, Name);
	    pgnParsed = true;
	}
	break;
	case 129810:
	{
	    uint8_t MessageID;
	    tN2kAISRepeat Repeat;
	    uint32_t UserID;
	    uint8_t VesselType;
	    char Vendor[56];
	    char Callsign[56];
	    double Length;
	    double Beam;
	    double PosRefStbd;
	    double PosRefBow;
	    uint32_t MothershipID;
	    ParseN2kPGN129810(N2kMsg, MessageID, Repeat, UserID, VesselType, Vendor, Callsign, Length, Beam, PosRefStbd, PosRefBow, MothershipID);
	    json_printf(&json_buf, "{ pgn: 129810, values: { MessageID: %d, UserID: %d, VesselType: %d, Vendor: %Q, Callsign: %Q, Length: %.10f, Beam: %.10f, PosRefStbd: %.10f, PosRefBow: %.10f, MothershipID: %d} }\n", MessageID, UserID, VesselType, Vendor, Callsign, Length, Beam, PosRefStbd, PosRefBow, MothershipID);
	    pgnParsed = true;
	}
	break;
	case 130306:
	{
	    unsigned char SID;
	    double WindSpeed;
	    double WindAngle;
	    tN2kWindReference WindReference;
	    ParseN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference);
	    json_printf(&json_buf, "{ pgn: 130306, values: { SID: %d, WindSpeed: %.10f, WindAngle: %.10f} }\n", SID, WindSpeed, WindAngle);
	    pgnParsed = true;
	}
	break;
	case 130310:
	{
	    unsigned char SID;
	    double WaterTemperature;
	    double OutsideAmbientAirTemperature;
	    double AtmosphericPressure;
	    ParseN2kPGN130310(N2kMsg, SID, WaterTemperature, OutsideAmbientAirTemperature, AtmosphericPressure);
	    json_printf(&json_buf, "{ pgn: 130310, values: { SID: %d, WaterTemperature: %.10f, OutsideAmbientAirTemperature: %.10f, AtmosphericPressure: %.10f} }\n", SID, WaterTemperature, OutsideAmbientAirTemperature, AtmosphericPressure);
	    pgnParsed = true;
	}
	break;
	case 130311:
	{
	    unsigned char SID;
	    tN2kTempSource TempSource;
	    double Temperature;
	    tN2kHumiditySource HumiditySource;
	    double Humidity;
	    double AtmosphericPressure;
	    ParseN2kPGN130311(N2kMsg, SID, TempSource, Temperature, HumiditySource, Humidity, AtmosphericPressure);
	    json_printf(&json_buf, "{ pgn: 130311, values: { SID: %d, Temperature: %.10f, Humidity: %.10f, AtmosphericPressure: %.10f} }\n", SID, Temperature, Humidity, AtmosphericPressure);
	    pgnParsed = true;
	}
	break;
	case 130312:
	{
	    unsigned char SID;
	    unsigned char TempInstance;
	    tN2kTempSource TempSource;
	    double ActualTemperature;
	    double SetTemperature;
	    ParseN2kPGN130312(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature);
	    json_printf(&json_buf, "{ pgn: 130312, values: { SID: %d, TempInstance: %d, ActualTemperature: %.10f, SetTemperature: %.10f} }\n", SID, TempInstance, ActualTemperature, SetTemperature);
	    pgnParsed = true;
	}
	break;
	case 130313:
	{
	    unsigned char SID;
	    unsigned char HumidityInstance;
	    tN2kHumiditySource HumiditySource;
	    double ActualHumidity;
	    double SetHumidity;
	    ParseN2kPGN130313(N2kMsg, SID, HumidityInstance, HumiditySource, ActualHumidity, SetHumidity);
	    json_printf(&json_buf, "{ pgn: 130313, values: { SID: %d, HumidityInstance: %d, ActualHumidity: %.10f, SetHumidity: %.10f} }\n", SID, HumidityInstance, ActualHumidity, SetHumidity);
	    pgnParsed = true;
	}
	break;
	case 130314:
	{
	    unsigned char SID;
	    unsigned char PressureInstance;
	    tN2kPressureSource PressureSource;
	    double ActualPressure;
	    ParseN2kPGN130314(N2kMsg, SID, PressureInstance, PressureSource, ActualPressure);
	    json_printf(&json_buf, "{ pgn: 130314, values: { SID: %d, PressureInstance: %d, ActualPressure: %.10f} }\n", SID, PressureInstance, ActualPressure);
	    pgnParsed = true;
	}
	break;
	case 130316:
	{
	    unsigned char SID;
	    unsigned char TempInstance;
	    tN2kTempSource TempSource;
	    double ActualTemperature;
	    double SetTemperature;
	    ParseN2kPGN130316(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature);
	    json_printf(&json_buf, "{ pgn: 130316, values: { SID: %d, TempInstance: %d, ActualTemperature: %.10f, SetTemperature: %.10f} }\n", SID, TempInstance, ActualTemperature, SetTemperature);
	    pgnParsed = true;
	}
	break;
	case 130576:
	{
	    int8_t PortTrimTab;
	    int8_t StbdTrimTab;
	    ParseN2kPGN130576(N2kMsg, PortTrimTab, StbdTrimTab);
	    json_printf(&json_buf, "{ pgn: 130576, values: { PortTrimTab: %d, StbdTrimTab: %d} }\n", PortTrimTab, StbdTrimTab);
	    pgnParsed = true;
	}
	break;

	default:
	break;
	}

	if (pgnParsed) {
		/* publish PGN message to listeners */
		publishN2kMessage(N2kMsg.PGN, buf);

		/* reset json buf out for next message */
		json_buf.u.buf.len = 0;
	}

}

void *udp_thread(void *args)
{
	struct sockaddr_in sockAddr;
	socklen_t addrLen = sizeof(sockAddr);
	char recvBuffer[64];
	int recvLen;
	/* create udp socket */
	udpSock = socket(AF_INET, SOCK_DGRAM, 0);

	memset(&sockAddr, 0, sizeof(sockAddr));

	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(UDP_PORT);
	sockAddr.sin_addr.s_addr = INADDR_ANY;

	bind(udpSock, (struct sockaddr *)&sockAddr, addrLen);

	clientAddrkLen = sizeof(clientAddr);

	for (;;) {
		if ( (recvLen = recvfrom(udpSock, recvBuffer, 64, 0, (struct sockaddr *)&clientAddr, &clientAddrkLen)) > 0) {
			/* data received, attempt to parse */
			printf("UDP Data recevied %d \n", recvLen);
			unsigned long iVal = 0;
			if (json_scanf(recvBuffer, recvLen, "{subscribe: %M}", parse_pgns, &clientAddr) > 0)
			{
				printf("subscriber\n");

			} else if (json_scanf(recvBuffer, recvLen, "{keepalive: %lu}", &iVal) > 0) {
				/* keep connection alive */
				/* */
				std::list<subscriber>::iterator subs_it;

				for (subs_it = subs.begin(); subs_it != subs.end(); subs_it++) {
					struct subscriber &sub = *subs_it;
					if (memcmp(&sub.ip, &clientAddr, sizeof(clientAddr)) == 0)
					{
						sub.lastKeepAliveTime = millis();
						break;
					}
				}
			}
		}

	}

	return NULL;
}

void *keepalive_timer_thread(void *args)
{
	for (;;)
	{
		sleep(1);
	}
}

/* parse run-time options */
void parseArguments(int argc, char *argv[])
{
	int opt;
	while ( ( opt = getopt(argc, argv, "dD")) != -1)
	{
		switch (opt) {
		case 'd': runAsDaemon = 1; break;
		case 'D': debugMode = 1; break;
		default:
			printf("invalid argument...\n");
		}
	}
}

int main(int argc, char *argv[]) {

	n2kCAN n2k;

	printf("*********N2k Parser. Started*********\n");

	/* parse cmd-line arguments */
	parseArguments(argc, argv);

	if(runAsDaemon) {
		/* run as daemon process */
		printf("running as daemon process\n");
		daemon(0, 0);
	}

	n2k.Open();
    n2k.SetMode(tNMEA2000::N2km_ListenAndSend);
    n2k.SetMsgHandler(_n2k_msg_handler);

    pthread_create(&udpThreadHandle, NULL, udp_thread, NULL);
    pthread_create(&keepAliveTimerThread, NULL, keepalive_timer_thread, NULL);

    while (true) {

    	/* parse N2k messages */
    	n2k.ParseMessages();

    }

	return EXIT_SUCCESS;
}
