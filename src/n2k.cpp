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

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string.h>
#include <pthread.h>

/* include N2k lib */
#include "NMEA2000.h"
#include "N2kMessages.h"

/* inc json parser lib */
#include "frozen.h"

/************ PARSER SETTINGS ******************/
#define UDP_PORT 8002

/* global variables to store received N2K packets's data */

/* PGN 127488 */
uint8_t engInstance;
double engSpeed;
double engPressure;
int8_t engTrim;

/* PGN 127505 */
unsigned char instance;
enum tN2kFluidType fluildType;
double level;
double capacity;

/* PGN 127506 */
unsigned char SID127506;
unsigned char DCInstance;
enum tN2kDCType DCType;
uint8_t StateOfCharge;
uint8_t StateOfHealth;
double TimeRemaining;
double RippleVoltage;

/* PGN 127508 */
unsigned char BatteryInstance;
double BatteryVoltage;
double BatteryCurrent;
double BatteryTemperature;
unsigned char SID127508;

/* PGN 130311 */
unsigned char SID;
unsigned char humidityInstance;
unsigned char tempInstance;
double envHumidity;
double envTemperature;
double envAtp;
tN2kTempSource tempSource;
tN2kHumiditySource humiditySource;


/* threads */
pthread_t udpThreadHandle;

/* network socket */
int udpSock;
struct sockaddr_in clientAddr;
socklen_t clientAddrkLen;
int connected;

/* daemon flag */
int runAsDaemon;
int debugMode;

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

		const char *ifname = "can0";

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
		printf("recving messages \n");
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

void _n2k_msg_handler(const tN2kMsg &N2kMsg)
{

	double _engSpeed;
	double _flevel;

	if (debugMode)
		printf("recvied PGN : %lu\n", N2kMsg.PGN);
	/* Engine data */
    if (N2kMsg.PGN == 127488) {


        if (ParseN2kPGN127488(N2kMsg, engInstance, _engSpeed, engPressure, engTrim)) {
        	if (engInstance == 0)
        		engSpeed = _engSpeed;
        }
    }

    /* fuel level */
    if (N2kMsg.PGN == 127505) {

    	if (ParseN2kPGN127505(N2kMsg, instance, fluildType, _flevel, capacity)) {
    		//
    		// update
    		//
    		if (instance == 0)
    			level = _flevel;
    	}
    }

    /* battery status : DC Detailed */
    if (N2kMsg.PGN == 127506) {

    	if (ParseN2kPGN127506(N2kMsg, SID127506, DCInstance, DCType, StateOfCharge, StateOfHealth, TimeRemaining, RippleVoltage)) {
    		//
    		// update
    		//
    	}

    }

    /* battery status : battery voltage */
    if (N2kMsg.PGN == 127508) {

    	if (ParseN2kPGN127508(N2kMsg, BatteryInstance, BatteryVoltage, BatteryCurrent, BatteryTemperature, SID127508)) {
    		//
    		// valid PGN, update
    		//
    	}
    }

    /* enviromental parameters */
    if (N2kMsg.PGN == 130311) {

    	if (ParseN2kPGN130311(N2kMsg, SID, tempSource, envTemperature, humiditySource, envHumidity, envAtp)) {

    		//
    		// valid PGN130311
    		//
    	}

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
			int c = 0;
			if (json_scanf(recvBuffer, recvLen, "{connect: %d}", &c) > 0)
			{
//				printf("connected %d \n", c);
				connected = c;
			}
		}

		delay(1);
	}

	return NULL;
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
	char buf[256];
	struct json_out json_buf = JSON_OUT_BUF(buf, 256);

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


    while (true) {

    	/* parse N2k messages */
    	n2k.ParseMessages();

    	/* format into JSON & send over UDP socket */
    	json_printf(&json_buf, "{ engine: {rpm: %.0f}, fuel: {level: %.0f, capacity: %.0f}, battery: {soc: %d, soh: %d, volt: %.2f, current: %.2f}, weather: " \
    			"{ humidity: %.2f, temperature: %.2f } }\n", \
    			engSpeed, level, capacity, StateOfCharge, StateOfHealth, BatteryVoltage, BatteryCurrent, envHumidity, envTemperature);
    	json_buf.u.buf.len = 0;

    	if (connected) {
    		sendto(udpSock, buf,  strlen(buf), 0, (struct sockaddr *)&clientAddr, clientAddrkLen);
    	}

    	// 50 Hz
    	//delay(20);
    }

	return EXIT_SUCCESS;
}
