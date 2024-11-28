/*
 * Copyright (c) 2023 Ian Freislich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <coredecls.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <FastCRC.h>
#include <lwip/def.h>
#include <time.h>
#include <sys/time.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Wire.h>

#define MAGIC		0xd41d8cd5
#define CFG_NCATS	7
struct cfg {
	uint32_t	magic;
	char		hostname[33];
	char		ssid[64];
	char		wpakey[64];
	char		ntpserver[64];
	char		timezone[32];
	uint8_t		flags;
	struct {
		 char		name[20];
		 char		topic[64];
		 uint8_t	facility;
		 uint16_t	id;
		 uint8_t	flags;
	} cat[CFG_NCATS];
	struct {
		char	url[64];
		char	topic[64];
		char	username[16];
		char	password[16];
	} ntfy;
	uint16_t	crc;
} __attribute__((__packed__));

#define CFG_NTFY_ENABLE		0x01

#define CFG_CAT_EXIT		0x01
#define CFG_CAT_ENTRY		0x02

#define PIN_EXIT_DATA0		12
#define PIN_EXIT_DATA1		14
#define PIN_ENTRY_DATA0		5
#define PIN_ENTRY_DATA1		4
#define PIN_DOOR_SENSOR		13
#define PIN_ENTRY_SOLENOID	2
#define PIN_EXIT_SOLENOID	16

#define WEIGAND_TIMEOUT				20	// timeout in ms on Wiegand sequence 
#define DOOR_TIMEOUT_DEFAULT		60	// Door stays unlocked for max X seconds
#define DOOR_SWING_TIMEOUT_DEFAULT	3	// Door stays unlocked for max X seconds

#define LOCK	0
#define OPEN	1
#define CLOSED	0

enum direction {EXIT, ENTRY};

// Bitmap States
#define STATE_ENTRY_WEIGAND_DONE	0x0001
#define STATE_EXIT_WEIGAND_DONE		0x0002
#define STATE_OTA_FLASH				0x0004
#define STATE_NTP_GOT_TIME			0x0008
#define STATE_DOOR_TRIGGER			0x0010
#define STATE_GOT_IP_ADDRESS		0x0020
#define STATE_BOOTUP_NTFY			0x0040
#define STATE_ENTRY_OPEN			0x0080
#define STATE_EXIT_OPEN				0x0100
#define STATE_ENTRY_LOCKED_OPEN		0x0200
#define STATE_EXIT_LOCKED_OPEN		0x0400

WiFiUDP				udp;
ESP8266WebServer	webserver(80);
WiFiEventHandler	eventConnected, eventDisconnected, eventGotIP;

uint8_t				catInOut = 0;
time_t				catTime[CFG_NCATS] = {0};
struct cfg			conf;
time_t				bootTime = 0;

volatile uint16_t	state = 0;
volatile uint64_t	entryDataBits;
volatile uint8_t	entryBitCount;
volatile u_long		entryLastBit;
volatile uint64_t	exitDataBits;
volatile uint8_t	exitBitCount;
volatile u_long		exitLastBit;
volatile u_long		doorTrigger;

void debug(byte, const char *, ...);
const char *catName(uint8_t, uint16_t);
const char *catTopic(uint8_t, uint16_t);
int catNumber(uint8_t, uint16_t);
int checkCard(enum direction, uint8_t, uint16_t);
void configInit(void);
void configSave(void);
void configDefault(void);
void exitLock(void);
void exitUnlock(void);
void entryLock(void);
void entryUnlock(void);
void ntfy(const char *, const char *, const char *, const uint8_t, const char *, ...);
void ntpCallBack(void);
int weigandDecode(uint8_t *, uint16_t *, uint8_t, uint64_t);

void handleRoot(void);
void handleConfig(void);
void handleSave(void);
void handleReboot(void);

void IRAM_ATTR ISR_ENTRY_D0(void);
void IRAM_ATTR ISR_ENTRY_D1(void);
void IRAM_ATTR ISR_EXIT_D0(void);
void IRAM_ATTR ISR_EXIT_D1(void);
void IRAM_ATTR ISR_DOOR(void);

void
setup()
{
	char      hostname[42];

	Serial.begin(115200);
	while (!Serial);
	Serial.println();
	debug(true, "Startup, reason: %s", (ESP.getResetReason()).c_str());
	EEPROM.begin(1536);
	configInit();

	analogWriteFreq(400);
	pinMode(PIN_ENTRY_DATA0, INPUT);
	pinMode(PIN_ENTRY_DATA1, INPUT);
	pinMode(PIN_ENTRY_SOLENOID, OUTPUT);
	digitalWrite(PIN_ENTRY_SOLENOID, LOW);
	pinMode(PIN_EXIT_DATA0, INPUT);
	pinMode(PIN_EXIT_DATA1, INPUT);
	pinMode(PIN_EXIT_SOLENOID, OUTPUT);
	digitalWrite(PIN_EXIT_SOLENOID, LOW);
	pinMode(PIN_DOOR_SENSOR, INPUT_PULLUP);

	eventGotIP = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
		debug(true, "IP address %s", WiFi.localIP().toString());
		state |= STATE_GOT_IP_ADDRESS;
	});
	eventConnected = WiFi.onStationModeConnected([](const WiFiEventStationModeConnected& event) {
		debug(true, "WiFi Connected");
	});
	eventDisconnected = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
	{
		state &= ~STATE_GOT_IP_ADDRESS;
		if (~state & STATE_OTA_FLASH) {
			debug(true, "WiFi Disconnected, reconnecting");
			WiFi.begin(conf.ssid, conf.wpakey);
		}
		else {
			debug(true, "WiFi Disconnected");
		}
	});
	
	snprintf(hostname, 42, "CatFlap-%s", conf.hostname);
	WiFi.mode(WIFI_STA);
	WiFi.hostname(hostname);
	WiFi.begin(conf.ssid, conf.wpakey);
	Wire.begin();
	MDNS.begin(hostname);

	bootTime = time(NULL);
	configTzTime(conf.timezone, conf.ntpserver);
	settimeofday_cb(ntpCallBack);

	ArduinoOTA.setPort(8266);
	ArduinoOTA.setHostname(conf.hostname);
	// ArduinoOTA.setPassword(F("admin"))
	ArduinoOTA.onStart([]() {
		const char *type = "Unknown";
		switch (ArduinoOTA.getCommand()) {
			case U_FLASH:
				type = "firmware";
				break;
			case U_FS:
				type = "SPIFFS";
				//SPIFFS.end();
				break;
		}
		ntfy(conf.ntfy.topic, WiFi.getHostname(), "floppy_disk", 3, "Updating: %s", type);
	});
	ArduinoOTA.onEnd([]() {
		state |= STATE_OTA_FLASH;
		debug(true, "Flashing...");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
			Serial.printf("Received: %7d of %7d\r", progress, total);
	});
	ArduinoOTA.onError([](ota_error_t error) {
		state &= ~STATE_OTA_FLASH;
		debug(true, "Flashing...");
		Serial.print("Error[");
		Serial.print(error);
		Serial.print("]:");
		switch (error) {
			case OTA_AUTH_ERROR:
				Serial.println("Auth Failed");
				break;
			case OTA_BEGIN_ERROR:
				Serial.println("Begin Failed");
				break;
			case OTA_CONNECT_ERROR:
				Serial.println("Connect Failed");
				break;
			case OTA_RECEIVE_ERROR:
				Serial.println("Receive Failed");
				break;
			case OTA_END_ERROR:
				Serial.println("End Failed");
				break;
		}
	});

	attachInterrupt(PIN_ENTRY_DATA0, ISR_ENTRY_D0, FALLING);
	attachInterrupt(PIN_ENTRY_DATA1, ISR_ENTRY_D1, FALLING);
	attachInterrupt(PIN_EXIT_DATA0, ISR_EXIT_D0, FALLING);
	attachInterrupt(PIN_EXIT_DATA1, ISR_EXIT_D1, FALLING);
	attachInterrupt(PIN_DOOR_SENSOR, ISR_DOOR, CHANGE);

	webserver.on("/", handleRoot);
	webserver.on("/config", handleConfig);
	webserver.on("/save", handleSave);
	webserver.on("/reboot", handleReboot);

	webserver.begin();
	ArduinoOTA.begin();
}

void
loop()
{
	static time_t	entryCloseAt = 0, exitCloseAt = 0;
	static time_t	entryCloseTime = 0, exitCloseTime = 0;
	static uint8_t	lastFacilityCode = 0;
	static uint16_t lastCardCode = 0;
	uint8_t			facilityCode;
	uint16_t		cardCode;
	int				catNum;

	ArduinoOTA.handle();
	webserver.handleClient();
	// We don't have an IP address until long after setup exits and sending the notification during the callback causes a crash
	if (~state & STATE_BOOTUP_NTFY && state & STATE_GOT_IP_ADDRESS) {
		ntfy(conf.ntfy.topic, WiFi.getHostname(), "facepalm", 3, "Boot up %6.3f seconds ago\\nReset cause: %s\\nFirmware %s %s",
		  millis() / 1000.0, (ESP.getResetReason()).c_str(), __DATE__, __TIME__);
		state |= STATE_BOOTUP_NTFY;
	}

	if (entryLastBit && entryLastBit <= millis())
		state |= STATE_ENTRY_WEIGAND_DONE;
	if (exitLastBit && exitLastBit <= millis())
		state |= STATE_EXIT_WEIGAND_DONE;

	if (state & STATE_ENTRY_WEIGAND_DONE) {
		if (weigandDecode(&facilityCode, &cardCode, entryBitCount, entryDataBits) && ~state & STATE_EXIT_OPEN) {
			switch (checkCard(ENTRY, facilityCode, cardCode)) {
				case 0:
					ntfy(catTopic(facilityCode, cardCode), WiFi.getHostname(), "stop_sign", 3, "Entry denied for %s", catName(facilityCode, cardCode));
					debug(true, "Entry denied for %s", catName(facilityCode, cardCode));
					break;
				case 1:
					entryUnlock();
					entryCloseAt = time(NULL) + DOOR_TIMEOUT_DEFAULT;
					if (lastFacilityCode != facilityCode && lastCardCode != cardCode) {
						ntfy(catTopic(facilityCode, cardCode), WiFi.getHostname(), "unlock,arrow_left", 3, "%s Entry", catName(facilityCode, cardCode));
						catNum = catNumber(facilityCode, cardCode);
						if (catNum < CFG_NCATS) {
							catTime[catNum] = time(NULL);
							catInOut |= 1 << catNum;
						}
					}
					lastFacilityCode = facilityCode;
					lastCardCode = cardCode;
					debug(true, "%s Entry", catName(facilityCode, cardCode));
					break;
			}
		}
		entryBitCount = 0;
		entryDataBits = 0;
		entryLastBit = 0;
		state &= ~STATE_ENTRY_WEIGAND_DONE;
	}

	if (state & STATE_EXIT_WEIGAND_DONE) {
		if (weigandDecode(&facilityCode, &cardCode, exitBitCount, exitDataBits) && ~state & STATE_ENTRY_OPEN) {
			switch (checkCard(EXIT, facilityCode, cardCode)) {
				case 0:
					ntfy(catTopic(facilityCode, cardCode), WiFi.getHostname(), "stop_sign", 3, "Exit denied for %s", catName(facilityCode, cardCode));
					debug(true, "Entry denied for %s", catName(facilityCode, cardCode));
					break;
				case 1:
					exitUnlock();
					exitCloseAt = time(NULL) + DOOR_TIMEOUT_DEFAULT;
					if (lastFacilityCode != facilityCode && lastCardCode != cardCode) {
						ntfy(catTopic(facilityCode, cardCode), WiFi.getHostname(), "arrow_right,unlock", 3, "%s Exit", catName(facilityCode, cardCode));
						catNum = catNumber(facilityCode, cardCode);
						if (catNum < CFG_NCATS) {
							catTime[catNum] = time(NULL);
							catInOut &= ~(1 << catNum);
						}
					}
					lastFacilityCode = facilityCode;
					lastCardCode = cardCode;
					debug(true, "%s Exit", catName(facilityCode, cardCode));
					break;
			}
		}
		exitBitCount = 0;
		exitDataBits = 0;
		exitLastBit = 0;
		state &= ~STATE_EXIT_WEIGAND_DONE;
	}
	if (state & STATE_DOOR_TRIGGER) {
		if (state & STATE_ENTRY_OPEN) {
			debug(true, "swing entry");
			entryCloseAt = time(NULL) + DOOR_SWING_TIMEOUT_DEFAULT;
		}
		if (state & STATE_EXIT_OPEN) {
			debug(true, "swing exit");
			exitCloseAt = time(NULL) + DOOR_SWING_TIMEOUT_DEFAULT;
		}
		state &= ~STATE_DOOR_TRIGGER;
	}
	if (~state & STATE_ENTRY_OPEN && ~state & STATE_ENTRY_LOCKED_OPEN && time(NULL) - entryCloseTime < 2 && digitalRead(PIN_DOOR_SENSOR)) {
		entryUnlock();
		ntfy(conf.ntfy.topic, WiFi.getHostname(), "lock,unlock", 3, "Locked open (entry)");
		debug(true, "Locked open (entry)");
		state |= STATE_ENTRY_LOCKED_OPEN;
	}
	if (~state & STATE_EXIT_OPEN && ~state & STATE_EXIT_LOCKED_OPEN && time(NULL) - exitCloseTime < 2 && digitalRead(PIN_DOOR_SENSOR)) {
		exitUnlock();
		ntfy(conf.ntfy.topic, WiFi.getHostname(), "lock,unlock", 3, "Locked open (exit)");
		debug(true, "Locked open (exit)");
		state |= STATE_EXIT_LOCKED_OPEN;
	}
	if (state & (STATE_ENTRY_LOCKED_OPEN | STATE_EXIT_LOCKED_OPEN))
		delay(500);
	if (state & STATE_ENTRY_OPEN && !digitalRead(PIN_DOOR_SENSOR) && entryCloseAt < time(NULL)) {
		entryLock();
		lastFacilityCode = 0;
		lastCardCode = 0;
		if (~state & STATE_ENTRY_LOCKED_OPEN)
			entryCloseTime = time(NULL);
		entryCloseAt = 0;
		state &= ~STATE_ENTRY_OPEN & ~STATE_ENTRY_LOCKED_OPEN;
		debug(true, "Lock entry");
	}
	if (state & STATE_EXIT_OPEN && !digitalRead(PIN_DOOR_SENSOR) && exitCloseAt < time(NULL)) {
		exitLock();
		lastFacilityCode = 0;
		lastCardCode = 0;
		if (~state & STATE_EXIT_LOCKED_OPEN)
			exitCloseTime = time(NULL);
		exitCloseAt = 0;
		state &= ~STATE_EXIT_OPEN & ~STATE_EXIT_LOCKED_OPEN;
		debug(true, "Lock exit");
	}
}

int
weigandDecode(uint8_t *facilityCode, uint16_t *cardCode, uint8_t bitCount, uint64_t dataBits)
{
	switch (bitCount) {
		case 26:
			*facilityCode = (dataBits & 0x1FE0000) >> 17;
			*cardCode = (dataBits & 0x1FFFE) >> 1;
			return(true);
			break;
		default:
			debug(true, "Unknown card format %d", bitCount);
			return(false);
			break;
	}
}

void
entryLock(void)
{
	digitalWrite(PIN_ENTRY_SOLENOID, LOCK);
	delay(8);
	analogWrite(PIN_ENTRY_SOLENOID, 180);
	delay(11);
	digitalWrite(PIN_ENTRY_SOLENOID, LOCK);
	state &= ~STATE_ENTRY_OPEN;
}

void
entryUnlock(void)
{
	digitalWrite(PIN_ENTRY_SOLENOID, OPEN);
	delay(30);
	analogWrite(PIN_ENTRY_SOLENOID, 180);
	state |= STATE_ENTRY_OPEN;
}

void
exitLock(void)
{
	digitalWrite(PIN_EXIT_SOLENOID, LOCK);
	delay(8);
	analogWrite(PIN_EXIT_SOLENOID, 180);
	delay(11);
	digitalWrite(PIN_EXIT_SOLENOID, LOCK);
	state &= ~STATE_EXIT_OPEN;
}

void
exitUnlock(void)
{
	digitalWrite(PIN_EXIT_SOLENOID, OPEN);
	delay(20);
	analogWrite(PIN_EXIT_SOLENOID, 50);
	state |= STATE_EXIT_OPEN;
}

int
checkCard(enum direction dir, uint8_t facilityCode, uint16_t cardCode)
{
	int   i;

	for (i = 0; i < CFG_NCATS; i++)
		if (conf.cat[i].facility == facilityCode && conf.cat[i].id == cardCode)
			break;

	if (i == CFG_NCATS) {
		debug(true, "Unknown Card: facility %d, card %d", facilityCode, cardCode);
		ntfy(conf.ntfy.topic, WiFi.getHostname(), "interrobang", 3, "Unknown Card: facility %d, card %d", facilityCode, cardCode);
		return(-1);
	}

	return((dir == EXIT && conf.cat[i].flags & CFG_CAT_EXIT) || (dir == ENTRY && conf.cat[i].flags & CFG_CAT_ENTRY));
}

const char *
catName(uint8_t facilityCode, uint16_t cardCode)
{
	int i = catNumber(facilityCode, cardCode);

	if (i == CFG_NCATS)
		return("Unnamed");
		
	return(conf.cat[i].name);
}

const char *
catTopic(uint8_t facilityCode, uint16_t cardCode)
{
	int i = catNumber(facilityCode, cardCode);

	if (i == CFG_NCATS || !strlen(conf.cat[i].topic))
		return(conf.ntfy.topic);
		
	return(conf.cat[i].topic);
}

int
catNumber(uint8_t facilityCode, uint16_t cardCode)
{
	int i;
	for (i = 0; i < CFG_NCATS; i++)
		if (conf.cat[i].facility == facilityCode && conf.cat[i].id == cardCode)
			break;
	return(i);
}

void
configDefault(void)
{
	memset(&conf, '\0', sizeof(conf));
	WiFi.macAddress().toCharArray(conf.hostname, 32);
	conf.magic = MAGIC;
	strcpy(conf.timezone, "EST5EDT,M3.2.0,M11.1.0");
	strcpy(conf.ssid, "");
	strcpy(conf.wpakey, "");
	strcpy(conf.ntpserver, "pool.ntp.org");
}

void
configSave(void)
{
	FastCRC16       CRC16;
	unsigned char  *p;

	conf.crc = CRC16.ccitt(reinterpret_cast<uint8_t *>(&conf), sizeof(cfg) - 2);
	p = reinterpret_cast<unsigned char *>(&conf);
	for (uint16_t i = 0; i < sizeof(struct cfg); i++)
		EEPROM.write(i, *p++);
	EEPROM.commit();
}

void
configInit(void)
{
	FastCRC16       CRC16;
	unsigned char  *p;

	p = reinterpret_cast<unsigned char *>(&conf);
	for (uint16_t i = 0; i < sizeof(struct cfg); i++)
		*p++ = EEPROM.read(i);
	if (conf.magic != MAGIC || conf.crc != CRC16.ccitt(reinterpret_cast<uint8_t *>(&conf), sizeof(struct cfg) - 2)) {
		debug(true, "Settings corrupted, defaulting");
		configDefault();
		configSave();
	}
}

void
debug(byte logtime, const char *format, ...)
{
	va_list    pvar;
	char       str[60];
	char       timestr[20];
	time_t     t = time(NULL);
	struct tm *tm;
	
	tm = localtime(&t);
	va_start(pvar, format);
	if (logtime && state & STATE_NTP_GOT_TIME) {
		strftime(timestr, 20, "%F %T", tm);
		Serial.print(timestr);
		Serial.print(": "); 
	}
	vsnprintf(str, 60, format, pvar);
	Serial.println(str);
	va_end(pvar);
}

// tags: https://docs.ntfy.sh/emojis/
// priority: https://docs.ntfy.sh/publish/#message-priority
void
ntfy(const char *topic, const char *title, const char *tags, const uint8_t priority, const char *format, ...)
{
	HTTPClient  http;
	WiFiClient  client;
	va_list     pvar;
	char       *buffer, *message;
	int         content_length;

	if (~conf.flags & CFG_NTFY_ENABLE)
		return;

	if ((buffer = static_cast<char *>(malloc(3072))) == NULL) {
		debug(true, "NTFY failed to allocate memory");
		return;
	}
	if ((message = static_cast<char *>(malloc(2048))) == NULL) {
		debug(true, "NTFY failed to allocate memory");
		free(buffer);
		return;
	}
	va_start(pvar, format);
	vsnprintf(message, 2048, format, pvar);
	va_end(pvar);

	http.setAuthorization(conf.ntfy.username, conf.ntfy.password);
	http.begin(client, static_cast<const char *>(conf.ntfy.url));
	http.addHeader("Content-Type", "application/json");

	const char *post_data = "{"
		"\"topic\":\"%s\","
		"\"title\":\"%s\","
		"\"tags\":[\"%s\"],"
		"\"priority\":%d,"
		"\"message\":\"%s\""
	"}";
	content_length = snprintf(buffer, 3072, post_data, topic, title, tags, priority, message);
	http.POST(reinterpret_cast<const uint8_t *>(buffer), content_length);

	http.end();
	free(message);
	free(buffer);
}

/*--------------------------------------------------------------
 * Web Server
 * 
 *--------------------------------------------------------------
 */

void
handleRoot()
{
	char		*body;
	char		 timestr[20];
	time_t		 t = time(NULL);
	int			 sec = t - bootTime;
	int			 min = sec / 60;
	int			 hr = min / 60;
	struct tm	*tm;
	int			 pos = 0;
	
	tm = localtime(&t);
	strftime(timestr, 20, "%F %T", tm);
	
	if ((body = static_cast<char *>(malloc(2048))) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	pos = snprintf(body, 2048,
		"<html>"
		"<head>"
		"<meta http-equiv='Refresh' content='60'>"
		"<title>CatFlap [%s]</title>\n"
		"<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
		"</head>\n"
		"<body>\n"
		"<h1>CatFlap %s</h1>"
		"Time: %s<BR>\n"
		"<p>"
		"<table border=0 width='520' cellspacing=4 cellpadding=0>\n",
		conf.hostname, conf.hostname, timestr);

	for (int i = 0; i < CFG_NCATS; i++) {
		if (catTime[i] == 0)
			continue;
		tm = localtime(&catTime[i]);
		strftime(timestr, 20, "%F %T", tm);
		pos += snprintf(body + pos, 2048 - pos, "<tr><td>%s</td><td>%s</td><td>%s</td></tr>",
		conf.cat[i].name, catInOut & (1 << i) ? "In" : "Out", timestr);
	}

	pos = snprintf(body + pos, 2048 - pos,
		"</table><p>"
		"<a href='/config'>System Configuration</a>"
		"<p><font size=1>"
		"Uptime: %d days %02d:%02d:%02d<br>"
		"Firmware: " __DATE__ " " __TIME__
		"</font"
		"</body>\n"
		"</html>", sec / 86400, hr % 24, min % 60, sec % 60);
	webserver.send(200, "text/html", body);
	free(body);
}

void
handleConfig()
{
	char *body, *temp;

	if ((body = static_cast<char *>(malloc(8800))) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	if ((temp = static_cast<char *>(malloc(865))) == NULL) {
		free(body);
		return;
	}
	
	snprintf(body, 2500, // 1917 chars
		"<html>"
		"<head>\n"
		"<title>CatFlap [%s]</title>\n"
		"<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
		"</head>\n"
		"<body>\n"
		"<form method='post' action='/save' name='Configuration'/>\n"
		"<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
		"<tr><td width='40%%'>Name:</td><td><input name='name' type='text' value='%s' size='31' maxlength='31'></td></tr>\n"
		"<tr><td width='40%%'>SSID:</td><td><input name='ssid' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
		"<tr><td width='40%%'>WPA Pass Phrase:</td><td><input name='key' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
		"<tr><td width='40%%'>NTP Server:</td><td><input name='ntp' type='text' value='%s' size='31' maxlength='63' "
			"pattern='^([a-z0-9]+)(\\.)([_a-z0-9]+)((\\.)([_a-z0-9]+))?$' title='A valid hostname'></td></tr>\n"
		"<tr><td width='40%%'>Timezone:</td><td><input name='tz' type='text' value='%s' size='31' maxlength='31'></td></tr>\n"
		"<tr><td width='40%%'>Notifications:</td><td><input name='ntfy' type='checkbox' value='true' %s></td></tr>\n"
		"<tr><td width='40%%'>Service URL:</td><td><input name='url' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
		"<tr><td width='40%%'>Topic:</td><td><input name='topic' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
		"<tr><td width='40%%'>Username:</td><td><input name='user' type='text' value='%s' size='15' maxlength='15'></td></tr>\n"
		"<tr><td width='40%%'>Password:</td><td><input name='passwd' type='text' value='%s' size='15' maxlength='15'></td></tr>\n"
		"</table><p>",
		conf.hostname, conf.hostname, conf.ssid, conf.wpakey, conf.ntpserver, conf.timezone,
		conf.flags & CFG_NTFY_ENABLE ? "checked" : "",
		conf.ntfy.url, conf.ntfy.topic, conf.ntfy.username, conf.ntfy.password);

	// max 576 characters per cat
	for (int i = 0; i < CFG_NCATS; i++) {
		snprintf(temp, 865,
			"<table border=0 width='520' cellspacing=4 cellpadding=0>\n"
			"<tr><td width='40%%'>Cat %d:</td><td><input name='catname%d' type='text' value='%s' size='19' maxlength='19'></td></tr>\n"
			"<tr><td width='40%%'>Topic:</td><td><input name='topic%d' type='text' value='%s' size='31' maxlength='63'></td></tr>\n"
			"<tr><td width='40%%'>Facility Code:</td><td><input name='facility%d' type='number' size='4' value='%d' min='0' max='255'></td></tr>\n"
			"<tr><td width='40%%'>Tag ID:</td><td><input name='id%d' type='number' size='8' value='%d' min='0' max='8191'></td></tr>\n"
			"<tr><td width='40%%'>Entry:</td><td><input name='entry%d' type='checkbox' value='true' %s></td></tr>\n"
			"<tr><td width='40%%'>Exit:</td><td><input name='exit%d' type='checkbox' value='true' %s></td></tr>"
			"</table><p>",
			i + 1, i, conf.cat[i].name, i, conf.cat[i].topic, i, conf.cat[i].facility, i, conf.cat[i].id,
			i, conf.cat[i].flags & CFG_CAT_ENTRY ? "checked" : "",
			i, conf.cat[i].flags & CFG_CAT_EXIT ? "checked" : "");
		strcat(body, temp);
	}
	strcat(body, //192 chars
		"<input name='Save' type='submit' value='Save'/>\n"
		"<br></form>"
		"<form method='post' action='/reboot' name='Reboot'/>\n"
		"<input name='Reboot' type='submit' value='Reboot'/>\n"
		"<br></form>\n"
		"</body>\n"
		"</html>");

	webserver.send(200, "text/html", body);
	free(body);
	free(temp);
}

void
handleReboot()
{
	char *body;

	if ((body = static_cast<char *>(malloc(500))) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}
	snprintf(body, 500,
		"<html>"
		"<head>"
		"<title>CatFlap [%s]</title>\n"
		"<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
		"</head>\n"
		"<body>\n"
		"Rebooting<br>"
		"<meta http-equiv='Refresh' content='5; url=/'>"
		"</body>\n"
		"</html>", conf.hostname);

	webserver.send(200, "text/html", body);
	free(body);
	delay(100);
	state |= STATE_OTA_FLASH;
	ESP.restart();
}

void
handleSave()
{
	char   *temp, hostname[42];
	String  value;

	if ((temp = static_cast<char *>(malloc(400))) == NULL) {
		debug(true, "WEB / failed to allocate memory");
		return;
	}

	if (webserver.hasArg("name")) {
		value = webserver.urlDecode(webserver.arg("name"));
		strncpy(conf.hostname, value.c_str(), 32);
	}

	if (webserver.hasArg("ssid")) {
		value = webserver.urlDecode(webserver.arg("ssid"));
		strncpy(conf.ssid, value.c_str(), 64);
	}

	if (webserver.hasArg("key")) {
		value = webserver.urlDecode(webserver.arg("key"));
		strncpy(conf.wpakey, value.c_str(), 64);
	}

	if (webserver.hasArg("ntp")) {
		value = webserver.urlDecode(webserver.arg("ntp"));
		strncpy(conf.ntpserver, value.c_str(), 32);
	}

	if (webserver.hasArg("tz")) {
		value = webserver.urlDecode(webserver.arg("tz"));
		strncpy(conf.timezone, value.c_str(), 32);
	}

	if (webserver.hasArg("ntfy"))
		conf.flags |= CFG_NTFY_ENABLE;
	else
		conf.flags &= ~CFG_NTFY_ENABLE;

	if (webserver.hasArg("url")) {
		value = webserver.urlDecode(webserver.arg("url"));
		strncpy(conf.ntfy.url, value.c_str(), 64);
	}

	if (webserver.hasArg("topic")) {
		value = webserver.urlDecode(webserver.arg("topic"));
		strncpy(conf.ntfy.topic, value.c_str(), 64);
	}

	if (webserver.hasArg("user")) {
		value = webserver.urlDecode(webserver.arg("user"));
		strncpy(conf.ntfy.username, value.c_str(), 16);
	}

	if (webserver.hasArg("passwd")) {
		value = webserver.urlDecode(webserver.arg("passwd"));
		strncpy(conf.ntfy.password, value.c_str(), 16);
	}

	for (int i=0; i < CFG_NCATS; i++) {
		snprintf(temp, 399, "catname%d", i);
		if (webserver.hasArg(temp)) {
			value = webserver.urlDecode(webserver.arg(temp));
			strncpy(conf.cat[i].name, value.c_str(), 20);
		}

		snprintf(temp, 399, "topic%d", i);
		if (webserver.hasArg(temp)) {
			value = webserver.urlDecode(webserver.arg(temp));
			strncpy(conf.cat[i].topic, value.c_str(), 64);
		}

		snprintf(temp, 399, "facility%d", i);
		value = webserver.arg(temp);
		if (value.length() && value.toInt() >= 0 && value.toInt() <= 255)
			conf.cat[i].facility = value.toInt();

		snprintf(temp, 399, "id%d", i);
		value = webserver.arg(temp);
		if (value.length() && value.toInt() >= 0 && value.toInt() <= 8191)
			conf.cat[i].id = value.toInt();

		snprintf(temp, 399, "entry%d", i);
		if (webserver.hasArg(temp))
			conf.cat[i].flags |= CFG_CAT_ENTRY;
		else
			conf.cat[i].flags &= ~CFG_CAT_ENTRY;

		snprintf(temp, 399, "exit%d", i);
		if (webserver.hasArg(temp))
			conf.cat[i].flags |= CFG_CAT_EXIT;
		else
			conf.cat[i].flags &= ~CFG_CAT_EXIT;
	}

	snprintf(hostname, 42, "CatFlap-%s", conf.hostname);
	WiFi.hostname(hostname);
	MDNS.setHostname(hostname);
	
	snprintf(temp, 400,
		"<html>"
		"<head>"
		"<title>CatFlap [%s]</title>\n"
		"<style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>"
		"</head>\n"
		"<body>\n"
		"Updated conguration, %d items<br>"
		"<meta http-equiv='Refresh' content='3; url=/'>"
		"</body>\n"
		"</html>", conf.hostname, webserver.args());

	webserver.send(200, "text/html", temp);
	free(temp);
		
	configSave();
}

/*--------------------------------------------------------------
 * Callbacks and ISRs
 * 
 *--------------------------------------------------------------
 */

void
ntpCallBack(void)
{
	if (~state & STATE_NTP_GOT_TIME)
		bootTime = time(NULL) - millis() / 1000;

	state |= STATE_NTP_GOT_TIME;
	debug(true, "ntp: time sync");
}

void IRAM_ATTR
ISR_ENTRY_D0(void)
{
	if (~state & STATE_ENTRY_WEIGAND_DONE) {
		entryLastBit = millis() + WEIGAND_TIMEOUT;
		entryBitCount++;
		entryDataBits <<= 1;
	}
}

void IRAM_ATTR
ISR_ENTRY_D1(void)
{
	if (~state & STATE_ENTRY_WEIGAND_DONE) {
		entryLastBit = millis() + WEIGAND_TIMEOUT;
		entryBitCount++;
		entryDataBits <<= 1;
		entryDataBits |= 1;
	}
}

void IRAM_ATTR
ISR_EXIT_D0(void)
{
	if (~state & STATE_EXIT_WEIGAND_DONE) {
		exitLastBit = millis() + WEIGAND_TIMEOUT;
		exitBitCount++;
		exitDataBits <<= 1;
	}
}

void IRAM_ATTR
ISR_EXIT_D1(void)
{
	if (~state & STATE_EXIT_WEIGAND_DONE) {
		exitLastBit = millis() + WEIGAND_TIMEOUT;
		exitBitCount++;
		exitDataBits <<= 1;
		exitDataBits |= 1;
	}
}

void IRAM_ATTR
ISR_DOOR(void)
{
	doorTrigger = millis();
	state |= STATE_DOOR_TRIGGER;
}
