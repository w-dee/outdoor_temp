#include "../.ssid.h"
#include <soc/io_mux_reg.h>
#include <soc/i2s_reg.h>
#include <soc/i2s_struct.h>
#include "freertos/queue.h"
#include "rom/lldesc.h"
#include <WiFi.h>
#include <Wire.h>
#include "bme280.h"
#include "buildinfo.h"
#include <WebServer.h>
#include <FS.h>
#include <esp_wifi.h>
#include <StreamString.h>
#include <Update.h>

#include <Preferences.h>

#define LED_POWER_PIN 25
#define LED_WIFI_PIN 26
#define LED_VALVE_PIN 27

//-----------------------------------------------------------------------//
static BME280 bme280;

struct bme280_result_t
{
	int temp_10; // temperature in deg C, 10 multiplied
	int humidity; // humidity in RH%
	int pressure; // atmosphere pressure in hPa
};

static bme280_result_t bme280_result;

static uint8_t valve_auto_status = 1; // 0=close, 1=auto, 2=forced open
static bool is_valve_open = 0;
//-----------------------------------------------------------------------//
static Preferences preferences;

// configs

// define 'daytime' if the hour is in range of:
static uint8_t daytime_start = 6;
static uint8_t daytime_end = 18;

// define 'rainy' if the relative humidity is equal to or above:
static uint8_t rainy_humidity = 97;

// define 'nonrainy' if the relative humidity is equal to or below:
static uint8_t nonrainy_humidity = 94;

// define 'hot' if the temperature in deg C x 10 is equal to or above:
static int16_t hot_temperature10 = 262;

// define hysteresis of the temperature; it is assumed as 'cool'
// if the temperature goes equal to / down below this temperature.
// unit is deg C x 10
static int16_t cool_temperature10 = 257; 

// open valve specified minute to specified minutes each hour
static uint8_t open_valve_min = 4;

// timeserver and timezone
static String time_server (F("192.168.0.40"));
static String time_zone = (F("JST-9"));

void init_preference()
{
	auto && p = preferences;
	p.begin("my-app", false);
#define L(x) x = p.getUChar(String(F(#x)).c_str(), x)
	L(daytime_start);
	L(daytime_end);
	// note: "*_humidity" and "*_temperature10" are too long name
	// to store/restore to/from preferences. So I use shortened name instead.
	rainy_humidity     = p.getShort(String(F("rainy_hum"   )).c_str(), rainy_humidity );
	nonrainy_humidity  = p.getShort(String(F("nonrainy_hum")).c_str(), nonrainy_humidity);
	hot_temperature10  = p.getShort(String(F("hot_temp10"  )).c_str(), hot_temperature10 );
	cool_temperature10 = p.getShort(String(F("cool_temp10 ")).c_str(), cool_temperature10);
	L(open_valve_min);
	time_server = p.getString(String(F("time_server")).c_str(), time_server);
	time_zone =   p.getString(String(F("time_zone")).c_str(), time_zone);
	p.end();
}

void save_preference()
{
	auto && p = preferences;
	p.begin("my-app", false);
#define S(x) p.putUChar(String(F(#x)).c_str(), x)
	S(daytime_start);
	S(daytime_end);
	p.putShort(String(F("rainy_hum"   )).c_str(),   rainy_humidity );
	p.putShort(String(F("nonrainy_hum")).c_str(), nonrainy_humidity);
	p.putShort(String(F("hot_temp10"  )).c_str(),  hot_temperature10 );
	p.putShort(String(F("cool_temp10" )).c_str(),  cool_temperature10);
	S(open_valve_min);
	p.putString(String(F("time_server")).c_str(), time_server);
	p.putString(String(F("time_zone")).c_str(), time_zone);
	p.end();

	configTzTime(time_zone.c_str(), time_server.c_str(), NULL, NULL);
}

//-----------------------------------------------------------------------//
// logic
static bool is_hot = false;
static bool is_rainy = false;
static bool should_open_valve(int min, int hour, int humidity, int temperature10)
{
	// check whether it is hot or not
	if(!is_hot && temperature10 >= hot_temperature10) 
	{
		is_hot = true;
	}
	else if(is_hot && temperature10 <= cool_temperature10)
	{
		is_hot = false;
	}

	// check whether it is rainy or not
	if(!is_rainy && humidity >= rainy_humidity) 
	{
		is_rainy = true;
	}
	else if(is_rainy && humidity <= nonrainy_humidity)
	{
		is_rainy = false;
	}

	// if hot, force open valve unless it is humid:
	if(is_hot)
	{
		if(!is_rainy)
		{
			// force open, but give it 5 minutes close per one hour
			// to prevent valve's solenoid from overheat
			if(min < 5) return false; else return true;
		}
		else
		{
			return false;
		}
	}

	// else, if it is in daytime and it is not humid,
	// open valve only in specified minutes in each hour
	if(daytime_start <= hour && hour < daytime_end)
	{
		if(!is_rainy)
		{
			if(min < open_valve_min) return true;
		}
	}

	return false;
}

#define VALVE_GATE_PIN 23
#define VALVE_DRAIN_PIN 13
#define PUMP_PIN 24 /* unused */

#define VALVE_DRAIN_DURATION 12000 // drain valve open duration
#define VALVE_GATE_OPEN_DERAY 100 // delay time from drain valve close to gate valve open
#define PUMP_CLOSE_DELAY 500 // delay time from valve gate close to pump power off

static void set_valve_open(bool b)
{
	// this function is called continuously with an argument which indicates
	// the valve should open or not at the time .
	static bool prev_valve_state = false;
	static uint32_t valve_action_tick = 0;
	static uint8_t valve_action_state = 0; // 0x1x = open action, 0x2x = close action

	if(b != prev_valve_state)
	{
		prev_valve_state = b;
		if(b)
		{
			// valve is to be open
			digitalWrite(LED_VALVE_PIN, 1);
			valve_action_tick = millis();
			valve_action_state = 0x10;
		}
		else
		{
			// valve is close
			digitalWrite(LED_VALVE_PIN, 0);
			valve_action_tick = millis();
			valve_action_state = 0x20;
		}
	}

	if(valve_action_state != 0)
	{
		if((int32_t)millis() - (int32_t)valve_action_tick >= 0)
		{
			switch(valve_action_state)
			{
			case 0x10: // open valve
				Serial.println("open: Opening valve");
				digitalWrite(VALVE_GATE_PIN, 1);
				valve_action_tick = millis() + 200;
				valve_action_state = 0x11;
				break;

			case 0x11: // open drain
				Serial.println("open: Opening drain");
				digitalWrite(VALVE_DRAIN_PIN, 1);
				valve_action_tick = millis() +     3000; // pre-open draining duration here
				valve_action_state = 0x12;
				break;

			case 0x12: // close drain
				Serial.println("open: Closing drain");
				digitalWrite(VALVE_DRAIN_PIN, 0);
				valve_action_state = 0x0; // end
				break;


			case 0x20: // open drain
				Serial.println("close: Opening drain");
				digitalWrite(VALVE_DRAIN_PIN, 1);
				valve_action_tick = millis() + 200;
				valve_action_state = 0x21;
				break;

			case 0x21: // close valve
				Serial.println("close: Closing valve");
				digitalWrite(VALVE_GATE_PIN, 0);
				valve_action_tick = millis() +     30000; // post-open draining duration here
				valve_action_state = 0x22;
				break;

			case 0x22: // close drain
				Serial.println("close: Closing drain");
				digitalWrite(VALVE_DRAIN_PIN, 0);
				valve_action_state = 0x0; // end
				break;


			default:
				; // do nothing
			}
		}
	}

}

static void check_valve_open()
{
	struct tm tm;
	getLocalTime(&tm, 0);
	if(tm.tm_year + 1900 < 2018)
	{
		// datetime is not retrieved yet
		is_valve_open = false;
	}
	if(valve_auto_status == 0)
	{
		// force close
		is_valve_open = false;
	}
	else if(valve_auto_status == 1)
	{
		// auto
		is_valve_open = should_open_valve(tm.tm_min, tm.tm_hour,
			(int)bme280_result.humidity, (int)bme280_result.temp_10 );
	}
	else if(valve_auto_status == 2)
	{
		// force open, but give it 5 minutes close per one hour
		// to prevent valve's solenoid from overheat
		if(tm.tm_min < 5) is_valve_open = false; else is_valve_open = true;
	}

	set_valve_open(is_valve_open);
}


//-----------------------------------------------------------------------//
#define WIRE_SDA 21
#define WIRE_SCL 22


static void wire_init()
{
	Wire.begin(WIRE_SDA, WIRE_SCL);
}

static void wire_reset()
{
	// Wire.reset() has gone... To reset the hardware, calling the destructor is needed
	Wire.~TwoWire();
	new (&Wire) TwoWire(0);
	wire_init();
}



static void sensors_bme280_init_registers()
{
	bme280.setMode(BME280_MODE_NORMAL, BME280_TSB_1000MS, BME280_OSRS_x1,
		BME280_OSRS_x1, BME280_OSRS_x1, BME280_FILTER_OFF);
}

static void sensors_bme280_get()
{
	double temperature, humidity, pressure;
	int retry = 5;
	while(retry--)
	{
		if(!bme280.getData(&temperature, &humidity, &pressure))
		{
			// failure;
			// umm...
//			Serial.println("BME280 reset");
			wire_reset();
			delay(10);
			bme280.begin();
			delay(10);
			sensors_bme280_init_registers();
			delay(10);
		}
		else
		{
			break;
		}

	}

	bme280_result.temp_10 = temperature < 0 ? (temperature * 10 - 0.5) : (temperature * 10 + 0.5) ;
	bme280_result.humidity = humidity;
	bme280_result.pressure = pressure;
}

static void sensors_bme280_init()
{
	wire_init();
	bme280.begin();
	sensors_bme280_init_registers();
}

//-----------------------------------------------------------------------//

static WebServer server(80);


 
static String updateIndex = F(
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>");



static void handleNotFound()
{
//	if(loadFromFS(server.uri())) return;
	String message = F("Not Found\n\n");
	message += String(F("URI: "));
	message += server.uri();
	message += String(F("\nMethod: "));
	message += (server.method() == HTTP_GET)?F("GET"):F("POST");
	message += String(F("\nArguments: "));
	message += server.args();
	message += String(F("\n"));
	for (uint8_t i=0; i<server.args(); i++){
		message += String(F(" NAME:"))+server.argName(i) + F("\n VALUE:") + server.arg(i) + F("\n");
	}
	Serial.print(message);
	server.send(404, F("text/plain"), message);

}

static String replace_value(const String &line, const String & key, const String &raw_value)
{
	String ret = line;
	String key_tag = String(F("name=\"")) + key + F("\"");
	ret.replace(key_tag, key_tag + String(F(" value=\"")) + raw_value + F("\""));
	return ret;
}


static void web_server_setup()
{
	// setup handlers

	server.on(F("/"), HTTP_GET, []() {
			String html = F(R"HTMLTEXT(

			<html><body>
			<h1>Outdoor environment</h1>
			<p>Valve operation mode:  OOOOO</p>
			<p>Currently the valve is:  PPPPP</p>
			<p>XXXXXX</p>
			<form action="/close" method=post><input type=submit name="ok" value="Force Close" /></form>
			<form action="/auto" method=post><input type=submit name="ok" value="Auto" /></form>
			<form action="/open" method=post><input type=submit name="ok" value="Force Open" /></form>
			<form action="/set" method="post">

<p>daytime_start      : <input type="text" name="daytime_start" width="3" /> o'clock</p>
<p>daytime_end        : <input type="text" name="daytime_end" width="3" /> o'clock</p>
<p>rainy_humidity     : <input type="text" name="rainy_humidity" width="3" /> % RH</p>
<p>nonrainy_humidity  : <input type="text" name="nonrainy_humidity" width="3" /> % RH</p>
<p>hot_temperature10  : <input type="text" name="hot_temperature10" width="3" /> deg C x 10</p>
<p>cool_temperature10 : <input type="text" name="cool_temperature10" width="3" /> deg C x 10</p>
<p>open_valve_min     : <input type="text" name="open_valve_min" width="3" /> minutes</p>
<p>time_server        : <input type="text" name="time_server" width="30" /></p>
<p>time_zone          : <input type="text" name="time_zone" width="3" /></p>

				<p><input type="submit" name="ok" value="ok" /> </p>
			</form>

			</body></html>

			)HTMLTEXT");

			switch(valve_auto_status)
			{
			case 0:
				html.replace(String(F("OOOOO")), String(F("Force close")));
				break;
			case 1:
				html.replace(String(F("OOOOO")), String(F("Auto")));
				break;
			case 2:
				html.replace(String(F("OOOOO")), String(F("Force open")));
				break;
			}

			html.replace(String(F("PPPPP")), String(is_valve_open? F("open") : F("close")));

			struct tm tm;
			getLocalTime(&tm, 0);

			StreamString st;
			st.printf_P(PSTR("As of %d/%02d/%02d %02d:%02d:%02d"),
				tm.tm_year + 1900,
				tm.tm_mon + 1,
				tm.tm_mday,
				tm.tm_hour,
				tm.tm_min,
				tm.tm_sec);
			st.printf_P(PSTR("<br>Temperature: %.1f deg C, Humidity: %d%%RH, Pressure: %dhPa"),
				bme280_result.temp_10 * (1.0 / 10.0),
				(int)bme280_result.humidity,
				(int)bme280_result.pressure);

			html.replace(String(F("XXXXXX")), st);
			html = replace_value(html, F("daytime_start"), String((long) daytime_start));
			html = replace_value(html, F("daytime_end"), String((long) daytime_end));
			html = replace_value(html, F("rainy_humidity"), String((long) rainy_humidity));
			html = replace_value(html, F("nonrainy_humidity"), String((long) nonrainy_humidity));
			html = replace_value(html, F("hot_temperature10"), String((long) hot_temperature10));
			html = replace_value(html, F("cool_temperature10"), String((long) cool_temperature10));
			html = replace_value(html, F("open_valve_min"), String((long) open_valve_min));
			html = replace_value(html, F("time_server"),  time_server); // TODO: escape
			html = replace_value(html, F("time_zone"), time_zone); // TODO: escape

			server.send(200,  F("text/html"), html		);
	});

	server.on(F("/set"), HTTP_POST, []() {

		
#define P(x) x = server.arg(F(#x)).toInt()
		P(daytime_start);
		P(daytime_end);
		P(rainy_humidity);
		P(nonrainy_humidity);
		P(hot_temperature10);
		P(cool_temperature10);
		P(open_valve_min);
		time_server = server.arg(F("time_server"));
		time_zone = server.arg(F("time_zone"));
		save_preference();


			server.send(200, 
			F("text/plain"), F(R"(
			
				OK.

			)")		
			); });
		

	server.on(F("/close"), HTTP_POST, []() {
			valve_auto_status = 0;
			server.send(200, F("text/plain"), F(R"(OK.)")); });
	server.on(F("/auto"), HTTP_POST, []() {
			valve_auto_status = 1;
			server.send(200, F("text/plain"), F(R"(OK.)")); });
	server.on(F("/open"), HTTP_POST, []() {
			valve_auto_status = 2;
			server.send(200, F("text/plain"), F(R"(OK.)")); });

	server.on("/update", HTTP_GET, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/html", updateIndex);
	});

	server.on("/update", HTTP_POST, []() {
		server.sendHeader("Connection", "close");
		server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		ESP.restart();
	  }, []() {
		HTTPUpload& upload = server.upload();
		if (upload.status == UPLOAD_FILE_START) {
		  Serial.printf("Update: %s\n", upload.filename.c_str());
		  if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
		    Update.printError(Serial);
		  }
		} else if (upload.status == UPLOAD_FILE_WRITE) {
		  /* flashing firmware to ESP*/
		  if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
		    Update.printError(Serial);
		  }
		} else if (upload.status == UPLOAD_FILE_END) {
		  if (Update.end(true)) { //true to set the size to the current progress
		    Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
		  } else {
		    Update.printError(Serial);
		  }
		}
	  });


	server.begin();
	Serial.println(F("HTTP server started"));

}

void web_server_handle_client()
{
	server.handleClient();
}

//-----------------------------------------------------------------------//
static String wifi_get_ap_name() { return ssid; }

static String wifi_get_connection_info_string()
{
	String m;
	switch(WiFi.status())
	{
	case WL_CONNECTED:
		m = String(F("Connected to \"")) + wifi_get_ap_name() + F("\"");
		if((uint32_t)WiFi.localIP() == 0)
			m += String(F(", but no IP address got."));
		else
			m += String(F(", IP address is ")) + WiFi.localIP().toString() + F(" .");
		break;

	case WL_NO_SSID_AVAIL:
		m = String(F("AP \"")) + wifi_get_ap_name() + F("\" not available.");
		break;

	case WL_CONNECT_FAILED:
		m = String(F("Connection to \"")) + wifi_get_ap_name() + F("\" failed.");
		break;

	case WL_IDLE_STATUS: // ?? WTF ??
		m = String(F("Connection to \"")) + wifi_get_ap_name() + F("\" is idling.");
		break;

	case WL_DISCONNECTED:
		m = String(F("Disconnected from \"")) + wifi_get_ap_name() + F("\".");
		break;
	}
	return m;
}

static void reconnect()
{
    // We start by connecting to a WiFi network
    WiFi.mode(WIFI_OFF);
    WiFi.persistent(false); 
    WiFi.disconnect(true);
    delay(100);
	WiFi.setAutoReconnect(true);

	// TODO: ensure proper reset procedure

    WiFi.mode(WIFI_STA);
    WiFi.persistent(false); 
    WiFi.disconnect(true);
    delay(100);
	WiFi.setAutoReconnect(true);

    esp_wifi_set_ps(WIFI_PS_NONE); // disable power saving. WiFi power saving could cause the station never to be connected
    WiFi.begin(ssid, password);

    Serial.println();
    Serial.println();
    Serial.print("Wait for WiFi... ");
}

void setup()
{

	pinMode(VALVE_GATE_PIN, OUTPUT);
	digitalWrite(VALVE_GATE_PIN, 0);
	pinMode(VALVE_DRAIN_PIN, OUTPUT);
	digitalWrite(VALVE_DRAIN_PIN, 0);
/* unused 
	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, 0);
*/

	pinMode(LED_POWER_PIN, OUTPUT);
	digitalWrite(LED_POWER_PIN, 1);
	pinMode(LED_WIFI_PIN, OUTPUT);
	digitalWrite(LED_WIFI_PIN, 0);
	pinMode(LED_VALVE_PIN, OUTPUT);
	digitalWrite(LED_VALVE_PIN, 0);

	Serial.begin(115200);
	Serial.print("\r\n\r\nWelcome\r\n");
	init_preference();

	Serial.printf_P(PSTR("Built at %s %s\r\n"), _BuildInfo.date, _BuildInfo.time);
	Serial.printf_P(PSTR("Source version : %s\r\n"), _BuildInfo.src_version);
	Serial.printf_P(PSTR("Arduino core version : %s\r\n"), _BuildInfo.env_version);
 	Serial.printf_P(PSTR("Sensors initialization...\r\n"));

	sensors_bme280_init();

	reconnect();

	web_server_setup();


}


static void check_environment()
{
	static uint32_t next = millis() + 1000;
	if((int32_t)millis() - (int32_t)next >= 0)
	{
		sensors_bme280_get();
		Serial.printf_P(PSTR("Temp10:%d  Hum:%d  Press:%d\r\n"), bme280_result.temp_10, bme280_result.humidity, bme280_result.pressure);
		next =millis() + 1000;
	}
}

static void check_wifi()
{
	static int last_wifi_status = -1;
	static bool last_wifi_connected = false;
	static uint32_t last_disconnected = 0;
	static uint32_t last_reset = 0;
	static uint32_t last_status_showed = 0;

	// reset wifi every day
	/*
	if((int32_t)millis() - (int32_t)last_reset >= 1000*60*60*24)
	{
		last_reset = millis();
		reconnect();
	}
	*/

	// print wifi status
	if((int32_t)millis() - (int32_t)last_status_showed >= 5000)
	{
		Serial.println(wifi_get_connection_info_string());
		last_status_showed = millis();
	}



	// check wifi status and print
	int wifi_status = WiFi.status();
	if(last_wifi_status != wifi_status)
	{
		switch(wifi_status)
		{
		case WL_CONNECTED:
			configTzTime(time_zone.c_str(), time_server.c_str(), NULL, NULL);
			Serial.println("");
			Serial.println("WiFi connected");
			Serial.println("IP address: ");
			Serial.println(WiFi.localIP());
			last_wifi_connected = true;
			break;

		default:
			// check timeout
			if(last_wifi_connected)
				last_disconnected = millis(); // previously connected, but now disconnected
			last_wifi_connected = false;
			break;
		}
    }
    last_wifi_status = wifi_status;

	// if wifi seems not to be able to connect, reset wifi
	if(wifi_status != WL_CONNECTED &&
		(int32_t)millis() - (int32_t)last_disconnected >= 30000)
	{
		reconnect();
		last_disconnected = millis();
	}

	// show wifi led status
	if(wifi_status == WL_CONNECTED)
		digitalWrite(LED_WIFI_PIN, 1);
	else
		digitalWrite(LED_WIFI_PIN, 0);
}

void loop()
{
	check_wifi();
	check_environment();
	check_valve_open();
	server.handleClient();
}


