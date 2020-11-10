#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <SimpleTimer.h>
#include <mcp4652.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266TimerInterrupt.h>

const String DEFAULT_SSID = "SmartPower2_" + String(ESP.getChipId(), HEX);
const String DEFAULT_PASSWORD = "12345678";

const uint32_t PRINT_SERIAL_DATA_INTERVAL_MS = 1000;
const uint32_t PRINT_LCD_DATA_INTERVAL_MS = 1000;
const uint32_t POWER_READING_INTERVAL_MS = 5;

#define CONSOLE_PORT 23

WiFiServer logServer(CONSOLE_PORT);
WiFiClient logClient;

#define USE_SERIAL Serial
ESP8266Timer ITimer;
void ICACHE_RAM_ATTR TimerHandler(void);

#define MAX_LCD_SSID_LENGTH  12
#define MAX_LCD_IP_LENGTH    14

#define ON 0
#define OFF 1
#define HOME 0
#define SETTINGS 1

#define POWER     D6
#define POWERLED  D1
#define BTN_ONOFF D7
#define I2C_SDA   D2
#define I2C_SCL   D5

#define SET_DEFAULT_VOLTAGE 'v'
#define SET_VOLTAGE         'w'
#define SAVE_NETWORKS       'n'
#define CMD_ONOFF           'o'
#define SET_AUTORUN         'a'
#define PAGE_STATE          'p'
#define DATA_PVI            'd'
#define MEASUREWATTHOUR     'm'
#define FW_VERSION          'f'

#define FWversion 1.6

uint8_t onoff = OFF;
unsigned char measureWh;
float setVoltage = 5.1f;
unsigned char connectedWeb;
unsigned char connectedLCD;
unsigned char autorun;

unsigned char D4state;
unsigned char D1state;

char ssid[20];
char password[20];

struct ReadingType {
    uint32_t time;
    float volt;
    float ampere;
    float watt;
    float watth;
};
ReadingType readings;

#define MAX_SRV_CLIENTS 1
#define WIFI_WAIT_TIMEOUT 20

IPAddress ip = IPAddress(192, 168, 4, 1);

void startWiFi() {
    USE_SERIAL.printf("Connecting to \"%s\"...", ssid);
    WiFi.mode(WIFI_STA);
    //WiFi.begin(ssid, password);
    WiFi.begin("LC012-RPi4-Router", "........");
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter != WIFI_WAIT_TIMEOUT) {
        delay(1000);
        USE_SERIAL.print(".");
        counter++;
    }
    if (counter == WIFI_WAIT_TIMEOUT) {
        ESP.restart();
    } else {
        ip = WiFi.localIP();
        USE_SERIAL.print("Connected!\nIP: ");
        USE_SERIAL.println(ip);
    }
}

// lcd slave address are 0x27 or 0x3f
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct client_sp2 {
    uint8_t connected;
    uint8_t page;
};

struct client_sp2 client_sp2[5];

void setup() {
    USE_SERIAL.begin(115200);
    //USE_SERIAL.setDebugOutput(true);
    USE_SERIAL.println("smartpower2_sta_slim release 20201110");
    pinMode(POWERLED, OUTPUT);
    
    startWiFi();

    Wire.begin(I2C_SDA, I2C_SCL);
    mcp4652_init();
    ina231_configure();

    pinMode(BTN_ONOFF, INPUT);
    attachInterrupt(digitalPinToInterrupt(BTN_ONOFF), pinChanged, CHANGE);

    for (uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n\r", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    USE_SERIAL.println("");
    USE_SERIAL.println("##############################");
    USE_SERIAL.printf("SmartPower2 v");
    USE_SERIAL.print(FWversion);
    USE_SERIAL.println(" (Serial Interface)");
    USE_SERIAL.println("##############################");
    USE_SERIAL.println("");

    initSmartPower();
    lcd_status();

    // Log
    logServer.begin();
    logServer.setNoDelay(true);

    // Interval in microsecs
    if (ITimer.attachInterruptInterval(POWER_READING_INTERVAL_MS * 1000, TimerHandler)) {
        Serial.println("Starting  ITimer OK, millis() = " + String(millis()));
    } else {
        Serial.println("Can't set ITimer correctly. Select another freq. or interval");
    }
}

void refreshWebServer() {
    if (logServer.hasClient()) {
        // A connection attempt is being made
        USE_SERIAL.printf("A connection attempt is being made.\n");
        if (!logClient) {
            // This is the first client to connect
            logClient = logServer.available();
            USE_SERIAL.printf("This is the first client to connect.\n");
        } else {
            if (!logClient.connected()) {
                // A previous client has disconnected.
                //  Connect the new client.
                logClient.stop();
                logClient = logServer.available();
                USE_SERIAL.printf("A previous client has disconnected.\n");
            } else {
                // A client connection is already in use.
                // Drop the new connection attempt.
                WiFiClient tempClient = logServer.available();
                tempClient.stop();
                USE_SERIAL.printf("A client connection is already in use.\n");
            }
        }
    }
}

void initSmartPower(void) {
    onoff = !autorun;


    mcp4652_write(WRITE_WIPER0, quadraticRegression(setVoltage));
    pinMode(POWER, OUTPUT);
    digitalWrite(POWER, onoff);

    pinMode(D4, OUTPUT);
    digitalWrite(D4, HIGH);
}

// lcd slave address are 0x27 or 0x3f
unsigned char lcdSlaveAddr;
int lcd_available(void) {
    Wire.beginTransmission(0x27);
    if (!Wire.endTransmission()) {
        lcdSlaveAddr = 0x27;
        return lcdSlaveAddr;
    } else {
        Wire.beginTransmission(0x3f);
        if (!Wire.endTransmission()) {
            lcdSlaveAddr = 0x3f;
            return lcdSlaveAddr;
        }
    }

    lcdSlaveAddr = 0;

    return 0;
}

void lcd_status(void) {
    if (lcd_available() > 0) {
        if (!connectedLCD) {
            lcd = LiquidCrystal_I2C(lcdSlaveAddr, 16, 2);
            lcd.init();
            lcd.backlight();
            connectedLCD = 1;
        }
    } else {
        connectedLCD = 0;
    }
}

void printPower_LCD(void) {
    float rwatth;
    lcd.setCursor(0, 0);
    lcd.print(readings.volt, 3);
    lcd.print(" V ");
    lcd.print(readings.ampere, 3);
    lcd.print(" A  ");

    lcd.setCursor(0, 1);
    if (readings.watt < 10) {
        lcd.print(readings.watt, 3);
    } else {
        lcd.print(readings.watt, 2);
    }
    lcd.print(" W ");

    rwatth = readings.watth / 3600;
    if (rwatth < 10) {
        lcd.print(rwatth, 3);
        lcd.print(" ");
    } else if (rwatth < 100) {
        lcd.print(rwatth, 2);
        lcd.print(" ");
    } else if (rwatth < 1000) {
        lcd.print(rwatth, 1);
        lcd.print(" ");
    } else {
        lcd.print(rwatth / 1000, 0);
        lcd.print(" K");
    }
    lcd.print("Wh     ");
}

uint8_t cnt_ssid;
int8_t cnt_ip;
int8_t cursor_ssid;
int8_t cursor_ip;
void printInfo_LCD(void) {
    String str;
    int i;
    cnt_ssid = String(ssid).length();
    lcd.setCursor(0, 0);
    lcd.print("SSID:");
    str = String(ssid);
    if (cnt_ssid < MAX_LCD_SSID_LENGTH) {
        lcd.print(str);
        for (i = 0; i < MAX_LCD_SSID_LENGTH; i++) {
            lcd.print(" ");
        }
    } else {
        if ((cursor_ssid - 5) == cnt_ssid) {
            cursor_ssid = 0;
        }
        lcd.print(str.substring(cursor_ssid++));
        if ((cnt_ssid - cursor_ssid) < MAX_LCD_SSID_LENGTH - 5) {
            if (cnt_ssid < cursor_ssid) {
                for (i = 0; i < 6 - (cursor_ssid - cnt_ssid); i++)
                    lcd.print(" ");
            } else {
                lcd.print("     ");
            }
            lcd.print(str);
        } else if ((cnt_ssid - cursor_ssid) < MAX_LCD_SSID_LENGTH) {
            for (i = 0; i < MAX_LCD_SSID_LENGTH - (cnt_ssid - cursor_ssid); i++)
                lcd.print(" ");
        }
    }

    lcd.setCursor(0, 1);
    lcd.print("IP:");
    cnt_ip = ip.toString().length();
    if (cnt_ip < MAX_LCD_IP_LENGTH) {
        lcd.print(ip.toString());
        for (i = 0; i < MAX_LCD_IP_LENGTH - cnt_ip; i++)
            lcd.print(" ");
    } else {
        if ((cursor_ip - 5) == cnt_ip) {
            cursor_ip = 0;
        }
        lcd.print(ip.toString().substring(cursor_ip++));
        if ((cnt_ip - cursor_ip) < MAX_LCD_IP_LENGTH - 5) {
            if (cnt_ip < cursor_ip) {
                for (i = 0; i < 6 - (cursor_ip - cnt_ip); i++)
                    lcd.print(" ");
            } else {
                lcd.print("     ");
            }
            lcd.print(ip.toString());
        } else if ((cnt_ip - cursor_ip) < MAX_LCD_IP_LENGTH) {
            for (i = 0; i < MAX_LCD_IP_LENGTH - (cnt_ip - cursor_ip); i++)
                lcd.print(" ");
        }
    }
}

void readPower(void) {
    readings.volt = ina231_read_voltage();
    readings.watt = ina231_read_power();
    readings.ampere = ina231_read_current();
}

float a = 0.0000006562;
float b = 0.0022084236;
float c = 4.08;
int quadraticRegression(float volt) {
    float d;
    float root;
    d = b * b - a * (c - volt);
    root = (-b + sqrt(d)) / a;
    if (root < 0) {
        root = 0;
    } else if (root > 255) {
        root = 255;
    }
    return root;
}

unsigned long btnPress;
unsigned long btnRelese = 1;
unsigned long currtime;
unsigned char btnChanged;
unsigned char resetCnt;
unsigned char swlock;
ICACHE_RAM_ATTR void pinChanged() {
    if ((millis() - currtime) > 30) {
        swlock = 0;
    }

    if (!swlock) {
        if (!digitalRead(BTN_ONOFF) && (btnPress == 0)) {
            swlock = 1;
            currtime = millis();
            btnPress = 1;
            btnRelese = 0;
        }
    }

    if (!swlock) {
        if (digitalRead(BTN_ONOFF) && (btnRelese == 0)) {
            swlock = 1;
            currtime = millis();
            btnRelese = 1;
            btnPress = 0;
            btnChanged = 1;
            onoff = !onoff;
            readings.watth = 0;
            digitalWrite(POWER, onoff);
            digitalWrite(POWERLED, LOW);
        }
    }
}

void readSystemReset() {
    if (!digitalRead(BTN_ONOFF) && (btnPress == 1)) {
        if (resetCnt++ > 5) {
            USE_SERIAL.println("System Reset!!");
            resetCnt = 0;
        }
    } else {
        resetCnt = 0;
    }
}

void loop() {
    static uint32_t lastRunWebServerTime = 0;
    static uint32_t lastUpdateTelnetTime = 0;

    ReadingType local_readings = readings;
    
    String datastr = String(local_readings.time) + " " + 
                     String(local_readings.volt, 3) + "," + String(local_readings.ampere, 3) + "," +
                     String(local_readings.watt, 3) + "," + String(local_readings.watth / 3600, 3) + "\r\n";

    if (lastUpdateTelnetTime != local_readings.time) {
        if (logClient && logClient.connected()) {
            logClient.write(datastr.c_str());
            while (logClient.available()) {
                logClient.read();
            }
        }
        lastUpdateTelnetTime = local_readings.time;
    }

    if (millis() - lastRunWebServerTime >= 1000) {
        refreshWebServer();

        if (onoff == ON) {
            digitalWrite(POWERLED, D1state = !D1state);
            Serial.print(datastr.c_str());
        }

        if (connectedLCD) {
            if (onoff == ON)
                printPower_LCD();
            else if (onoff == OFF)
                printInfo_LCD();
        }

        if (btnChanged) {
            if (onoff == OFF) {
                readings.watt = readings.volt = readings.ampere = readings.watth = 0;
            }
            btnChanged = 0;
        }
        lcd_status();
        //wifi_connection_status();
        readSystemReset();

        lastRunWebServerTime = millis();
    }
}

void ICACHE_RAM_ATTR TimerHandler(void) {
    readings.time = millis();
    readPower();
}
