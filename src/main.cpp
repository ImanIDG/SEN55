#define SENSENET_DEBUG // enable debug on SerialMon
#define SerialMon Serial // if you need DEBUG SerialMon should be defined

#define FIRMWARE_TITLE "Arduino_Data_Collector"
#define FIRMWARE_VERSION "0.3.14"

#include <Arduino.h>
#include "sensenet.h"
#include "WiFi.h"
#include "Wire.h"
#include "SPI.h"
//#include "Preferences.h"
#include "Ticker.h"
#include <esp_task_wdt.h>
#include <ESP32Time.h>

#include "DEV_Config.h"
#include "L76X.h"

#include <SensirionI2CSen5x.h>
#include <MH411D.h>

#define WIFI_SSID "Sensenet_2.4G"
#define WIFI_PASS "Sensenet123"
#define TB_URL "tb.sensenet.ca"

// for COM7
//#define TOKEN "SGP4xESP32_3"

// for COM8
//#define TOKEN "SGP4xESP32_2"

// for COM10
#define TOKEN "GPSESP32"

ESP32Time internalRtc(0);  // offset in seconds GMT
Ticker restartTicker;
//Preferences preferences;
NetworkInterface wifiInterface("wifi", 2, 2);
NetworkInterfacesController networkController;
MQTTController mqttController;
MQTTOTA ota(&mqttController, 5120);

WiFiClient wiFiClient;

GNRMC GPS1;
Coordinates B_GPS;
int i = 0;
char buff_G[800] = {0};

void resetESP() {
    ESP.restart();
}

uint64_t getTimestamp() {
    if (internalRtc.getEpoch() < 946713600)
        return 0;
    uint64_t ts = internalRtc.getEpoch();
    ts = ts * 1000L;
    ts = ts + internalRtc.getMillis();
    return ts;
}

// Sampling interval in seconds
char errorMessage[32];

bool on_message(const String &topic, DynamicJsonDocument json) {
    Serial.print("Topic1: ");
    Serial.println(topic);
    Serial.print("Message1: ");
    Serial.println(json.as<String>());

    if (json.containsKey("shared")) {
        JsonObject sharedKeys = json["shared"].as<JsonObject>();
        for (JsonPair kv: sharedKeys)
            json[kv.key()] = sharedKeys[kv.key()];

    }

    if (json.containsKey("method")) {
        String method = json["method"].as<String>();

        bool handled = false;
        if (method.equalsIgnoreCase("restart_device")) {
            float seconds = 0;
            if (json["params"].containsKey("seconds"))
                seconds = json["params"]["seconds"];
            if (seconds == 0) seconds = 1;
            printDBGln("Device Will Restart in " + String(seconds) + " Seconds");
            restartTicker.once(seconds, resetESP);
            handled = true;
        }

        if (handled) {
            String responseTopic = String(topic);
            responseTopic.replace("request", "response");
            DynamicJsonDocument responsePayload(300);
            responsePayload["result"] = "true";
            mqttController.addToPublishQueue(responseTopic, responsePayload.as<String>(), true);
            return true;
        }
    }

    return false;
}

void connectToNetwork() {
    Serial.println("Added WiFi Interface");
    networkController.addNetworkInterface(&wifiInterface);

    networkController.setAutoReconnect(true, 10000);
    networkController.autoConnectToNetwork();
}

void connectToPlatform(Client &client, const bool enableOTA) {

    Serial.println("Trying to Connect Platform");
    mqttController.connect(client, "esp", TOKEN, "", TB_URL,
                           1883, on_message,
                           nullptr, [&]() {
                Serial.println("Connected To Platform");
                DynamicJsonDocument info(512);
                info["Token"] = TOKEN;
                info.shrinkToFit();
                mqttController.sendAttributes(info, true);
                if (enableOTA)
                    ota.begin(FIRMWARE_TITLE, FIRMWARE_VERSION);
                else
                    ota.stopHandleOTAMessages();

                DynamicJsonDocument requestKeys(512);
                requestKeys["sharedKeys"] = "desiredAllowSleep,desiredDisableIR,desiredSEN55TempOffset";
                requestKeys.shrinkToFit();
                mqttController.requestAttributesJson(requestKeys.as<String>());

                if (getTimestamp() == 0) {
                    DynamicJsonDocument requestTime(512);
                    requestTime["method"] = "requestTimestamp";
                    requestTime.shrinkToFit();
                    mqttController.requestRPC(requestTime.as<String>(),
                                              [](const String &rpcTopic, const DynamicJsonDocument &rpcJson) -> bool {
                                                  Serial.print("Updating Internal RTC to: ");
                                                  Serial.println(rpcJson.as<String>());
                                                  uint64_t tsFromCloud = rpcJson["timestamp"].as<uint64_t>();
                                                  tsFromCloud = tsFromCloud / 1000;
                                                  internalRtc.setTime(tsFromCloud);
                                                  Serial.print("Internal RTC updated to: ");
                                                  Serial.println(internalRtc.getDateTime(true));
                                                  getTimestamp();
                                                  return true;
                                              });
                } else {
                    Serial.print("Internal RTC updated to: ");
                    Serial.println(internalRtc.getDateTime(true));
                }
            });
}

int retry = 0;

void initInterfaces() {
    retry = 0;
    wifiInterface.setTimeoutMs(30000);
    wifiInterface.setConnectInterface([]() -> bool {
        Serial.println(String("Connecting To WiFi ") + WIFI_SSID);
        WiFi.mode(WIFI_MODE_NULL);
        delay(2000);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        return true;
    });
    wifiInterface.setConnectionCheckInterfaceInterface([]() -> bool {
        return WiFi.status() == WL_CONNECTED;
    });
    wifiInterface.OnConnectingEvent([]() {
        Serial.print(".");
    }, 500);
    wifiInterface.OnConnectedEvent([]() {
        retry = 0;
        Serial.println(String("Connected to WIFI with IP: ") + WiFi.localIP().toString());
        connectToPlatform(wiFiClient, true);
        DynamicJsonDocument data(200);
        data["Connection Type"] = "WIFI";
        data["IP"] = WiFi.localIP().toString();
        data.shrinkToFit();
    });
    wifiInterface.OnTimeoutEvent([]() {
        retry++;
        Serial.println("WiFi Connecting Timeout! retrying for " + String(retry) + " Times");
        WiFi.mode(WIFI_MODE_NULL);

//        if (retry >= 20)
//            ESP.restart();
    });
}

uint64_t lastSEN55_MH_411D = 0;

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;
WinsenSensors myMH411D;

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}

void loopSEN55_MH_411D(DynamicJsonDocument &data) {
    uint16_t error;
    char errorMessage[256];

    if( myMH411D.startMeasure() != 0) {
        Serial.print("CO2 concentration: ");
        int foodata = myMH411D.getGasConcentration();
        Serial.println(foodata, DEC);
        data["MH411D"] = String(foodata);
    }
    else {
        Serial.print("Measure failed !!");
        Serial.print("myMH411D.getGasConcentration()");
        error = 0;
        errorToString(error, errorMessage, 256);
    }
    // Read Measurement
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;

    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("MassConcentrationPm1p0:");
        Serial.print(massConcentrationPm1p0);
        data["massConcentrationPm1p0"] = String(massConcentrationPm1p0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm2p5:");
        Serial.print(massConcentrationPm2p5);
        data["massConcentrationPm2p5"] = String(massConcentrationPm2p5);
        Serial.print("\t");
        Serial.print("MassConcentrationPm4p0:");
        Serial.print(massConcentrationPm4p0);
        data["massConcentrationPm4p0"] = String(massConcentrationPm4p0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm10p0:");
        Serial.print(massConcentrationPm10p0);
        data["massConcentrationPm10p0"] = String(massConcentrationPm10p0);
        Serial.print("\t");
        Serial.print("AmbientHumidity:");
        if (isnan(ambientHumidity)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientHumidity);
            data["ambientHumidity"] = String(ambientHumidity);
        }
        Serial.print("\t");
        Serial.print("AmbientTemperature:");
        if (isnan(ambientTemperature)) {
            Serial.print("n/a");
        } else {
            Serial.print(ambientTemperature);
            data["ambientTemperature"] = String(ambientTemperature);
        }
        Serial.print("\t");
        Serial.print("VocIndex:");
        if (isnan(vocIndex)) {
            Serial.print("n/a");
        } else {
            Serial.print(vocIndex);
            data["vocIndex"] = String(vocIndex);
        }
        Serial.print("\t");
        Serial.print("NoxIndex:");
        if (isnan(noxIndex)) {
            Serial.println("n/a");
        } else {
            Serial.println(noxIndex);
            data["noxIndex"] = String(noxIndex);
        }
    }
}

void core0Loop(void *parameter) {
    //Dont do anything 1
    esp_task_wdt_init(600, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    //Dont do anything1

    lastSEN55_MH_411D = Uptime.getMilliseconds();
    uint64_t now = Uptime.getMilliseconds();
    DynamicJsonDocument data(5120);
    if (now - lastSEN55_MH_411D > 60000) {
        lastSEN55_MH_411D = now;
        loopSEN55_MH_411D(data);
    }

    if (data.size() > 0 && getTimestamp() > 0) {
        data.shrinkToFit();
        Serial.println("Data: " + data.as<String>());
        mqttController.sendTelemetry(data, true, getTimestamp());
    }
    delayMicroseconds(1);
    esp_task_wdt_reset();
}

void setup() {
    //Dont do anything in setup
    //Add setup SEN55
    internalRtc.setTime(1000);
    btStop();
    esp_task_wdt_init(60, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    esp_task_wdt_reset();

    Serial.begin(9600);
//    preferences.begin("Configs", false);
//    Serial.println("Hello from: " + preferences.getString("token", "not-set"));
    mqttController.init();
    mqttController.sendSystemAttributes(true);
    initInterfaces();
    Wire.begin();
    esp_task_wdt_reset();

    uint16_t error;
    char errorMessage[256];

    sen5x.begin(Wire);
    myMH411D.begin(&Serial1);
    error = sen5x.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
#endif

    // set a temperature offset in degrees celsius
    // Note: supported by SEN54 and SEN55 sensors
    // By default, the temperature and humidity outputs from the sensor
    // are compensated for the modules self-heating. If the module is
    // designed into a device, the temperature compensation might need
    // to be adapted to incorporate the change in thermal coupling and
    // self-heating of other device components.
    //
    // A guide to achieve optimal performance, including references
    // to mechanical design-in examples can be found in the app note
    // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
    // Please refer to those application notes for further information
    // on the advanced compensation settings used
    // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and
    // `setRhtAccelerationMode`.
    //
    // Adjust tempOffset to account for additional temperature offsets
    // exceeding the SEN module's self heating.
    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature Offset set to ");
        Serial.print(tempOffset);
        Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
    delay(1000);  // needed on some Arduino boards in order to have Serial ready

    esp_task_wdt_reset();
    connectToNetwork();
    esp_task_wdt_reset();

    delay(1000);
    xTaskCreatePinnedToCore(
            core0Loop, // Function to implement the task
            "Core0Loop", // Name of the task
            10000, // Stack size in words
            NULL,  // Task input parameter
            0, // Priority of the task
            NULL,  // Task handle.
            0); // Core where the task should run
    esp_task_wdt_reset();
}

uint64_t core1Heartbeat;

void loop() {
    //Dont do anything
    esp_task_wdt_reset();

    if (Serial.available()) {
        if (Serial.readString().indexOf("reboot") >= 0)
            resetESP();
    }

    if (networkController.getCurrentNetworkInterface() != nullptr &&
        networkController.getCurrentNetworkInterface()->lastConnectionStatus()) {
        mqttController.loop();
    }

    networkController.loop();

    if ((Uptime.getSeconds() - core1Heartbeat) > 10) {
        core1Heartbeat = Uptime.getSeconds();
        printDBGln("Core 1 Heartbeat");
    }
}
