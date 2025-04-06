#include <M5Dial.h>
#include <M5Unified.h>
#include <time.h>
#include "esp_mac.h"
#include "Adafruit_FRAM_I2C.h"
#include <Arduino.h>
#include "bme68xLibrary.h"
#include <Wire.h>
#include "WiFi.h"
#include <bsec2.h>
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const char* apssid = "Czujniczek";
const char* appassword = "12345678";
char ssid[32] = "";        
char password[32] = "";
#define SDA_PIN 13
#define SCL_PIN 15
int framIndex = 0;
AsyncWebServer server(80);
bool isConfigured = false;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);
#define FRAM_SIZE 32000
#define FRAM_HEADER_SIZE sizeof(uint32_t)
#define MAX_ENTRIES 100
#define IAQ_EXCELLENT 0x07E0 
#define IAQ_GOOD 0x9E54      
#define IAQ_MODERATE 0xFFE0  
#define IAQ_POOR 0xFB40      
#define IAQ_UNHEALTHY 0xF800  
#define IAQ_HAZARDOUS 0x8010 

uint16_t getIAQColor(float value) {
    if (value <= 50) return IAQ_EXCELLENT;
    if (value <= 100) return IAQ_GOOD;
    if (value <= 150) return IAQ_MODERATE;
    if (value <= 200) return IAQ_POOR;
    if (value <= 300) return IAQ_UNHEALTHY;
    return IAQ_HAZARDOUS;
}

struct SensorData {
    int hour;
    int minute;
    int second;
    float temperature;
    float pressure;
    float humidity;
    float co2;
    float voc;
};

Bme68x bme;
bme68xData data;
Adafruit_FRAM_I2C fram;
Bsec2 envSensor;

void checkBsecStatus(Bsec2 bsec);
void displaySensorData(float temperature, float pressure, float humidity, float co2, float voc);
void initFRAM();
void saveToFRAM(float temperature, float pressure, float humidity, float co2, float voc);
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
void displayTime();
void getLatestData(AsyncResponseStream *response);
void readFromFRAM();
void resetFRAMIndex();

/**
 * Checks and reports the status of BSEC sensor
 * @param bsec {Bsec2} BSEC sensor instance
 */

void checkBsecStatus(Bsec2 bsec) {
    if (bsec.status < BSEC_OK) {
        Serial.println("BSEC error code : " + String(bsec.status));
        delay(1000);
    } else if (bsec.status > BSEC_OK) {
        Serial.println("BSEC warning code : " + String(bsec.status));
    }
    if (bsec.sensor.status < BME68X_OK) {
        Serial.println("BME68X error code : " + String(bsec.sensor.status));
        delay(1000);
    } else if (bsec.sensor.status > BME68X_OK) {
        Serial.println("BME68X warning code : " + String(bsec.sensor.status));
    }
}

/**
 * Displays sensor readings on the M5Dial screen
 * @param temperature {float} Temperature in Celsius
 * @param pressure {float} Atmospheric pressure in hPa
 * @param humidity {float} Relative humidity percentage
 * @param co2 {float} CO2 equivalent in ppm
 * @param voc {float} Volatile Organic Compounds in ppm
 */
void displaySensorData(float temperature, float pressure, float humidity, float co2, float voc) {

    M5Dial.Display.setTextSize(2.5);

    uint16_t tempColor = getIAQColor(temperature);
    M5Dial.Display.setTextColor(tempColor, BLACK);
    M5Dial.Display.setCursor(30, 70);
    M5Dial.Display.print(temperature, 1);
    M5Dial.Display.print(" C");

    M5Dial.Display.setTextColor(DARKGREEN, BLACK);
    M5Dial.Display.setCursor(55, 100);
    M5Dial.Display.print(pressure, 1);
    M5Dial.Display.print(" hPa");

    M5Dial.Display.setTextColor(DARKGREEN, BLACK);
    M5Dial.Display.setCursor(135, 70);
    M5Dial.Display.print(humidity, 1);
    M5Dial.Display.print("%");

    uint16_t co2Color = getIAQColor(co2/10);
    M5Dial.Display.setTextColor(co2Color, BLACK);
    M5Dial.Display.setCursor(30, 130);
    M5Dial.Display.print("CO2: ");
    M5Dial.Display.print(co2, 1);
    M5Dial.Display.print("ppm");

    uint16_t vocColor = getIAQColor(voc * 50);
    M5Dial.Display.setTextColor(vocColor, BLACK);
    M5Dial.Display.setCursor(30, 160);
    M5Dial.Display.print("VOC: ");
    M5Dial.Display.print(voc, 2);
    M5Dial.Display.print("ppm");
}

/**
 * Initializes FRAM memory and recovers the last saved index
 * Handles index overflow by resetting to 0 if MAX_ENTRIES is exceeded
 */
void initFRAM() {
    if (!fram.begin()) {
        Serial.println("Could not initialize FRAM!");
        return;
    }
  
    uint32_t savedIndex;
    fram.read(0, (uint8_t*)&savedIndex, sizeof(savedIndex));
    
    if (savedIndex >= MAX_ENTRIES) {
        savedIndex = 0;
        fram.write(0, (uint8_t*)&savedIndex, sizeof(savedIndex));
    }
    
    framIndex = savedIndex;
    Serial.printf("Recovered FRAM index: %d\n", framIndex);
}

/**
 * Saves current sensor readings to FRAM memory with timestamp
 * @param temperature {float} Temperature in Celsius
 * @param pressure {float} Atmospheric pressure in hPa
 * @param humidity {float} Relative humidity percentage
 * @param co2 {float} CO2 equivalent in ppm
 * @param voc {float} Volatile Organic Compounds in ppm
 */
void saveToFRAM(float temperature, float pressure, float humidity, float co2, float voc) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    int dataAddress = FRAM_HEADER_SIZE + (framIndex * sizeof(SensorData));

    SensorData data = {
        .hour = timeinfo.tm_hour,
        .minute = timeinfo.tm_min,
        .second = timeinfo.tm_sec,
        .temperature = temperature,
        .pressure = pressure,
        .humidity = humidity,
        .co2 = co2,
        .voc = voc
    };

    Serial.printf("Zapisywanie: %02d:%02d:%02d, Temp: %.1f, Ciśn: %.1f, Wilg: %.1f, CO2: %.1f, VOC: %.2f\n",
                  data.hour, data.minute, data.second, data.temperature, data.pressure, data.humidity, data.co2, data.voc);

    fram.write(dataAddress, (uint8_t*)&data, sizeof(SensorData));

    framIndex = (framIndex + 1) % MAX_ENTRIES;

    fram.write(0, (uint8_t*)&framIndex, sizeof(framIndex));
}

/**
 * Callback function for new sensor data processing
 * Validates readings and updates display
 * @param data {bme68xData} Raw BME68X sensor data
 * @param outputs {bsecOutputs} Processed BSEC outputs
 * @param bsec {Bsec2} BSEC sensor instance
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
    if (!outputs.nOutputs) {
        return;
    }

    float temperature = 0.0, pressure = 0.0, humidity = 0.0, co2 = 0.0, voc = 0.0;

    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id) {
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                temperature = output.signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                pressure = output.signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                humidity = output.signal;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                co2 = output.signal;
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:  
                voc = output.signal;
                break;
            default:
                break;
        }
    }

    Serial.printf("Dane: Temp: %.2f°C, Ciśn: %.2f hPa, Wilg: %.2f%%, CO2: %.2f ppm, VOC: %.2f ppm\n",
                  temperature, pressure, humidity, co2, voc);

    if (pressure < 850.0 || pressure > 1100.0) {
        Serial.println("Błędny odczyt ciśnienia!");
        return;
    }
    if (temperature < -40.0 || temperature > 85.0) {
        Serial.println("Błędny odczyt temperatury!");
        return;
    }
    if (humidity < 0.0 || humidity > 100.0) {
        Serial.println("Błędny odczyt wilgotności!");
        return;
    }

    displaySensorData(temperature, pressure, humidity, co2, voc);
    
    static unsigned long lastWriteTime = 0;
    const unsigned long WRITE_INTERVAL = 600000;

    if (millis() - lastWriteTime >= WRITE_INTERVAL) {
        saveToFRAM(temperature, pressure, humidity, co2, voc);
        lastWriteTime = millis();
    }
}

/**
 * Displays current time on M5Dial screen
 * Updates every second using NTP time
 */
void displayTime() {

    time_t now = timeClient.getEpochTime();
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);

    char timeString[16];
    if (timeinfo.tm_year > 70) {
        strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);

        M5Dial.Display.setTextSize(2.5);
        M5Dial.Display.setTextColor(MAGENTA, BLACK);
        M5Dial.Display.setCursor(70, 28);
        M5Dial.Display.println(timeString); 
    }
}

/**
 * Generates JSON response with latest sensor readings from FRAM
 * @param response {AsyncResponseStream*} Async web response stream for data
 */
void getLatestData(AsyncResponseStream *response) {
    DynamicJsonDocument doc(16384);
    JsonArray array = doc.to<JsonArray>();
    
    for (uint32_t i = 0; i < MAX_ENTRIES; i++) {
        SensorData data;
        int dataAddress = FRAM_HEADER_SIZE + (i * sizeof(SensorData));
        fram.read(dataAddress, (uint8_t*)&data, sizeof(SensorData));
        
        if (data.hour != 0 || data.minute != 0 || data.second != 0) {
            JsonObject obj = array.add<JsonObject>();
            char timeStr[9];
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", data.hour, data.minute, data.second);
            obj["timestamp"] = timeStr;
            obj["temperature"] = data.temperature;
            obj["pressure"] = data.pressure;
            obj["humidity"] = data.humidity;
            obj["co2"] = data.co2;
            obj["voc"] = data.voc;
        }
    }
    
    serializeJson(doc, *response);
}

/**
 * Reads and prints all stored sensor data from FRAM memory
 * Used for debugging and data verification
 */
void readFromFRAM() {
    for (int i = 0; i < MAX_ENTRIES; i++) {
        int address = FRAM_HEADER_SIZE + (i * sizeof(SensorData));
        
        SensorData data;
        fram.read(address, (uint8_t*)&data, sizeof(SensorData));
        
        Serial.print("Odczyt numer: ");
        Serial.print(i);
        Serial.print(" - Czas: ");
        Serial.print(data.hour);
        Serial.print(":");
        if (data.minute < 10) Serial.print("0");
        Serial.print(data.minute);
        Serial.print(":");
        if (data.second < 10) Serial.print("0");
        Serial.print(data.second);
        Serial.println(" (hh:mm:ss)");

        Serial.print("  Temp.: ");
        Serial.print(data.temperature, 1);
        Serial.println(" C");
        Serial.print("  Cisn.: ");
        Serial.print(data.pressure, 1);
        Serial.println(" hPa");
        Serial.print("  Wilg.: ");
        Serial.print(data.humidity, 1);
        Serial.println(" %");
        Serial.print("  CO2: ");
        Serial.print(data.co2, 1);
        Serial.println(" ppm");
    }
}

/**
 * Resets FRAM index to 0
 * Used when memory needs to be reinitialized
 */
void resetFRAMIndex() {
    framIndex = 0;
    fram.write(0, (uint8_t*)&framIndex, sizeof(framIndex));
    Serial.println("Indeks FRAM zresetowany do 0.");
}

/**
 * Generates HTML for WiFi setup page
 * @return {String} Complete HTML page with setup form
 */
String wifiSetupPage() {
  return R"rawliteral(
    <!DOCTYPE html>
    <html lang="pl">
    <head>
        <meta charset="UTF-8">
        <title>Konfiguracja Wi-Fi</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; }
            .container { max-width: 400px; margin: 0 auto; }
            input[type="text"], input[type="password"] {
                width: 100%;
                padding: 8px;
                margin: 8px 0;
            }
            input[type="submit"] {
                background-color: #4CAF50;
                color: white;
                padding: 10px 15px;
                border: none;
                border-radius: 4px;
                cursor: pointer;
            }
            #status {
                margin-top: 20px;
                padding: 10px;
                border-radius: 4px;
            }
        </style>
    </head>
    <body>
        <div class="container">
            <h2>Konfiguracja Wi-Fi</h2>
            <form id="wifiForm" onsubmit="connect(event)">
                <label for="ssid">SSID:</label>
                <input type="text" id="ssid" name="ssid" required><br>
                <label for="password">Hasło:</label>
                <input type="password" id="password" name="password" required><br><br>
                <input type="submit" value="Połącz">
            </form>
            <div id="status"></div>
        </div>
        <script>
            function connect(e) {
                e.preventDefault();
                const form = document.getElementById('wifiForm');
                const status = document.getElementById('status');
                const formData = new FormData(form);

                status.innerHTML = 'Łączenie...';
                status.style.backgroundColor = '#fff3cd';

                fetch('/connect', {
                    method: 'POST',
                    body: formData
                })
                .then(response => response.text())
                .then(() => {
                    checkStatus();
                })
                .catch(error => {
                    status.innerHTML = 'Błąd połączenia: ' + error;
                    status.style.backgroundColor = '#f8d7da';
                });
            }

            function checkStatus() {
                const status = document.getElementById('status');
                let attempts = 0;
                const maxAttempts = 30; // 30 seconds timeout

                const checkInterval = setInterval(() => {
                    attempts++;
                    fetch('/status')
                        .then(response => response.json())
                        .then(data => {
                            if (data.connected) {
                                clearInterval(checkInterval);
                                status.innerHTML = 'Połączono! IP: ' + data.ip;
                                status.style.backgroundColor = '#d4edda';
                                setTimeout(() => {
                                    window.location.href = '/dashboard';
                                }, 2000);
                            } else if (attempts >= maxAttempts) {
                                clearInterval(checkInterval);
                                status.innerHTML = 'Timeout - nie udało się połączyć';
                                status.style.backgroundColor = '#f8d7da';
                            }
                        })
                        .catch(() => {
                            if (attempts >= maxAttempts) {
                                clearInterval(checkInterval);
                                status.innerHTML = 'Błąd połączenia';
                                status.style.backgroundColor = '#f8d7da';
                            }
                        });
                }, 1000);
            }
        </script>
    </body>
    </html>
  )rawliteral";
}

/**
 * Configures web server routes and handlers
 * Sets up endpoints for dashboard, WiFi setup, and data API
 */
void setupServer() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS initialization error!");
        return;
    }


    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (WiFi.status() != WL_CONNECTED) {

            Serial.println("Serwowanie strony konfiguracji Wi-Fi...");
            request->send(200, "text/html", wifiSetupPage());
        } else {

            Serial.println("Urządzenie podłączone do Wi-Fi, ale brak przekierowania...");
            String message = "<html><body>"
                             "<h2>Urządzenie jest podłączone do Wi-Fi!</h2>"
                             "<p>Adres IP: " + WiFi.localIP().toString() + "</p>"
                             "<p><a href='/dashboard'>Przejdź do dashboardu</a></p>"
                             "</body></html>";
            request->send(200, "text/html", message);
        }
    });

    server.on("/dashboard", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/connect", HTTP_POST, [](AsyncWebServerRequest *request) {
        String new_ssid, new_password;
        if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
            new_ssid = request->getParam("ssid", true)->value();
            new_password = request->getParam("password", true)->value();

            strncpy(ssid, new_ssid.c_str(), sizeof(ssid) - 1);
            ssid[sizeof(ssid) - 1] = 0;
            strncpy(password, new_password.c_str(), sizeof(password) - 1);
            password[sizeof(password) - 1] = 0;

            WiFi.disconnect();
            delay(1000);
            WiFi.begin(ssid, password);
            isConfigured = true;
            
            request->send(200, "text/plain", "Wi-Fi konfigurowane...");
        } else {
            request->send(400, "text/plain", "Brakuje SSID lub hasła");
        }
    });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        DynamicJsonDocument doc(128);
        doc["connected"] = (WiFi.status() == WL_CONNECTED);
        doc["ip"] = WiFi.localIP().toString();
        serializeJson(doc, *response);
        request->send(response);
    });

    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        getLatestData(response);
        request->send(response);
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(204);
    });

    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    server.begin();
}

void setup() {

    auto cfg = M5.config();
    M5Dial.begin(cfg, true, false);
    Serial.begin(115200);
    delay(1000);

    if(!SPIFFS.begin(true)) {
        Serial.println("SPIFFS initialization error!");
        return;
    }

    Wire.begin(SDA_PIN, SCL_PIN);
    initFRAM();

    M5Dial.Display.setTextSize(2);
    M5Dial.Display.setTextColor(WHITE, BLACK);
    M5Dial.Display.fillScreen(BLACK);

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(apssid, appassword);
        Serial.print("Access Point IP: ");
        Serial.println(WiFi.softAPIP());

        setupServer();
    } else {
        Serial.println("Already connected to Wi-Fi!");
    }

    if (!envSensor.begin(BME68X_I2C_ADDR_HIGH, Wire)) {
        Serial.println("BSEC initialization error!");
        checkBsecStatus(envSensor);
        delay(1000);
    }

    bsec_virtual_sensor_t sensorList[] = {
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT
    };

    if (!envSensor.updateSubscription(sensorList, sizeof(sensorList) / sizeof(bsec_virtual_sensor_t), BSEC_SAMPLE_RATE_LP)) {
        Serial.println("BSEC subscription update error!");
        checkBsecStatus(envSensor);
    }

    envSensor.attachCallback(newDataCallback);

    timeClient.begin();
    configTime(3600, 0, "pool.ntp.org", "time.nist.gov");
}

void loop() {
    M5Dial.update();
    displayTime();

    static unsigned long lastWiFiCheck = 0;
    static bool wasConnected = false;
    if (isConfigured && (millis() - lastWiFiCheck >= 5000)) {
        lastWiFiCheck = millis();

        if (WiFi.status() != WL_CONNECTED) {
            if (wasConnected) {
                Serial.println("WiFi connection lost. Reconnecting...");
                WiFi.begin(ssid, password);
            }
            wasConnected = false;
        } else {
            if (!wasConnected) {
                Serial.println("WiFi connected!");
            }
            wasConnected = true;
        }
    }

    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 3000) {
        if (!envSensor.run()) {
            Serial.println("Error running BSEC!");
            checkBsecStatus(envSensor);
        }
        lastSensorRead = millis();
    }

    static unsigned long lastTimeSync = 0;
    if (millis() - lastTimeSync >= 60000) {
        timeClient.update();
        lastTimeSync = millis();
    }

    delay(10);  
}