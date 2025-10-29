#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#if __has_include("credentials.h")
#include "credentials.h"
#else
#error "Missing credentials.h. Copy it to include/ and set Wi-Fi credentials."
#endif

static_assert(sizeof(WIFI_SSID) > 1, "Set WIFI_SSID in credentials.h");
static_assert(sizeof(WIFI_PASSWORD) > 1, "Set WIFI_PASSWORD in credentials.h");

namespace
{
constexpr uint32_t STM32_UART_BAUD = 115200;
constexpr uint32_t DEBUG_UART_BAUD = 115200;

constexpr uint8_t MAX_HTTP_RETRIES = 3;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 10000;
constexpr uint32_t SERIAL_FRAME_TIMEOUT_MS = 500;
constexpr size_t STM32_FRAME_MAX_BYTES = 256;
constexpr char NTP_POOL[] = "pool.ntp.org";

#ifndef STM32_RX_PIN
#define STM32_RX_PIN 16 // GPIO16 (RX2)
#endif

#ifndef STM32_TX_PIN
#define STM32_TX_PIN 17 // GPIO17 (TX2)
#endif

HardwareSerial &stm32Serial = Serial1;
HardwareSerial &logSerial = Serial;

WiFiUDP ntpUdp;
NTPClient timeClient(ntpUdp, NTP_POOL, 0, 60000); // update every 60 s

unsigned long lastWifiAttempt = 0;

String toIso8601(const time_t epoch)
{
    char buffer[30];
    tm *timeinfo = gmtime(&epoch);
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeinfo);
    return String(buffer);
}

String wrapPayload(const String &metricsJson)
{
    String payload{"{"};
    payload += "\"device_id\":\"";
    payload += DEVICE_ID;
    payload += "\",\"timestamp\":\"";
    payload += toIso8601(timeClient.getEpochTime());
    payload += "\",\"metrics\":";
    payload += metricsJson;
    payload += "}";
    return payload;
}

void ensureTimeSync()
{
    if (!timeClient.isTimeSet())
    {
        timeClient.begin();
        while (!timeClient.update())
        {
            delay(200);
        }
    }
    else
    {
        timeClient.update();
    }
}

void ensureWifi()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        return;
    }

    const unsigned long now = millis();
    if (now - lastWifiAttempt < WIFI_RECONNECT_INTERVAL_MS)
    {
        return;
    }

    lastWifiAttempt = now;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    for (uint8_t attempt = 0; attempt < 20; ++attempt)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            logSerial.println(F("[WiFi] Connected"));
            ensureTimeSync();
            return;
        }
        delay(500);
    }

    logSerial.println(F("[WiFi] Connection timeout"));
}

bool transmitToServer(const String &payload)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        logSerial.println(F("[HTTP] Wi-Fi not connected"));
        return false;
    }

    WiFiClient client;
    HTTPClient http;
    uint8_t attempt = 0;

    while (attempt < MAX_HTTP_RETRIES)
    {
        ++attempt;
        if (!http.begin(client, TELEMETRY_URL))
        {
            logSerial.println(F("[HTTP] Unable to start request"));
            return false;
        }
        http.addHeader("Content-Type", "application/json");
        const int httpCode = http.POST(payload);
        if (httpCode > 0 && httpCode < 400)
        {
            logSerial.printf("[HTTP] Delivered (%d)\n", httpCode);
            http.end();
            return true;
        }
        logSerial.printf("[HTTP] Error (%d), retry %u/%u\n", httpCode, attempt, MAX_HTTP_RETRIES);
        http.end();
        delay(500 * attempt);
    }

    return false;
}

void sendAck(const bool success)
{
    if (success)
    {
        stm32Serial.println(F("OK"));
    }
    else
    {
        stm32Serial.println(F("ERR"));
    }
}

String readFrame()
{
    static String buffer;
    static unsigned long lastByteMs = 0;

    while (stm32Serial.available())
    {
        const char c = static_cast<char>(stm32Serial.read());
        lastByteMs = millis();

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            String frame = buffer;
            buffer.clear();
            frame.trim();
            if (frame.length() > 0)
            {
                return frame;
            }
            continue;
        }

        if (buffer.length() < STM32_FRAME_MAX_BYTES)
        {
            buffer += c;
        }
        else
        {
            buffer.clear();
            logSerial.println(F("[Serial] Dropped frame (too long)"));
            sendAck(false);
            return String();
        }
    }

    const unsigned long now = millis();
    if (buffer.length() > 0 && lastByteMs != 0 && (now - lastByteMs) >= SERIAL_FRAME_TIMEOUT_MS)
    {
        String frame = buffer;
        buffer.clear();
        frame.trim();
        logSerial.println(F("[Serial] Flushed partial frame after timeout"));
        sendAck(false);
        return String();
    }

    return String();
}

} // namespace

void setup()
{
    logSerial.begin(DEBUG_UART_BAUD);
    logSerial.println();
    logSerial.println(F("STM32 -> ESP32 Telemetry Bridge"));

    stm32Serial.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    stm32Serial.setTimeout(50);

    ensureWifi();
}

void loop()
{
    ensureWifi();

    const String metricsJson = readFrame();
    if (metricsJson.isEmpty())
    {
        delay(5);
        return;
    }

    ensureTimeSync();
    const String payload = wrapPayload(metricsJson);
    const bool success = transmitToServer(payload);
    sendAck(success);
}
