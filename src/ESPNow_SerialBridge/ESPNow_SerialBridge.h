#ifndef ESPNow_SerialBridge_h
#define ESPNow_SerialBridge_h

#include "Arduino.h"
#include "WiFi.h"
#include "esp_now.h"
#include "esp_wifi.h"

// Arduino-ESP32 3.x 才用新回调(esp_now_recv_info_t)；2.x 用 (mac, data, len)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define ESPNOW_RECV_CB_V2 1
#else
#define ESPNOW_RECV_CB_V2 0
#endif

// ===================== 你需要修改的参数 =====================
// 目标设备 MAC：斜面 → 填【垂直板 Total】上电打印的本机 MAC
extern uint8_t peerMac[6];
#define WIFI_CHANNEL 1
#define WIFI_MODE ESPNOW_MODE_STA
// ==========================================================

enum ESPNowWifiMode {
    ESPNOW_MODE_AP = 0,
    ESPNOW_MODE_STA = 1
};

class ESPNow_SerialBridge {
private:
    static constexpr int QUEUE_DEPTH = 8;
    static constexpr int MAX_PACKET = 250;

    uint8_t _peerMac[6];
    uint8_t _wifiChannel;
    ESPNowWifiMode _wifiMode;
    wifi_interface_t _wifiIf;

    char _queue[QUEUE_DEPTH][MAX_PACKET + 1];
    uint8_t _queueLen[QUEUE_DEPTH];
    volatile uint8_t _qHead;
    volatile uint8_t _qTail;
    volatile uint8_t _qCount;

    String _receivedData = "";
    uint8_t _recvBuffer[MAX_PACKET];
    int _recvLen = 0;

    bool initWiFi();
    bool initESPNow();

#if ESPNOW_RECV_CB_V2
    static void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
#else
    static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len);
#endif
    static void onDataSent(const uint8_t *mac, esp_now_send_status_t status);
    static ESPNow_SerialBridge *instance;

    void enqueuePacket(const uint8_t *data, int len);
    bool dequeuePacket(String &out);

    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

public:
    ESPNow_SerialBridge(const uint8_t *peerMac, uint8_t channel, ESPNowWifiMode mode);

    bool begin();
    bool send(const char *data);
    bool send(const uint8_t *data, size_t len);

    /** 取出队列中最早一包（取出后从队列移除）；无数据返回空串 */
    String getReceivedData();

    uint8_t *getRawBuffer();
    int getBufferLength();
    void clearReceivedData();

    /** 队列中待取包数量 */
    int pendingCount() const;

    String getLocalMac();
};

#endif
