#include "ESPNow_SerialBridge.h"

// 【必改】填斜面板 Slope 上电串口打印的「本机 MAC」（须与 STA 打印完全一致）
// 当前实测：98:A3:16:CC:F0:14（旧值 9A:... 差 0x02，ESP-NOW 无法送达）
uint8_t peerMac[] = {0x98, 0xA3, 0x16, 0xCC, 0xF0, 0x14};

ESPNow_SerialBridge *ESPNow_SerialBridge::instance = nullptr;

ESPNow_SerialBridge::ESPNow_SerialBridge(const uint8_t *peerMacAddr, uint8_t channel, ESPNowWifiMode mode)
    : _wifiChannel(channel), _wifiMode(mode), _qHead(0), _qTail(0), _qCount(0) {
    memcpy(_peerMac, peerMacAddr, 6);
    instance = this;
    _wifiIf = (_wifiMode == ESPNOW_MODE_STA) ? WIFI_IF_STA : WIFI_IF_AP;
    memset(_queue, 0, sizeof(_queue));
    memset(_queueLen, 0, sizeof(_queueLen));
}

bool ESPNow_SerialBridge::initWiFi() {
    if (_wifiMode == ESPNOW_MODE_STA) {
        WiFi.mode(WIFI_STA);
    } else {
        WiFi.mode(WIFI_AP);
    }

    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_err_t ret = esp_wifi_set_channel(_wifiChannel, second);
    if (ret != ESP_OK) {
        Serial.printf("设置WiFi信道失败:%d\n", ret);
        return false;
    }

    delay(100);
    return true;
}

static bool isMacZero(const uint8_t *mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) {
            return false;
        }
    }
    return true;
}

bool ESPNow_SerialBridge::initESPNow() {
    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    // peer 未配置时仍可初始化，方便先读本机 MAC；配置后再重新烧录
    if (isMacZero(_peerMac)) {
        Serial.println("警告: peerMac 未配置，ESP-NOW 发送不可用");
        return true;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, _peerMac, 6);
    peerInfo.channel = _wifiChannel;
    peerInfo.ifidx = _wifiIf;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        return false;
    }

    return true;
}

bool ESPNow_SerialBridge::begin() {
    if (!initWiFi()) {
        return false;
    }
    return initESPNow();
}

bool ESPNow_SerialBridge::send(const char *data) {
    return send((const uint8_t *)data, strlen(data));
}

bool ESPNow_SerialBridge::send(const uint8_t *data, size_t len) {
    if (len == 0 || len > MAX_PACKET || isMacZero(_peerMac)) {
        return false;
    }
    return esp_now_send(_peerMac, data, len) == ESP_OK;
}

void ESPNow_SerialBridge::enqueuePacket(const uint8_t *data, int len) {
    if (len <= 0 || len > MAX_PACKET) {
        return;
    }

    portENTER_CRITICAL(&_mux);
    uint8_t count = _qCount;
    if (count >= QUEUE_DEPTH) {
        _qHead = (_qHead + 1) % QUEUE_DEPTH;
        count = (uint8_t)(count - 1);
    }

    memcpy(_queue[_qTail], data, len);
    _queue[_qTail][len] = '\0';
    _queueLen[_qTail] = (uint8_t)len;
    _qTail = (_qTail + 1) % QUEUE_DEPTH;
    _qCount = (uint8_t)(count + 1);
    portEXIT_CRITICAL(&_mux);
}

bool ESPNow_SerialBridge::dequeuePacket(String &out) {
    char local[MAX_PACKET + 1];
    uint8_t localLen = 0;

    portENTER_CRITICAL(&_mux);
    uint8_t count = _qCount;
    if (count == 0) {
        portEXIT_CRITICAL(&_mux);
        out = "";
        return false;
    }

    localLen = _queueLen[_qHead];
    memcpy(local, _queue[_qHead], localLen);
    local[localLen] = '\0';
    _qHead = (_qHead + 1) % QUEUE_DEPTH;
    _qCount = (uint8_t)(count - 1);
    portEXIT_CRITICAL(&_mux);

    out = String(local);
    _recvLen = localLen;
    memcpy(_recvBuffer, local, localLen);
    return true;
}

#if ESPNOW_RECV_CB_V2
void ESPNow_SerialBridge::onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!instance || !info || !data || len <= 0) {
        return;
    }
    instance->enqueuePacket(data, len);
}
#else
void ESPNow_SerialBridge::onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
    if (!instance || !data || len <= 0) {
        return;
    }
    instance->enqueuePacket(data, len);
}
#endif

void ESPNow_SerialBridge::onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    (void)status;
}

String ESPNow_SerialBridge::getReceivedData() {
    String out;
    if (dequeuePacket(out)) {
        _receivedData = out;
        return out;
    }
    _receivedData = "";
    _recvLen = 0;
    return "";
}

uint8_t *ESPNow_SerialBridge::getRawBuffer() {
    return _recvBuffer;
}

int ESPNow_SerialBridge::getBufferLength() {
    return _recvLen;
}

void ESPNow_SerialBridge::clearReceivedData() {
    portENTER_CRITICAL(&_mux);
    _receivedData = "";
    _recvLen = 0;
    memset(_recvBuffer, 0, sizeof(_recvBuffer));
    _qHead = 0;
    _qTail = 0;
    _qCount = 0;
    memset(_queue, 0, sizeof(_queue));
    memset(_queueLen, 0, sizeof(_queueLen));
    portEXIT_CRITICAL(&_mux);
}

int ESPNow_SerialBridge::pendingCount() const {
    return _qCount;
}

String ESPNow_SerialBridge::getLocalMac() {
    if (_wifiMode == ESPNOW_MODE_STA) {
        return WiFi.macAddress();
    }
    return WiFi.softAPmacAddress();
}
