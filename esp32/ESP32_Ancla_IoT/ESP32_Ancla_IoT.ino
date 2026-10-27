// ESP32 del Grupo 1: lectura Wi-Fi en modo promiscuo y envío RSSI por USB.
// Protocolo serie: SNIFF,MAC,RSSI | 115200 baudios.
// Captura destinada a laboratorios y redes con autorización.
#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "configuracion_local.h"

struct Cabecera80211 {
  uint8_t control[2], duracion[2];
  uint8_t addr1[6], addr2[6], addr3[6], secuencia[2];
};
struct Muestra {
  uint8_t mac[6];
  int8_t rssi;
};
static QueueHandle_t cola = nullptr;

void al_recibir(void* buffer, wifi_promiscuous_pkt_type_t tipo) {
  if (cola == nullptr || buffer == nullptr ||
      (tipo != WIFI_PKT_MGMT && tipo != WIFI_PKT_DATA)) return;
  const auto* pkt = static_cast<const wifi_promiscuous_pkt_t*>(buffer);
  if (pkt->rx_ctrl.sig_len < sizeof(Cabecera80211)) return;
  const auto* encabezado = reinterpret_cast<const Cabecera80211*>(pkt->payload);
  const int rssi = pkt->rx_ctrl.rssi;
  if (rssi <= -85 || rssi >= -10) return;
  Muestra m{};
  memcpy(m.mac, encabezado->addr2, 6);
  m.rssi = static_cast<int8_t>(rssi);
  xQueueSend(cola, &m, 0); // Sin esperas ni impresión serial dentro del callback Wi-Fi.
}

void setup() {
  Serial.begin(115200);
  delay(400);
  cola = xQueueCreate(128, sizeof(Muestra));
  if (cola == nullptr) {
    Serial.println("ERROR_COLA_ESP32");
    return;
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, 4)) {
    Serial.println("ERROR_AP_ESP32");
    return;
  }
  wifi_promiscuous_filter_t filtro{};
  filtro.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filtro));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&al_recibir));
  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
  Serial.printf("ESP32_LISTO,CANAL,%d\n", AP_CHANNEL);
}

void loop() {
  if (cola == nullptr) { delay(1000); return; }
  Muestra m{};
  if (xQueueReceive(cola, &m, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.printf("SNIFF,%02X:%02X:%02X:%02X:%02X:%02X,%d\n",
                  m.mac[0], m.mac[1], m.mac[2], m.mac[3], m.mac[4], m.mac[5], m.rssi);
  }
}
