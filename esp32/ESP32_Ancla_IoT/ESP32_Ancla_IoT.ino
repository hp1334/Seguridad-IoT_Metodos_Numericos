// Grupo 1 — ESP32: captura de tramas Wi-Fi y RSSI por puerto serie.
// Protocolo del prototipo: SNIFF,MAC,RSSI (115200 baudios).
// Uso en laboratorio o red cuya observación esté autorizada.
#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "configuracion_local.h"

struct wifi_ieee80211_mac_hdr_t {
  uint8_t frame_ctrl[2];
  uint8_t duration[2];
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint8_t seq_ctrl[2];
};

void sniffer_callback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (buf == nullptr || (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA)) return;
  const auto* paquete = static_cast<const wifi_promiscuous_pkt_t*>(buf);
  if (paquete->rx_ctrl.sig_len < sizeof(wifi_ieee80211_mac_hdr_t)) return;
  const auto* cabecera = reinterpret_cast<const wifi_ieee80211_mac_hdr_t*>(paquete->payload);
  const int rssi = paquete->rx_ctrl.rssi;
  if (rssi > -85 && rssi < -10 && Serial.availableForWrite() >= 40) {
    Serial.printf("SNIFF,%02X:%02X:%02X:%02X:%02X:%02X,%d\n",
                  cabecera->addr2[0], cabecera->addr2[1], cabecera->addr2[2],
                  cabecera->addr2[3], cabecera->addr2[4], cabecera->addr2[5], rssi);
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  const bool ap_listo = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, 4);
  if (!ap_listo) {
    Serial.println("ERROR: no se pudo iniciar el punto de acceso ESP32.");
    return;
  }
  wifi_promiscuous_filter_t filtro{};
  filtro.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  esp_wifi_set_promiscuous_filter(&filtro);
  esp_wifi_set_promiscuous_rx_cb(&sniffer_callback);
  esp_wifi_set_promiscuous(true);
  Serial.printf("ESP32_LISTO,CANAL,%d\n", AP_CHANNEL);
}

void loop() {
  delay(100);
}
