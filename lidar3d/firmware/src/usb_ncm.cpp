// NCM-Transport, siehe usb_ncm.h.
//
// ACHTUNG, ungetestet und versionsabhaengig: die API von `esp_tinyusb` hat
// sich zwischen den IDF-Versionen mehrfach geaendert (tinyusb_net_init,
// tinyusb_net_send_sync, Signatur der Callbacks). Vor dem ersten Flashen
// gegen das mitgelieferte Beispiel
// `esp-idf/examples/peripherals/usb/device/tusb_ncm` der eigenen IDF-Version
// abgleichen. Die Logik drumherum - Adressen, DHCP-Server, und vor allem die
// Reihenfolge beim Link-Up - ist davon unberuehrt.

#include "usb_ncm.h"

#include <esp_check.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <string.h>
#include <tinyusb.h>
#include <tinyusb_net.h>

#include "../include/config.h"

namespace nwl {
namespace {

const char *kTag = "usb_ncm";

esp_netif_t *g_netif = nullptr;
esp_netif_driver_ifconfig_t g_ifconfig = {};
bool g_linkUp = false;
bool g_hostPresent = false;

// Vom Netzwerkstack zum iPhone.
esp_err_t netifTransmit(void *, void *buffer, size_t len) {
  if (!g_linkUp) return ESP_OK;  // vor dem Link-Up nichts senden
  if (tinyusb_net_send_sync(buffer, len, nullptr, pdMS_TO_TICKS(100)) != ESP_OK) {
    return ESP_FAIL;
  }
  return ESP_OK;
}

// Vom iPhone in den Netzwerkstack.
esp_err_t usbReceive(void *buffer, uint16_t len, void *) {
  g_hostPresent = true;
  if (g_netif == nullptr) return ESP_OK;
  return esp_netif_receive(g_netif, buffer, len, nullptr);
}

void usbFreeBuffer(void *, void *) {
  // esp_netif_receive kopiert; hier ist nichts freizugeben.
}

// Statische Adresse plus DHCP-Server: das iPhone bekommt 192.168.7.2.
bool configureAddresses(esp_netif_t *netif) {
  esp_netif_ip_info_t ip = {};
  ip.ip.addr = esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 7, 1));
  ip.gw.addr = esp_netif_htonl(esp_netif_ip4_makeu32(192, 168, 7, 1));
  ip.netmask.addr = esp_netif_htonl(esp_netif_ip4_makeu32(255, 255, 255, 0));

  // Der DHCP-Server muss zum Umkonfigurieren stehen.
  esp_netif_dhcps_stop(netif);
  if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) return false;
  if (esp_netif_dhcps_start(netif) != ESP_OK) return false;
  return true;
}

}  // namespace

bool usbNcmStart() {
  // 1. Netzwerkinterface anlegen - noch ohne Link.
  esp_netif_inherent_config_t base = {};
  base.flags = static_cast<esp_netif_flags_t>(ESP_NETIF_DHCP_SERVER |
                                              ESP_NETIF_FLAG_AUTOUP);
  base.if_desc = "usb-ncm";
  base.route_prio = 20;  // niedriger als WLAN, damit beides koexistieren kann

  esp_netif_config_t cfg = {};
  cfg.base = &base;
  cfg.driver = nullptr;
  cfg.stack = ESP_NETIF_NETSTACK_DEFAULT_ETH;

  g_netif = esp_netif_new(&cfg);
  if (g_netif == nullptr) {
    ESP_LOGE(kTag, "esp_netif_new fehlgeschlagen");
    return false;
  }

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
  esp_netif_set_mac(g_netif, mac);

  g_ifconfig.handle = reinterpret_cast<void *>(1);  // Dummy, kein echter Treiber
  g_ifconfig.transmit = netifTransmit;
  g_ifconfig.driver_free_rx_buffer = nullptr;
  if (esp_netif_set_driver_config(g_netif, &g_ifconfig) != ESP_OK) {
    ESP_LOGE(kTag, "Treiberkonfiguration fehlgeschlagen");
    return false;
  }

  if (!configureAddresses(g_netif)) {
    ESP_LOGE(kTag, "Adressvergabe fehlgeschlagen");
    return false;
  }

  // 2. TinyUSB im NCM-Modus starten.
  tinyusb_config_t usbCfg = {};
  usbCfg.external_phy = false;
  if (tinyusb_driver_install(&usbCfg) != ESP_OK) {
    ESP_LOGE(kTag, "tinyusb_driver_install fehlgeschlagen");
    return false;
  }

  tinyusb_net_config_t netCfg = {};
  memcpy(netCfg.mac_addr, mac, sizeof(mac));
  netCfg.on_recv_callback = usbReceive;
  netCfg.free_tx_buffer = usbFreeBuffer;
  if (tinyusb_net_init(TINYUSB_USBDEV_0, &netCfg) != ESP_OK) {
    ESP_LOGE(kTag, "tinyusb_net_init fehlgeschlagen");
    return false;
  }

  // 3. Link bleibt unten, bis usbNcmSetLinkUp(true) kommt. Das ist die
  //    eigentliche Pointe: iOS fragt DHCP nur einmal beim Link-Up.
  g_linkUp = false;
  esp_netif_action_stop(g_netif, nullptr, 0, nullptr);
  ESP_LOGI(kTag, "NCM bereit, Link noch unten");
  return true;
}

void usbNcmSetLinkUp(bool up) {
  if (g_netif == nullptr || up == g_linkUp) return;
  g_linkUp = up;
  if (up) {
    esp_netif_action_start(g_netif, nullptr, 0, nullptr);
    esp_netif_action_connected(g_netif, nullptr, 0, nullptr);
    ESP_LOGI(kTag, "Link oben, DHCP-Server laeuft auf 192.168.7.1");
  } else {
    esp_netif_action_disconnected(g_netif, nullptr, 0, nullptr);
    esp_netif_action_stop(g_netif, nullptr, 0, nullptr);
    ESP_LOGI(kTag, "Link unten");
  }
}

bool usbNcmHostPresent() { return g_hostPresent; }

}  // namespace nwl
