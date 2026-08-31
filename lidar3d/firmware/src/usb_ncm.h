// USB-C-Transport: der ESP32-S3 meldet sich am iPhone als USB-Ethernet
// (CDC-NCM). Das iPhone sieht ein ganz normales Netzwerkinterface, bekommt
// per DHCP eine Adresse, und der TCP-Server aus main.cpp ist darueber
// erreichbar - ohne eine einzige Zeile Protokollaenderung.
//
// Warum nicht seriell: iOS reicht generische USB-Geraete nicht an Apps durch,
// und DriverKit gibt es auf dem iPhone nicht. Begruendung und Fallstricke
// stehen in docs/04-ios-usb.md.
//
// Reihenfolge ist hier entscheidend: iOS fragt DHCP genau einmal beim
// Link-Up und wiederholt es nie. Der Link darf deshalb erst hoch, wenn
// DHCP-Server und TCP-Server stehen - sonst wartet das iPhone fuer immer auf
// eine Adresse.
#pragma once

#include <stdbool.h>

namespace nwl {

// Initialisiert TinyUSB im NCM-Modus und legt das Netzwerkinterface an.
// Der Link bleibt danach absichtlich UNTEN.
bool usbNcmStart();

// Link freigeben. Erst aufrufen, wenn der TCP-Server lauscht.
void usbNcmSetLinkUp(bool up);

// Hat sich ein Host (das iPhone) angemeldet?
bool usbNcmHostPresent();

}  // namespace nwl
