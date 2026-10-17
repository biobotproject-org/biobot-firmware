// config.example.h - per-node identity and power settings.
//
// Copy this file to config.h in the same folder and fill in your values.
// config.h is git-ignored so per-node settings never land in the repo.
#pragma once

// Blues Notehub product UID for this fleet (Notehub -> Project -> Settings).
#define NOTEHUB_PRODUCT_UID "com.example.you:biobot"

// "periodic" lets the modem sleep between syncs. Every batch and every alert
// is added with sync:true, so delivery timing is unchanged.
#define NOTEHUB_MODE "periodic"

// This node.
#define DEVICE_ID "biobot-001"
#define DEVICE_NAME "BioBot Node 1"
#define DEVICE_TYPE "air-quality-monitor"

// ---------------------------------------------------------------------------
// Power
// ---------------------------------------------------------------------------
// 0 never sleep (plain delay between ticks - behaves like the main firmware)
// 1 sleep unless the USB serial monitor is open, so the bench stays readable
//   and the field unit sleeps
// 2 always sleep, even with the monitor open (the port drops at every sleep;
//   use this once to prove sleep works, then go back to 1)
#define SLEEP_MODE 1

// ---------------------------------------------------------------------------
// Battery readout over Bluetooth from the Victron SmartSolar
// ---------------------------------------------------------------------------
// 1 to read the controller's instant readout, 0 to leave BLE out entirely.
#define VICTRON_BLE_ENABLED 1

// VictronConnect -> your SmartSolar -> gear icon -> Product info ->
// "Instant readout via Bluetooth" (turn it on) -> "Show encryption data".
// Paste the 32-character key here.
#define VICTRON_BLE_KEY "00000000000000000000000000000000"

// 1 prints every Victron advertisement seen, with raw bytes - leave on for the
// first bench run so a wrong key or an unexpected record is obvious.
#define VICTRON_BLE_VERBOSE 1
