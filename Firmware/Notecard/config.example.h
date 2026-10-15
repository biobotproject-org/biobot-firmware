// config.example.h - per-node identity.
//
// Copy this file to config.h in the same folder and fill in your values.
// config.h is git-ignored so per-node settings never land in the repo.
// The node carries no API credentials: the Notehub route that forwards
// notes to the BioBot API adds the server's token itself.
#pragma once

// Blues Notehub product UID for this fleet (Notehub -> Project -> Settings).
#define NOTEHUB_PRODUCT_UID "com.example.you:biobot"

// "periodic" lets the modem sleep between syncs. Every batch and every alert
// is added with sync:true, so delivery timing is unchanged - the only thing
// that changes is that the modem is not holding a tower connection for the
// 599 seconds out of every 600 when it has nothing to say. On the bench this
// is the difference between ~44 mA and ~4 mA average for the Notecard.
// "continuous" keeps the connection up permanently; use it only on mains.
#define NOTEHUB_MODE "periodic"

// This node.
#define DEVICE_ID "biobot-001"
#define DEVICE_NAME "BioBot Node 1"
#define DEVICE_TYPE "air-quality-monitor"
