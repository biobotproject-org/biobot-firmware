// config.example.h - per-node identity.
//
// Copy this file to config.h in the same folder and fill in your values.
// config.h is git-ignored so per-node settings never land in the repo.
// The node carries no API credentials: the Notehub route that forwards
// notes to the BioBot API adds the server's token itself.
#pragma once

// Blues Notehub product UID for this fleet (Notehub -> Project -> Settings).
#define NOTEHUB_PRODUCT_UID "com.example.you:biobot"

// "continuous" keeps the modem connected and delivers batches immediately.
// "periodic" is far kinder to a solar battery; the Notecard then syncs on its
// own schedule, but alert notes are still pushed immediately because they are
// added with sync:true.
#define NOTEHUB_MODE "continuous"

// This node.
#define DEVICE_ID "biobot-001"
#define DEVICE_NAME "BioBot Node 1"
#define DEVICE_TYPE "air-quality-monitor"
