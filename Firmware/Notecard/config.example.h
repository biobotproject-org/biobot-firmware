// config.example.h - per-node identity and secrets.
//
// Copy this file to config.h in the same folder and fill in your values.
// config.h is listed in .gitignore so credentials never land in the repo.
#pragma once

// Blues Notehub product UID for this fleet (Notehub -> Project -> Settings).
#define NOTEHUB_PRODUCT_UID "com.example.you:biobot"

// "continuous" keeps the modem connected and delivers batches immediately.
// "periodic" is far kinder to a solar battery; the Notecard then syncs on its
// own schedule, but alert notes are still pushed immediately because they are
// added with sync:true.
#define NOTEHUB_MODE "continuous"

// BioBot API. The Notehub route forwards each note to these endpoints.
#define API_AUTH_TOKEN "replace-me"
#define API_DEVICES_URL "https://example.com/devices"
#define API_ALERTS_URL "https://example.com/alerts"

// This node.
#define DEVICE_ID "biobot-001"
#define DEVICE_NAME "BioBot Node 1"
#define DEVICE_TYPE "air-quality-monitor"
