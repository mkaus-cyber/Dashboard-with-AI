#pragma once
// Fill these in. Do NOT share this file or post it online.

// ---------- WiFi (first start only) ----------
// After the first start, WiFi is chosen on the Settings page (gear icon) and saved in the board's
// flash. These values are only used until you pick a network there.
#define WIFI_SSID          "your-wifi-name"
#define WIFI_PASS          "your-wifi-password"

// ---------- API keys ----------
// Claude (https://console.anthropic.com) - used for the voice box replies
#define ANTHROPIC_API_KEY  "sk-ant-..."

// OpenAI - used ONLY for speech-to-text and text-to-speech in the voice box
// (the Claude API takes text in / text out, so something else must hear and speak)
#define OPENAI_API_KEY     "sk-..."

// Time zone and city are now chosen on the Settings page, so the old location lines
// (LOCATION_NAME, LAT, LON, TZ_POSIX, WEATHER_TZ) are no longer needed here.
