#pragma once

// Copy this file to gemini_config.h in this same folder, then set a real key
// and model URL. gemini_connection.h includes gemini_config.h, and that real
// file is gitignored. Do not commit API keys.
//
// Names consumed by gemini_connection.h:
//   GEMINI_API_KEY   sent as the x-goog-api-key header
//   GEMINI_URL       full HTTPS URL, including the generateContent path
//
// The sketch refuses to call Gemini while the key is still this placeholder.

inline constexpr const char *GEMINI_API_KEY = "YOUR_GEMINI_API_KEY";

// Shape (replace YOUR_MODEL with a model the key can call):
//   https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent
// The helper splits on the first slash after "https://" into host and path.
inline constexpr const char *GEMINI_URL =
    "https://generativelanguage.googleapis.com/v1beta/models/YOUR_MODEL:generateContent";
