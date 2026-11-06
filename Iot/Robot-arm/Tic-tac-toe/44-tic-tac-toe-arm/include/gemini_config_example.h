#pragma once

// Copy this file to gemini_config.h (same folder) and replace the placeholders.
// gemini_config.h is gitignored. Do not commit it.
//
// GEMINI_API_KEY must not stay as YOUR_GEMINI_API_KEY; the firmware treats that
// exact text as "not set".
// GEMINI_URL must be an https URL that includes a path. Use the public Gemini
// generateContent URL for the model you call. The host below is only a dummy.

static const char GEMINI_API_KEY[] = "YOUR_GEMINI_API_KEY";
static const char GEMINI_URL[] = "https://example.invalid/gemini/generateContent";
