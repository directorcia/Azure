// Overhead board camera: /capture = JPEG photo, /debug = sample boxes,
// /cells = cell colours, /board = nine-character text for the UNO R4.
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "wifi_secrets.h" // copy include/wifi_secrets_example.h and fill in the network

WebServer server(80);

// Board area inside the 320x240 frame; adjust after viewing /capture
int boardX = 55, boardY = 60, boardW = 155, boardH = 130;

// fmt2rgb888 writes B, G, R on this board (verified on hardware).
// Read as R,G,B, a red X was (114, 119, 165) and a yellow O was (192, 231, 240).
// Byte 2 is red, so those are RGB (165, 119, 114) and (240, 231, 192).
// Empty cells sit around 190-245 with the channels roughly equal.
constexpr int kEmptyMaxRedMinusBlue = 30;    // r - b below this is empty
constexpr double kYellowMinGreenToRed = 0.8; // g > r * this is a yellow O

// '.' empty, 'O' yellow, 'X' red.
char classifyCell(int r, int g, int b) {
  if (r - b < kEmptyMaxRedMinusBlue) return '.';
  if (g > r * kYellowMinGreenToRed) return 'O';
  return 'X';
}

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  // AI-Thinker ESP32-CAM pinout
  c.pin_d0 = 5;  c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_xclk = 0; c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sccb_sda = 26; c.pin_sccb_scl = 27; c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QVGA;      // 320x240 is plenty for 9 cells
  c.jpeg_quality = 12; c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  return esp_camera_init(&c) == ESP_OK;
}

void handleCapture() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleDebug() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { server.send(500, "text/plain", "capture failed"); return; }
  int W = fb->width, H = fb->height;
  size_t rgbSize = (size_t)W * H * 3;
  uint8_t *rgb = (uint8_t *)ps_malloc(rgbSize);
  bool ok = rgb && fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
  esp_camera_fb_return(fb);
  if (!ok) { free(rgb); server.send(500, "text/plain", "decode failed"); return; }

  int cw = boardW / 3, ch = boardH / 3;
  for (int cell = 0; cell < 9; cell++) {
    int x0 = boardX + (cell % 3) * cw + cw / 4;
    int y0 = boardY + (cell / 3) * ch + ch / 4;
    int x1 = x0 + cw / 2, y1 = y0 + ch / 2;
    auto drawRedPixel = [rgb, W, H](int x, int y) {
      if (x < 0 || x >= W || y < 0 || y >= H) return;
      uint8_t *p = rgb + ((size_t)y * W + x) * 3;
      // Same B, G, R order as fmt2rgb888 (verified on hardware).
      // fmt2jpg(PIXFORMAT_RGB888) swaps these bytes back to R, G, B,
      // so red in this buffer is (0, 0, 255) and the boxes draw red.
      p[0] = 0; p[1] = 0; p[2] = 255;
    };
    for (int x = x0; x <= x1; x++) {
      drawRedPixel(x, y0);
      drawRedPixel(x, y1);
    }
    for (int y = y0; y <= y1; y++) {
      drawRedPixel(x0, y);
      drawRedPixel(x1, y);
    }
  }

  uint8_t *jpg = nullptr;
  size_t len = 0;
  ok = fmt2jpg(rgb, rgbSize, W, H, PIXFORMAT_RGB888, 80, &jpg, &len);
  free(rgb);
  if (!ok || !jpg) {
    free(jpg);
    server.send(500, "text/plain", "encode failed");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char *)jpg, len);
  free(jpg);
}

// One cell after averaging. `cell` is the board number 1..9.
struct CellReading {
  int cell;
  int r, g, b;
  char piece;
};

// Average R, G, B over the middle half of each cell (skips grid lines).
// Image index 0 is the top-left of the frame. The camera sees the board
// rotated 180 degrees, so the reported cell is 9 - that index.
// Fills `cells` in image order. Returns a short error string, or nullptr.
const char *sampleCells(CellReading cells[9]) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return "capture failed";
  int W = fb->width;
  size_t n = (size_t)fb->width * fb->height * 3;
  uint8_t *rgb = (uint8_t *)ps_malloc(n);
  bool ok = rgb && fmt2rgb888(fb->buf, fb->len, fb->format, rgb);
  esp_camera_fb_return(fb);
  if (!ok) { free(rgb); return "decode failed"; }

  int cw = boardW / 3, ch = boardH / 3;
  for (int i = 0; i < 9; i++) {
    int x0 = boardX + (i % 3) * cw + cw / 4, y0 = boardY + (i / 3) * ch + ch / 4;
    uint32_t r = 0, g = 0, b = 0, cnt = 0;
    for (int y = y0; y < y0 + ch / 2; y++)
      for (int x = x0; x < x0 + cw / 2; x++) {
        uint8_t *p = rgb + ((size_t)y * W + x) * 3;
        // fmt2rgb888 is B, G, R here (verified on hardware).
        b += p[0]; g += p[1]; r += p[2]; cnt++;
      }
    cells[i].cell = 9 - i;
    cells[i].r = r / cnt;
    cells[i].g = g / cnt;
    cells[i].b = b / cnt;
    cells[i].piece = classifyCell(cells[i].r, cells[i].g, cells[i].b);
  }
  free(rgb);
  return nullptr;
}

void handleCells() {
  CellReading cells[9];
  const char *err = sampleCells(cells);
  if (err) { server.send(500, "text/plain", err); return; }

  // Image order, so the objects run cell 9, 8, ... 1. "s" is '.', 'X', or 'O'.
  String json = "[";
  for (int i = 0; i < 9; i++) {
    json += "{\"cell\":" + String(cells[i].cell) + ",\"r\":" + String(cells[i].r) +
            ",\"g\":" + String(cells[i].g) + ",\"b\":" + String(cells[i].b) +
            ",\"s\":\"";
    json += cells[i].piece;
    json += "\"}";
    if (i < 8) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// Exactly nine characters, cell 1 first, for the UNO R4. Example: X...O....
void handleBoard() {
  CellReading cells[9];
  const char *err = sampleCells(cells);
  if (err) { server.send(500, "text/plain", err); return; }

  char board[10];
  board[9] = '\0';
  for (int i = 0; i < 9; i++) board[cells[i].cell - 1] = cells[i].piece;
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/plain", board, 9);
}

void setup() {
  Serial.begin(115200);
  if (!initCamera()) { Serial.println("Camera init FAILED - check pins/PSRAM"); return; }
  WiFi.begin(wifi_secrets::ssid, wifi_secrets::password);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  String ip = WiFi.localIP().toString();
  Serial.printf("\nCamera ready: http://%s/capture  http://%s/debug  http://%s/cells  http://%s/board\n",
                ip.c_str(), ip.c_str(), ip.c_str(), ip.c_str());
  server.on("/capture", handleCapture);
  server.on("/debug", handleDebug);
  server.on("/cells", handleCells);
  server.on("/board", HTTP_GET, handleBoard);
  server.begin();
}

void loop() { server.handleClient(); }
