#include <Arduino.h>

#include <BeeGuardAI_Hornet-clone_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// ===============================
// UART 3 octets vers uPesy
// ===============================
static const uint32_t LINK_BAUD = 115200;

// XIAO pins I2C réutilisées en UART
// SDA = D4, SCL = D5
static const int UART_RX_GPIO = D4; // optionnel (uPesy TX -> XIAO RX)
static const int UART_TX_GPIO = D5; // XIAO TX -> uPesy RX (GPIO21)

static inline uint8_t clamp_u8(int v, int lo, int hi) {
  if (v < lo) return (uint8_t)lo;
  if (v > hi) return (uint8_t)hi;
  return (uint8_t)v;
}

static void send_packet_3(uint8_t type, uint8_t count, uint8_t conf) {
  uint8_t pkt[3] = { type, count, conf };
  Serial1.write(pkt, 3);
}

// ===============================
// XIAO ESP32S3 Sense camera pins
// ===============================
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1

#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40   // CAM_SDA
#define SIOC_GPIO_NUM    39   // CAM_SCL

#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15

#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE         3

static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t* snapshot_buf = nullptr;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static bool ei_camera_init(void);
static bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf);
static int  ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
  Serial.begin(115200);
  delay(300);

  // UART vers uPesy sur D4/D5
  Serial1.begin(LINK_BAUD, SERIAL_8N1, UART_RX_GPIO, UART_TX_GPIO);
  Serial.printf("[XIAO] Serial1 UART on D4/D5 -> TX=D5(%d) RX=D4(%d) @%lu\n",
                (int)UART_TX_GPIO, (int)UART_RX_GPIO, (unsigned long)LINK_BAUD);

  Serial.println("[XIAO] Init camera...");
  if (!ei_camera_init()) {
    ei_printf("Failed to initialize camera!\r\n");
    while (1) delay(1000);
  }
  ei_printf("Camera initialized\r\n");

  snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                                  EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                                  EI_CAMERA_FRAME_BYTE_SIZE);//3 octets par pixel
  if (!snapshot_buf) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    while (1) delay(1000);
  }

  ei_printf("\nStarting continuous inference...\n");
  ei_sleep(1000);
}

void loop() {
  if (ei_sleep(10) != EI_IMPULSE_OK) return;

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (!ei_camera_capture((uint32_t)EI_CLASSIFIER_INPUT_WIDTH,
                         (uint32_t)EI_CLASSIFIER_INPUT_HEIGHT,
                         snapshot_buf)) {
    ei_printf("Failed to capture image\r\n");
    return;
  }

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: run_classifier (%d)\n", err);
    return;
  }

  uint8_t type = 0, count = 0, conf = 0;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  const float TH = 0.50f;
  int hornet_count = 0;
  float best = 0.0f;
  for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
    auto bb = result.bounding_boxes[i];
    if (bb.value == 0) continue;
    if (strcasecmp(bb.label, "Hornet") == 0) {
      if (bb.value >= TH) hornet_count++; // nombre
      if (bb.value > best) best = bb.value;//confiance
    }
  }

  type  = (hornet_count > 0) ? 1 : 0;
  count = clamp_u8(hornet_count, 0, 255);
  conf  = clamp_u8((int)(best * 100.0f + 0.5f), 0, 100);

#else
  float hornet_p = 0.0f;

  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char* lab = ei_classifier_inferencing_categories[i];
    float p = result.classification[i].value;
    if (strcasecmp(lab, "Hornet") == 0) hornet_p = p;
  }

  const float TH = 0.70f;
  type  = (hornet_p >= TH) ? 1 : 0;
  count = type ? 1 : 0;
  conf  = clamp_u8((int)(hornet_p * 100.0f + 0.5f), 0, 100);
#endif

  send_packet_3(type, count, conf);

  Serial.printf("[XIAO] Sent 3B => type=%u count=%u conf=%u\n", type, count, conf);
}

static bool ei_camera_init(void) {
  if (is_initialised) return true;

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;

  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);

  is_initialised = true;
  return true;
}

static bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf) {
  if (!is_initialised) return false;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;

  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, out_buf);
  esp_camera_fb_return(fb);
  if (!ok) return false;

  if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
      img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height
    );
  }
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++) {
    uint8_t r = snapshot_buf[pixel_ix + 0];
    uint8_t g = snapshot_buf[pixel_ix + 1];
    uint8_t b = snapshot_buf[pixel_ix + 2];
    out_ptr[i] = (float)((r << 16) | (g << 8) | b);
    pixel_ix += 3;
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA)
#error "Invalid model for current sensor"
#endif