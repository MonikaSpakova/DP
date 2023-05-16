#include "WiFi.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "soc/soc.h"           
#include "soc/rtc_cntl_reg.h"  
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>

//Enter your network credentials
const char* ssid = "ssid";
const char* password = "password";

#define API_KEY "xxx"

// ENter Authorized Email and Password
#define USER_EMAIL "xxx"
#define USER_PASSWORD "xxx"

// Enter Firebase storage bucket ID
#define STORAGE_BUCKET_ID "xxx"

#define IMAGE_PATH "/pictures/image.jpg"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

boolean new_image = true;

FirebaseData fbdo;
FirebaseAuth authentication;
FirebaseConfig configuration;

bool done = false;

bool check_photo( fs::FS &fs ) {
  File f_pic = fs.open( IMAGE_PATH );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void captureSave_photo( void ) {
  camera_fb_t * fb = NULL; 
  bool ok = 0; 
  do {
    Serial.println("ESP32-CAM capturing photo...");

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed");
      return;
    }
    Serial.printf("Picture file name: %s\n", IMAGE_PATH);
    File file = SPIFFS.open(IMAGE_PATH, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); 
      Serial.print("The picture has been saved in ");
      Serial.print(IMAGE_PATH);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    file.close();
    esp_camera_fb_return(fb);
    ok = check_photo(SPIFFS);
  } while ( !ok );
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

 // initialize OV2640 camera module
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  } 

  configuration.api_key = API_KEY;
  authentication.user.email = USER_EMAIL;
  authentication.user.password = USER_PASSWORD;
  configuration.token_status_callback = tokenStatusCallback; 

  Firebase.begin(&configuration, &authentication);
  Firebase.reconnectWiFi(true);
}

void loop() {
  if (new_image) {
    captureSave_photo();
    new_image = true;
  }
  delay(10000);
  if (Firebase.ready() && !done){
    done = true;
    Serial.print("Uploading Photo... ");

    if (Firebase.Storage.upload(&fbdo, STORAGE_BUCKET_ID, IMAGE_PATH, mem_storage_type_flash, IMAGE_PATH, "image/jpeg" )) {
      Serial.printf("\nDownload URL: %s\n", fbdo.downloadURL().c_str());
    }
    else{
      Serial.println(fbdo.errorReason());
    }
    done = false;
  }
  delay(600000);
}
