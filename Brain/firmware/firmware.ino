#include <Arduino.h>
#include <ESP_I2S.h>


#include "srmodels.h"
#include "model_path.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"

#include "esp_mn_speech_commands.h"

#define I2S_BCLK 4
#define I2S_WS 5
#define I2S_DIN 6

#define SAMPLE_RATE 16000
#define BUFFER_SIZE 512

HardwareSerial Link(1);

#define UART_TX 17
#define UART_RX 18


I2SClass Mic;


int32_t rawSample[BUFFER_SIZE];
int16_t samples[BUFFER_SIZE];

srmodel_list_t *models = nullptr;

esp_mn_iface_t *multinet = nullptr;
model_iface_data_t *mnData = nullptr;

char mnName[] = "mn7_en";

constexpr char *commands[] = { "SASHA", "SASHA LIGHTS ON", "SASHA LIGHTS OFF", "SASHA LIGHTS" };
constexpr size_t commandCount = sizeof(commands) / sizeof(commands[0]);

constexpr size_t MULTINET_FRAME_SIZE = 512;
int16_t mnFrame[MULTINET_FRAME_SIZE];

size_t frameSamples = MULTINET_FRAME_SIZE;
size_t samplesUsed = 0;

bool initCommands() {
  esp_err_t err = esp_mn_commands_alloc(multinet, mnData);

  if (err != ESP_OK) {
    Serial.println("Command allocation failed");
    return false;
  }

  for (size_t i = 0; i < commandCount; i++) {
    if (esp_mn_commands_add(i , commands[i]) != ESP_OK) {
      Serial.printf("Failed to add command: %s\n", commands[i]);
      return false;
    }
  }

  esp_mn_error_t *updateError = esp_mn_commands_update();
  if (updateError != nullptr) {
    Serial.println("Commands couldn't be parsed");
    return false;
  }

  esp_mn_commands_print();

  return true;
}
bool initModels() {
  models = srmodel_load(srmodels_bin);
  if (models == nullptr) {
    Serial.println("Failed to load speech models");
    return false;
  }

  Serial.println("Speech model loaded");

  multinet = esp_mn_handle_from_name(mnName);

  if (multinet == nullptr) {
    Serial.println("Multinet interface not found");
    return false;
  }

  mnData = multinet->create(mnName, 6000);

  if (mnData == nullptr) {
    Serial.println("Failed to crate MultiNet");
    return false;
  }


  return true;
}

void setup() {

  //Setup Serial Turn off on Prod
  Serial.begin(115200);

  //Setup data transfer
  Link.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  //Mic setup
  Mic.setPins(I2S_BCLK, I2S_WS, -1, I2S_DIN);

  if (!Mic.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Init Failed");
    while (true) delay(1000);
  }

  //Load models
  if (!initModels()) {
    Serial.println("Models failed");
    while (true) delay(1000);
  }

  //Register commands
  if (!initCommands()) {
    Serial.println("Comamnds Init Failed");
    while (true) delay(1000);
  }
}

constexpr size_t bytesNeeded = sizeof(rawSample);
bool readMicrophone() {
  size_t bytesRead = Mic.readBytes((char *)rawSample, bytesNeeded);
  if (bytesRead != bytesNeeded) {
    return false;
  }
  for (int i = 0; i < BUFFER_SIZE; i++) {
    samples[i] = (int16_t)(rawSample[i] >> 16);
  }
  return true;
}
void loop() {
  if (!readMicrophone()) {
    Serial.println("Microphone reading failed");
    return;
  }

  size_t offset = 0;

  while(offset<BUFFER_SIZE){
    size_t remaining = frameSamples - samplesUsed;
    size_t available = BUFFER_SIZE - offset;

    size_t count = (remaining < available) ? remaining : available;

    memcpy(mnFrame + samplesUsed, samples + offset, count*sizeof(int16_t));

    samplesUsed += count;
    offset+=count;

    if(samplesUsed == frameSamples){

      esp_mn_state_t state = multinet->detect(mnData, mnFrame);

      if(state == ESP_MN_STATE_DETECTED){
        esp_mn_results_t *result = multinet->get_results(mnData);
        uint8_t command = result->command_id[0];
        Serial.printf("Detected:%s\n", commands[command]);
        Link.write(&command, 1);
      }
      samplesUsed = 0;
    }
  }
}