#include "include/main.h"

#include <math.h>
#include <stdio.h>
#include <Arduino.h>
#include <ArduinoBLE.h>
#include <SparkFunMPU9250-DMP.h>
#include "gesture-control_inferencing.h"

#include "include/helper.h"
#include "include/ble_manager.h"

MPU9250_DMP imu;
Quaternion initial_quaternion;

BLEService InferenceDataService("d2883342-3f05-4c93-b000-e56af810c35d");

BLECharacteristic ClassificationTimestamp("f3a38005-66ac-4ec8-9bab-30e77ac32ae8",
                                          BLERead | BLENotify, BLE_BUFFER_SIZE);
BLECharacteristic ClassificationValue("f753a6c0-350d-42ab-a7bb-104957c8a7e1",
                                      BLERead | BLENotify, BLE_BUFFER_SIZE);
BLECharacteristic ClassificationLabel("514d03fe-aa3b-46ee-a281-521270edc7ce",
                                      BLERead | BLENotify, BLE_BUFFER_SIZE);
BLECharacteristic ClassificationAnomaly("aaec8b2e-207f-41ec-98f3-0bf25910a18d",
                                        BLERead | BLENotify, BLE_BUFFER_SIZE);

BLECharacteristic DspTiming("8e097b91-55e5-4503-b4dd-98d819b8240f",
                            BLERead | BLENotify, BLE_BUFFER_SIZE);
BLECharacteristic TimingClassification("7951b2bf-b7aa-4426-8e48-2cffeaa57cae",
                                       BLERead | BLENotify, BLE_BUFFER_SIZE);
BLECharacteristic TimingAnomaly("831d524f-4657-47a7-aa6d-2998b87a99aa",
                                BLERead | BLENotify, BLE_BUFFER_SIZE);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  while (imu.begin() != INV_SUCCESS) {
    while (1) {
      Serial.println("Failed to initialize IMU!");
      delay(5000);
    }
  }

  // InitializeBLE();

  /* IMU and Magnetometer Configuration */
  imu.setSampleRate(100);
  imu.setCompassSampleRate(100);

  /* Magnetometer Calibration */
  float mag_x = 0, mag_y = 0, mag_z = 0, samples = 0;
  float start_time = millis();

  while (millis() - start_time < 2000) {
    if (imu.dataReady()) {
      imu.update(UPDATE_COMPASS);
      imu.calcMag(imu.mx);
      imu.calcMag(imu.my);
      imu.calcMag(imu.mz);
    }
  }

  start_time = millis();
  while (millis() - start_time < 2000) {
    if (imu.dataReady()) {
      imu.update(UPDATE_COMPASS);
      mag_x += imu.calcMag(imu.mx);
      mag_y += imu.calcMag(imu.my);
      mag_z += imu.calcMag(imu.mz);
      samples++;
    }
  }

  mag_x = mag_x / samples - b_mag[0];
  mag_y = mag_y / samples - b_mag[1];
  mag_z = mag_z / samples - b_mag[2];
  mag_x = mag_x * a_mag[0][0] + mag_y * a_mag[0][1] + mag_z * a_mag[0][2];
  mag_y = mag_x * a_mag[1][0] + mag_y * a_mag[1][1] + mag_z * a_mag[1][2];
  mag_z = mag_x * a_mag[2][0] + mag_y * a_mag[2][1] + mag_z * a_mag[2][2];

  /* Compute an Initial Quaternion */
  initial_quaternion = ComputeInitialQuaternions(mag_x, mag_y);

  /* DMP Configuration */
  imu.dmpBegin(
    DMP_FEATURE_SEND_RAW_ACCEL | /* Allows for Raw Accelerometer Readings  */
      DMP_FEATURE_6X_LP_QUAT |   /* Allows for 6-axis Quaternion calculations */
      DMP_FEATURE_GYRO_CAL,      /* Allows for Gyroscope Calibration */
    10);
}

void loop() {
  switch (current_state) {
    case STATE_IDLE:
      {
        Serial.println("Wearable Status: IDLE");
        unsigned long currentTime = millis();
        if (currentTime - lastTriggerTime > DEBOUNCE_TIME) {
          current_state = STATE_INFERENCE;
          lastTriggerTime = currentTime;
        }

        break;
      }

    case STATE_INFERENCE:
      {
        digitalWrite(LEDR, 1);
        digitalWrite(LEDG, 0);
        digitalWrite(LEDB, 1);

        if (imu.fifoAvailable()) {
          if (imu.dmpUpdateFifo() == INV_SUCCESS) {
            run_continuous_inference();
          }
        }

        break;
      }
  }
}

int run_continuous_inference() {
  /* Fixed Interval Sampling every 40 ms (or 25 Hz)*/
  if (millis() - last_sample_time < SAMPLE_RATE_MS) {
    return 0;
  }
  last_sample_time = millis();

  unsigned long timestamp = millis();
  // ClassificationTimestamp.writeValue((byte *)&timestamp, sizeof(timestamp));

  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);
  float ax = imu.calcAccel(imu.ax);
  float ay = imu.calcAccel(imu.ay);
  float az = imu.calcAccel(imu.az);

  float raw_features_buffer[7] = { q0, q1, q2, q3, ax, ay, az };

  float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

  for (int i = 0; i < 7; i++) {
    inference_buffer[i] = raw_features_buffer[i];
  }

  for (int i = 7; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i++) {
    inference_buffer[i] = raw_features_buffer[i % 7];
  }

  size_t inference_buffer_size = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  Serial.print("Running inference with buffer size: ");
  Serial.println(inference_buffer_size);

  signal_t signal;
  numpy::signal_from_buffer(inference_buffer, inference_buffer_size, &signal);
  run_classifier_init();

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR res = run_classifier_continuous(&signal, &result, false);

  ei_printf("Inference Results:\r\n");

  String predicted_gesture = "";
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("  %s: %.5f\r\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
    // ClassificationValue.writeValue((byte *)&result.classification[i].value, sizeof(result.classification[i].value));

    if ((result.classification[i].value > CONFIDENCE_THRESHOLD)) {
      predicted_gesture = result.classification[i].label;
      Serial.println(predicted_gesture);

      // ClassificationLabel.writeValue((byte *)&predicted_gesture, sizeof(predicted_gesture));

      digitalWrite(LEDG, 1);
      delay(100);
      digitalWrite(LEDG, 0);
    }
  }

  ei_printf("Profiling: %d\r\n", res);
  ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

  // DspTiming.writeValue((byte *)&result.timing.dsp, sizeof(result.timing.dsp));
  // TimingClassification.writeValue((byte *)&result.timing.classification, sizeof(result.timing.classification));
  // TimingAnomaly.writeValue((byte *)&result.timing.anomaly, sizeof(result.timing.anomaly));

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
  // ClassificationAnomaly.writeValue((byte *)&result.anomaly, sizeof(result.anomaly));
#endif

  return 0;
}

Quaternion ComputeInitialQuaternions(float mx, float my) {
  /* Compute yaw from mag x,y and return a quaternion around Z */
  float yaw = atan2f(-my, mx);
  float half_yaw = yaw * 0.5f;

  Quaternion q;
  q.w = cosf(half_yaw);
  q.x = 0.0f;
  q.y = 0.0f;
  q.z = sinf(half_yaw);
  return q;
}

void InitializeBLE(void) {
  if (!BLE.begin()) {
    while (1) {
      Serial.println("Failed to initialize BLE!");
      delay(5000);
    }
  }

  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setAdvertisingInterval(BLE_ADVERTISING_INTERVAL);
  BLE.setAdvertisedService(InferenceDataService);

  InferenceDataService.addCharacteristic(ClassificationTimestamp);
  InferenceDataService.addCharacteristic(ClassificationValue);
  InferenceDataService.addCharacteristic(ClassificationLabel);
  InferenceDataService.addCharacteristic(ClassificationAnomaly);
  InferenceDataService.addCharacteristic(DspTiming);
  InferenceDataService.addCharacteristic(TimingClassification);
  InferenceDataService.addCharacteristic(TimingAnomaly);

  BLE.addService(InferenceDataService);
  BLE.advertise();

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  Serial.println("The Mac Address of the BLE Device is: " + BLE.address());
}

void blePeripheralConnectHandler(BLEDevice central) {
  Serial.println("The peripheral has connected to device with MAC Address: " + central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  Serial.println("The peripheral has disconnected from device with MAC Address: " + central.address());
}