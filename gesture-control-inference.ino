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

  InitializeBLE();

  /* IMU and Magnetometer Configuration */
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
      DMP_FEATURE_LP_QUAT |      /* Allows for Accelerometer and Low-Power Quaternion Calculation */
      DMP_FEATURE_6X_LP_QUAT |   /* Allows for 6-axis Quaternion calculations */
      DMP_FEATURE_GYRO_CAL,      /* Allows for Gyroscope Calibration */
    100);

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

// void loop() {
//   digitalWrite(LEDR, 1);
//   digitalWrite(LEDG, 0);
//   digitalWrite(LEDB, 1);

//   if (imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS) {
//     if (!buffer_filled) return;

//     unsigned long now = millis();
//     if (now - last_hop < HOP_MS) return;
//     last_hop = now;

//     memmove(
//       ei_feature_buffer,
//       ei_feature_buffer + (SHIFT_SAMPLES * FEATURES_PER_SAMPLE),
//       (TOTAL_SAMPLES - SHIFT_SAMPLES) * FEATURES_PER_SAMPLE * sizeof(float));

//     int tail_index = (TOTAL_SAMPLES - SHIFT_SAMPLES) * FEATURES_PER_SAMPLE;

//     bool ok = collect_samples(SHIFT_SAMPLES, tail_index);
//     if (!ok) {
//       Serial.println("Hop fill failed");
//       return;
//     }

//     run_inference();
//   }
// }

void loop() {
  static BLEDevice central;
  if (!central) central = BLE.central();

  if (central) {
    while (central.connected()) {
      /* IDLE State (BLUE LED) */
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDG, LOW);

      Serial.println("=== Start Sampling (4 seconds) ===");

      unsigned long capture_start = millis();
      int last_second = 0;
      int samples_collected = 0;

      int feature_index = 0;
      while (samples_collected < TOTAL_SAMPLES) {
        unsigned long elapsed = millis() - capture_start;
        int current_second = 4 - (elapsed / 1000);

        if (current_second != last_second && current_second >= 0) {
          Serial.print("Countdown: ");
          Serial.print(current_second);
          Serial.println(" s");
          last_second = current_second;

          digitalWrite(LEDR, HIGH);
          delay(100); /* Quick Blink for each Second (RED LED) */
          digitalWrite(LEDR, LOW);
        }

        if (imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS) {
          if (feature_index + FEATURES_PER_SAMPLE <= TOTAL_SAMPLES * FEATURES_PER_SAMPLE) {
            ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qw);
            ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qx);
            ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qy);
            ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qz);

            ei_feature_buffer[feature_index++] = imu.calcAccel(imu.ax);
            ei_feature_buffer[feature_index++] = imu.calcAccel(imu.ay);
            ei_feature_buffer[feature_index++] = imu.calcAccel(imu.az);

            samples_collected++;
          }
        } else {
          delay(1);
        }
      }

      Serial.println("=== Sampling Completed ===");

      /* Inference State (GREEN LED) */
      digitalWrite(LEDG, HIGH);
      run_inference();
      digitalWrite(LEDG, LOW);

      memset(ei_feature_buffer, 0, sizeof(ei_feature_buffer));
      Serial.println("=== Next Sampling (4 seconds) ===");
    }

    /* After disconnecting, reset Central to allow for new connections */
    central = BLEDevice();
  }
}

// bool collect_samples(int n_samples, int start_index) {
//   int samples_collected = 0;
//   int feature_index = start_index;

//   unsigned long start_time = millis();
//   const unsigned long TIMEOUT_MS = 5000;

//   while (samples_collected < n_samples) {
//     if (millis() - start_time > TIMEOUT_MS || millis() < start_time) break;

//     if (imu.fifoAvailable() && imu.dmpUpdateFifo() == INV_SUCCESS) {
//       if (feature_index + FEATURES_PER_SAMPLE <= TOTAL_SAMPLES * FEATURES_PER_SAMPLE) {
//         ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qw);
//         ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qx);
//         ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qy);
//         ei_feature_buffer[feature_index++] = imu.calcQuat(imu.qz);
//         ei_feature_buffer[feature_index++] = imu.calcAccel(imu.ax);
//         ei_feature_buffer[feature_index++] = imu.calcAccel(imu.ay);
//         ei_feature_buffer[feature_index++] = imu.calcAccel(imu.az);
//         samples_collected++;
//       } else break;
//     } else {
//       delay(1);
//     }
//   }

//   return samples_collected == n_samples;
// }

// void run_inference() {
//   signal_t signal;
//   numpy::signal_from_buffer(ei_feature_buffer, EI_FEATURE_BUFFER_SIZE, &signal);

//   ei_impulse_result_t result;
//   EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);

//   if (rc != EI_IMPULSE_OK) {
//     ei_printf("Inference ERROR: %d\n", rc);
//     return;
//   }

//   unsigned long timestamp = millis();
// #if BLE_ON
//   ClassificationTimestamp.writeValue((byte *)&timestamp, sizeof(timestamp));
// #endif

//   ei_printf("\n === Inference Results: ===\n");

//   float best = 0.0f;
//   int best_i = -1;

//   for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
//     ei_printf("%s: %.5f\r\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
// #if BLE_ON
//     ClassificationValue.writeValue((byte *)&result.classification[i].value, sizeof(result.classification[i].value));
// #endif
//     if (result.classification[i].value > best) {
//       best = result.classification[i].value;
//       best_i = i;
//     }
//   }

//   if (best_i >= 0 && best > EI_CLASSIFIER_THRESHOLD) {
//     String predicted_gesture = ei_classifier_inferencing_categories[best_i];
//     Serial.print(">>> Predicted Gesture: ");
//     Serial.println(predicted_gesture);
//     Serial.println("====================");

// #if BLE_ON
//     ClassificationLabel.writeValue((byte *)&predicted_gesture, sizeof(predicted_gesture));
// #endif

//     digitalWrite(LEDG, 1);
//     delay(100);
//     digitalWrite(LEDG, 0);

//     /* Flush buffer by shifting out only the portion that contributed to current classification */
//     // memmove(
//     //   ei_feature_buffer,
//     //   ei_feature_buffer + SHIFT_SAMPLES * FEATURES_PER_SAMPLE,
//     //   (TOTAL_SAMPLES - SHIFT_SAMPLES) * FEATURES_PER_SAMPLE * sizeof(float));

//     /* Refill tail */
//     // collect_samples(SHIFT_SAMPLES, (TOTAL_SAMPLES - SHIFT_SAMPLES) * FEATURES_PER_SAMPLE);

//     memset(ei_feature_buffer, 0, sizeof(ei_feature_buffer));
//     collect_samples(TOTAL_SAMPLES, 0);
//     buffer_filled = true;

//   } else {
//     Serial.println(">>> No Confident Classification.");
//     Serial.println("====================");
//   }

// #if BLE_ON
//   DspTiming.writeValue((byte *)&result.timing.dsp, sizeof(result.timing.dsp));
//   TimingClassification.writeValue((byte *)&result.timing.classification, sizeof(result.timing.classification));
//   TimingAnomaly.writeValue((byte *)&result.timing.anomaly, sizeof(result.timing.anomaly));
// #endif

// #if EI_CLASSIFIER_HAS_ANOMALY == 1
// #if BLE_ON
//   ClassificationAnomaly.writeValue((byte *)&result.anomaly, sizeof(result.anomaly));
// #endif
// #endif

//   /* Inference Diagnostic Logger */
//   {
//     unsigned long now = millis();

//     float best = 0.0f;
//     int best_i = -1;
//     for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
//       if (result.classification[i].value > best) {
//         best = result.classification[i].value;
//         best_i = i;
//       }
//     }

//     /* Confidence Slope Calculation */
//     float delta_p = fabs(best - last_best_conf);
//     unsigned long hop_dt = (last_infer_ts == 0) ? 0 : (now - last_infer_ts);

//     Serial.print("HOP_DT(ms): ");
//     Serial.print(hop_dt);

//     Serial.print(" | LABEL: ");
//     Serial.print(best_i >= 0 ? ei_classifier_inferencing_categories[best_i] : "none");

//     Serial.print(" | CONF: ");
//     Serial.print(best, 4);

//     Serial.print(" | ΔCONF: ");
//     Serial.println(delta_p, 4);

//     if (best_i != last_best_label && last_best_label != -1) {
//       Serial.print("LABEL SWITCH after ");
//       Serial.print(hop_dt);
//       Serial.println(" ms");
//     }

//     last_infer_ts = now;
//     last_best_conf = best;
//     last_best_label = best_i;
//   }
// }

void run_inference() {
  signal_t signal;
  numpy::signal_from_buffer(ei_feature_buffer, TOTAL_SAMPLES * FEATURES_PER_SAMPLE, &signal);

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);
  if (rc != EI_IMPULSE_OK) {
    ei_printf("Inference ERROR: %d\n", rc);
    return;
  }

  unsigned long timestamp = millis();
#if BLE_ON
  ClassificationTimestamp.writeValue((byte *)&timestamp, sizeof(timestamp));
#endif

  float best = 0.0f;
  int best_i = -1;
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf("%s: %.5f\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
#if BLE_ON
    ClassificationValue.writeValue((byte *)&result.classification[i].value, sizeof(result.classification[i].value));
#endif
    if (result.classification[i].value > best) {
      best = result.classification[i].value;
      best_i = i;
    }
  }

  if (best_i >= 0 && best > EI_CLASSIFIER_THRESHOLD) {
    String predicted_gesture = ei_classifier_inferencing_categories[best_i];
    Serial.print(">>> Predicted Gesture: ");
    Serial.println(predicted_gesture);
#if BLE_ON
    ClassificationLabel.writeValue((byte *)&predicted_gesture, sizeof(predicted_gesture));
#endif
  } else {
    Serial.println(">>> No Confident Classification.");
  }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
#if BLE_ON
  ClassificationAnomaly.writeValue((byte *)&result.anomaly, sizeof(result.anomaly));
#endif
#endif
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

  InferenceDataService.addCharacteristic(ClassificationTimestamp);
  InferenceDataService.addCharacteristic(ClassificationValue);
  InferenceDataService.addCharacteristic(ClassificationLabel);
  InferenceDataService.addCharacteristic(ClassificationAnomaly);

  InferenceDataService.addCharacteristic(DspTiming);
  InferenceDataService.addCharacteristic(TimingClassification);
  InferenceDataService.addCharacteristic(TimingAnomaly);

  BLE.addService(InferenceDataService);
  BLE.setAdvertisedService(InferenceDataService);
  
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