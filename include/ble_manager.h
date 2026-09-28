/**
 * @file BLEManager.h
 * @brief Manages BLE services and characteristics for the Gesture Control
 * Wearable
 *
 * This header file defines the Bluetooth Low Energy (BLE) functionality for
 * enabling services and characteristic used by the Arduino Nano 33 BLE Sense
 * Rev2 in the Gesture Control Wearable.
 *
 */

#ifndef BLEMANAGER_H_
#define BLEMANAGER_H_

#include "Arduino.h"
#include "ArduinoBLE.h"

#define BLE_DEVICE_NAME "Arduino Nano 33 BLE Sense Rev2"
#define BLE_LOCAL_NAME "Gesture Control Wearable"
#define BLE_BUFFER_SIZE 20
#define BLE_ADVERTISING_INTERVAL (80 * 0.625) /* in (ms) */

#define QUATERNION_BUFFER_SIZE (4 * sizeof(float))
#define LINEAR_ACCELERATION_BUFFER_SIZE (3 * sizeof(float))

/* BLE Services */
extern BLEService DeviceInformationService;
extern BLEService BatteryService;
extern BLEService CurrentTimeService;
extern BLEService GenericAccessService;
extern BLEService GenericAttributeService;
extern BLEService OrientationDataService;
extern BLEService InferenceDataService;

/* General Access Service Characteristics */
extern BLECharacteristic DeviceName;
extern BLECharacteristic Appearance;

/* General Attribute Service Characteristics */
extern BLECharacteristic ServiceChannged;

/* Device Information Service (DIS) Characteristics */
extern BLECharacteristic ManufacturerName;
extern BLECharacteristic ModelNumber;
extern BLECharacteristic SerialNumber;
extern BLECharacteristic FirmwareRevision;
extern BLECharacteristic HardwareRevision;
extern BLECharacteristic SoftwareRevision;

/* Battery Service (BS) Characteristics */
extern BLECharacteristic BatteryLevel;

/* Current Time Service (CTS) Characteristics*/
extern BLECharacteristic CurrentTime;

/* Orientation Data Service (ODS) Characteristics */
extern BLECharacteristic Timestamp;
extern BLECharacteristic Quaternions;
extern BLECharacteristic LinearAcceleration;

/* Inference Data Service (IDS) Characteristics */

/* Classification Characteristics */
extern BLECharacteristic ClassificationTimestamp;
extern BLECharacteristic ClassificationValue;
extern BLECharacteristic ClassificationLabel;
extern BLECharacteristic ClassificationAnomaly;

/* DSP Profiling Characteristics */
extern BLECharacteristic DspTiming;
extern BLECharacteristic TimingClassification;
extern BLECharacteristic TimingAnomaly;

extern char quaternion_buffer[BLE_BUFFER_SIZE];
extern char linear_acceleration_buffer[BLE_BUFFER_SIZE];

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the Bluetooth Low Energy (BLE) functionality for the
 * device.
 *
 * This function sets up the BLE peripheral with device name and services. It:
 * 1. Begins the BLE peripheral operation
 * 2. Sets the device and local name for the BLE device
 * 3. Sets up and adds the following services:
 *    - Generic Access Service
 *    - Generic Attribute Service
 *    - Device Information Service
 *    - Battery Service
 *    - Current Time Service
 *    - Custom Orientation Data Service
 * 4. Starts advertising the BLE peripheral
 *
 * The function blocks if BLE initialization fails until it succeeds.
 *
 * @return void
 */
void InitializeBLE(void);

void blePeripheralConnectHandler(BLEDevice central);

void blePeripheralDisconnectHandler(BLEDevice central);

#ifdef __cplusplus
}

#endif /* End of CPP Guard */
#endif /* BLEMANAGER_H_ */
