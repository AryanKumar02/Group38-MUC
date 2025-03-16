#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include "rotational_model_data.h"
#include "vertical_model_data.h"
#include "horizontal_model_data.h"

BLEService gymService("19b9d254-3c2a-44b4-95d0-11746426f144");
BLEFloatCharacteristic rotationTotalCharacteristic("ff7f", BLERead | BLENotify);
BLEIntCharacteristic repCountCharacteristic("793d", BLERead | BLENotify);
BLEIntCharacteristic distanceCountCharacteristic("7a2e", BLERead | BLENotify);
BLEIntCharacteristic modeCharacteristic("330d", BLERead | BLEWrite);
BLEIntCharacteristic sessionStatusCharacteristic("a200", BLERead | BLEWrite);
BLEStringCharacteristic motionQualityCharacteristic("00a3", BLERead | BLENotify, 20);

float cumulativeRotationDegrees = 0.0;
int repCount = 0;
int operationMode = 0;
int sessionStatus = 0;
String motionQualityString = "";
unsigned long previousTimeMicros = 0;
int lastSentReps = -1;

bool calibrationDone = false;
unsigned long calibrationStartTime = 0;

const float gyroscopeNoiseThreshold = 10.0;
const float rotationRepThresholdDegrees = 20.0;
const float accelerationNoiseThreshold = 0.1;
const float pressingRepThresholdDistance = 0.001;
const unsigned long CALIBRATION_DELAY_MS = 5000;
const unsigned long INACTIVITY_MS = 300;

int rotationDirection = 0;
float accumulatedAngleDegrees = 0.0;
float halfRepAngle = 0.0;

float baselineY = 0.0;
float verticalTotalDistance = 0.0;
float verticalVelocity = 0.0;

float baselineX = 0.0;
float horizontalTotalDistance = 0.0;
float horizontalVelocity = 0.0;

const char* rep_quality_labels[] = {"bad", "excellent", "good", "okay", "perfect"};

#include <TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

constexpr int kTensorArenaSize = 2000;
uint8_t tensor_arena[kTensorArenaSize];
const tflite::Model* model_ptr = nullptr;
tflite::AllOpsResolver resolver_tflite;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input_tensor = nullptr;
TfLiteTensor* output_tensor = nullptr;

constexpr int kVerticalTensorArenaSize = 2000;
uint8_t vertical_tensor_arena[kVerticalTensorArenaSize];
const tflite::Model* vertical_model_ptr = nullptr;
tflite::MicroInterpreter* vertical_interpreter = nullptr;
TfLiteTensor* vertical_input_tensor = nullptr;
TfLiteTensor* vertical_output_tensor = nullptr;

constexpr int kHorizontalTensorArenaSize = 2000;
uint8_t horizontal_tensor_arena[kHorizontalTensorArenaSize];
const tflite::Model* horizontal_model_ptr = nullptr;
tflite::MicroInterpreter* horizontal_interpreter = nullptr;
TfLiteTensor* horizontal_input_tensor = nullptr;
TfLiteTensor* horizontal_output_tensor = nullptr;

void resetSession() {
  cumulativeRotationDegrees = 0;
  repCount = 0;
  lastSentReps = -1;
  rotationDirection = 0;
  accumulatedAngleDegrees = 0.0;
  halfRepAngle = 0.0;
  verticalTotalDistance = 0.0;
  verticalVelocity = 0.0;
  horizontalTotalDistance = 0.0;
  horizontalVelocity = 0.0;
  previousTimeMicros = micros();
  calibrationDone = false;
  calibrationStartTime = millis();
  Serial.println("Session reset: All integration values cleared.");
}

void calibrateVerticalAxis() {
  float sum = 0;
  int count = 0;
  Serial.println("Calibrating Y-axis (vertical)... keep device still.");
  for (int i = 0; i < 100; i++) {
    float ax, ay, az;
    if (IMU.accelerationAvailable() && IMU.readAcceleration(ax, ay, az)) {
      sum += ay;
      count++;
    }
    delay(10);
  }
  if (count > 0) baselineY = sum / count;
  Serial.print("Calibrated baselineY: ");
  Serial.println(baselineY);
}

void calibrateHorizontalAxis() {
  float sum = 0;
  int count = 0;
  Serial.println("Calibrating X-axis (horizontal)... keep device still.");
  for (int i = 0; i < 100; i++) {
    float ax, ay, az;
    if (IMU.accelerationAvailable() && IMU.readAcceleration(ax, ay, az)) {
      sum += ax;
      count++;
    }
    delay(10);
  }
  if (count > 0) baselineX = sum / count;
  Serial.print("Calibrated baselineX: ");
  Serial.println(baselineX);
}

void onModeWritten(BLEDevice central, BLECharacteristic characteristic) {
  int newMode = modeCharacteristic.value();
  if (newMode != operationMode) {
    operationMode = newMode;
    resetSession();
    Serial.print("Mode updated to: ");
    Serial.println(operationMode);
    if (operationMode == 1) {
      calibrationDone = true;
    } else {
      calibrationDone = false;
      calibrationStartTime = millis();
    }
  }
}

void onSessionStatusWritten(BLEDevice central, BLECharacteristic characteristic) {
  sessionStatus = sessionStatusCharacteristic.value();
  Serial.print("Session status updated to: ");
  Serial.println(sessionStatus);
  if (sessionStatus == 1) {
    resetSession();
    if (operationMode != 1) {
      calibrationDone = false;
      calibrationStartTime = millis();
    } else {
      calibrationDone = true;
    }
  }
}

void initModel() {
  model_ptr = tflite::GetModel(rotational_model_data);
  if (model_ptr->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema version mismatch!");
    while (1);
  }
  static tflite::MicroInterpreter static_interpreter(
      model_ptr, resolver_tflite, tensor_arena, kTensorArenaSize, nullptr);
  interpreter = &static_interpreter;
  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed");
    while (1);
  }
  input_tensor = interpreter->input(0);
  output_tensor = interpreter->output(0);
}

void initVerticalModel() {
  vertical_model_ptr = tflite::GetModel(vertical_model_data);
  if (vertical_model_ptr->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Vertical model schema version mismatch!");
    while (1);
  }
  static tflite::MicroInterpreter static_vertical_interpreter(
      vertical_model_ptr, resolver_tflite, vertical_tensor_arena, kVerticalTensorArenaSize, nullptr);
  vertical_interpreter = &static_vertical_interpreter;
  if (vertical_interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed for vertical model");
    while (1);
  }
  vertical_input_tensor = vertical_interpreter->input(0);
  vertical_output_tensor = vertical_interpreter->output(0);
}

void initHorizontalModel() {
  horizontal_model_ptr = tflite::GetModel(horizontal_model_data);
  if (horizontal_model_ptr->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Horizontal model schema version mismatch!");
    while (1);
  }
  static tflite::MicroInterpreter static_horizontal_interpreter(
      horizontal_model_ptr, resolver_tflite, horizontal_tensor_arena, kHorizontalTensorArenaSize, nullptr);
  horizontal_interpreter = &static_horizontal_interpreter;
  if (horizontal_interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("AllocateTensors() failed for horizontal model");
    while (1);
  }
  horizontal_input_tensor = horizontal_interpreter->input(0);
  horizontal_output_tensor = horizontal_interpreter->output(0);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  if (!BLE.begin()) {
    Serial.println("BLE initialization failed!");
    while (1);
  }
  
  BLE.setLocalName("Gym_Arduino");
  BLE.setAdvertisedService(gymService);
  gymService.addCharacteristic(rotationTotalCharacteristic);
  gymService.addCharacteristic(repCountCharacteristic);
  gymService.addCharacteristic(distanceCountCharacteristic);
  gymService.addCharacteristic(modeCharacteristic);
  gymService.addCharacteristic(sessionStatusCharacteristic);
  gymService.addCharacteristic(motionQualityCharacteristic);
  BLE.addService(gymService);
  
  rotationTotalCharacteristic.writeValue(cumulativeRotationDegrees);
  repCountCharacteristic.writeValue(repCount);
  distanceCountCharacteristic.writeValue(0);
  modeCharacteristic.writeValue(operationMode);
  sessionStatusCharacteristic.writeValue(sessionStatus);
  motionQualityCharacteristic.writeValue("");
  
  modeCharacteristic.setEventHandler(BLEWritten, onModeWritten);
  sessionStatusCharacteristic.setEventHandler(BLEWritten, onSessionStatusWritten);
  
  BLE.advertise();
  
  initModel();
  initVerticalModel();
  initHorizontalModel();
  
  previousTimeMicros = micros();
  Serial.println("IMU, BLE, and TFLite models initialized successfully. Starting rep classification session...");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to Central: ");
    Serial.println(central.address());
    
    while (central.connected()) {
      if ((operationMode == 2 || operationMode == 3) && sessionStatus == 1 && !calibrationDone) {
        if (millis() - calibrationStartTime >= CALIBRATION_DELAY_MS) {
          if (operationMode == 2) {
            calibrateVerticalAxis();
          } else if (operationMode == 3) {
            calibrateHorizontalAxis();
          }
          calibrationDone = true;
          Serial.println("Calibration complete. Starting exercise.");
        } else {
          Serial.print("Calibrating... ");
          Serial.print((CALIBRATION_DELAY_MS - (millis() - calibrationStartTime)) / 1000);
          Serial.println(" seconds remaining");
          BLE.poll();
          delay(200);
          continue;
        }
      }
      
      if (operationMode == 1 && sessionStatus == 1) {
        if (IMU.gyroscopeAvailable()) {
          float gx, gy, gz;
          if (IMU.readGyroscope(gx, gy, gz)) {
            unsigned long currentTime = micros();
            float dt = (currentTime - previousTimeMicros) / 1000000.0;
            previousTimeMicros = currentTime;
            float absGz = fabs(gz);
            if (absGz < gyroscopeNoiseThreshold)
              absGz = 0;
            float deltaDegrees = absGz * dt;
            cumulativeRotationDegrees += deltaDegrees;
            
            if (fabs(gz) >= gyroscopeNoiseThreshold) {
              int readingDirection = (gz > 0) ? 1 : -1;
              if (rotationDirection == 0) {
                rotationDirection = readingDirection;
                accumulatedAngleDegrees = deltaDegrees;
              } else if (readingDirection == rotationDirection) {
                accumulatedAngleDegrees += deltaDegrees;
              } else {
                if (accumulatedAngleDegrees >= rotationRepThresholdDegrees) {
                  if (halfRepAngle == 0.0) {
                    halfRepAngle = accumulatedAngleDegrees;
                  } else {
                    float fullRepDegrees = halfRepAngle + accumulatedAngleDegrees;
                    repCount++;
                    input_tensor->data.f[0] = fullRepDegrees;
                    input_tensor->data.f[1] = cumulativeRotationDegrees;
                    input_tensor->data.f[2] = 1.0;
                    if (interpreter->Invoke() != kTfLiteOk) {
                      Serial.println("Rotation: Invoke failed");
                    } else {
                      float max_score = output_tensor->data.f[0];
                      int max_index = 0;
                      for (int i = 1; i < 5; i++) {
                        if (output_tensor->data.f[i] > max_score) {
                          max_score = output_tensor->data.f[i];
                          max_index = i;
                        }
                      }
                      motionQualityCharacteristic.writeValue(rep_quality_labels[max_index]);
                      repCountCharacteristic.writeValue(repCount);
                      Serial.print("Rotation Rep ");
                      Serial.print(repCount);
                      Serial.print(" (");
                      Serial.print(fullRepDegrees);
                      Serial.print("°): Classified as ");
                      Serial.println(rep_quality_labels[max_index]);
                    }
                    halfRepAngle = 0.0;
                  }
                }
                rotationDirection = readingDirection;
                accumulatedAngleDegrees = deltaDegrees;
              }
            }
            rotationTotalCharacteristic.writeValue(cumulativeRotationDegrees);
            Serial.print("Total rotation (degrees): ");
            Serial.println(cumulativeRotationDegrees);
          }
        }
      }
      
      if (operationMode == 2 && sessionStatus == 1) {
        if (IMU.accelerationAvailable()) {
          float ax, ay, az;
          if (IMU.readAcceleration(ax, ay, az)) {
            float rawAccel = ay;
            static float dynamicBias = baselineY;
            const float biasAlpha = 0.01;
            if (fabs(rawAccel - dynamicBias) < 0.3)
              dynamicBias = (1 - biasAlpha) * dynamicBias + biasAlpha * rawAccel;
            float calibratedAccel = rawAccel - dynamicBias;
            if (fabs(calibratedAccel) < accelerationNoiseThreshold) {
              calibratedAccel = 0;
              verticalVelocity = 0;
            }
            unsigned long currentTime = micros();
            float dt = (currentTime - previousTimeMicros) / 1000000.0;
            previousTimeMicros = currentTime;
            float velocity = verticalVelocity + calibratedAccel * dt;
            float deltaDistance = verticalVelocity * dt + 0.5f * calibratedAccel * dt * dt;
            verticalVelocity = velocity;
            verticalTotalDistance += fabs(deltaDistance);
            static float currentDisp = 0.0f;
            static float maxDisp = 0.0f;
            static float minDisp = 0.0f;
            static unsigned long lastMotionTime = millis();
            currentDisp += deltaDistance;
            if (fabs(calibratedAccel) >= accelerationNoiseThreshold) {
              lastMotionTime = millis();
              if (currentDisp > maxDisp) maxDisp = currentDisp;
              if (currentDisp < minDisp) minDisp = currentDisp;
            } else {
              if (millis() - lastMotionTime > INACTIVITY_MS) {
                float repDistance = maxDisp - minDisp;
                if (repDistance >= pressingRepThresholdDistance) {
                  repCount++;
                  vertical_input_tensor->data.f[0] = repDistance * 100.0;
                  vertical_input_tensor->data.f[1] = verticalTotalDistance * 100.0;
                  vertical_input_tensor->data.f[2] = 1.0;
                  if (vertical_interpreter->Invoke() != kTfLiteOk) {
                    Serial.println("Vertical model invoke failed");
                  } else {
                    float max_score = vertical_output_tensor->data.f[0];
                    int max_index = 0;
                    for (int i = 1; i < 5; i++) {
                      if (vertical_output_tensor->data.f[i] > max_score) {
                        max_score = vertical_output_tensor->data.f[i];
                        max_index = i;
                      }
                    }
                    String verticalLabel = rep_quality_labels[max_index];
                    repCountCharacteristic.writeValue(repCount);
                    motionQualityCharacteristic.writeValue(verticalLabel);
                    Serial.print("Vertical Rep ");
                    Serial.print(repCount);
                    Serial.print(" (");
                    Serial.print(repDistance * 100.0, 4);
                    Serial.print(" cm): Classified as ");
                    Serial.println(verticalLabel);
                  }
                }
                currentDisp = 0.0f;
                maxDisp = 0.0f;
                minDisp = 0.0f;
              }
            }
            distanceCountCharacteristic.writeValue((int)(verticalTotalDistance * 100));
            Serial.print("Vertical Total Distance (cm): ");
            Serial.println(verticalTotalDistance * 100, 4);
            Serial.print("Vertical Rep ");
            Serial.print(repCount);
          }
        }
      }
      
      if (operationMode == 3 && sessionStatus == 1) {
        if (IMU.accelerationAvailable()) {
          float ax, ay, az;
          if (IMU.readAcceleration(ax, ay, az)) {
            float rawAccel = ax;
            static float dynamicBias = baselineX;
            const float biasAlpha = 0.01;
            if (fabs(rawAccel - dynamicBias) < 0.3)
              dynamicBias = (1 - biasAlpha) * dynamicBias + biasAlpha * rawAccel;
            float calibratedAccel = rawAccel - dynamicBias;
            if (fabs(calibratedAccel) < accelerationNoiseThreshold) {
              calibratedAccel = 0;
              horizontalVelocity = 0;
            }
            unsigned long currentTime = micros();
            float dt = (currentTime - previousTimeMicros) / 1000000.0;
            previousTimeMicros = currentTime;
            float velocity = horizontalVelocity + calibratedAccel * dt;
            float deltaDistance = horizontalVelocity * dt + 0.5f * calibratedAccel * dt * dt;
            horizontalVelocity = velocity;
            horizontalTotalDistance += fabs(deltaDistance);
            static float currentDisp = 0.0f;
            static float maxDisp = 0.0f;
            static float minDisp = 0.0f;
            static unsigned long lastMotionTime = millis();
            currentDisp += deltaDistance;
            if (fabs(calibratedAccel) >= accelerationNoiseThreshold) {
              lastMotionTime = millis();
              if (currentDisp > maxDisp) maxDisp = currentDisp;
              if (currentDisp < minDisp) minDisp = currentDisp;
            } else {
              if (millis() - lastMotionTime > INACTIVITY_MS) {
                float repDistance = maxDisp - minDisp;
                if (repDistance >= pressingRepThresholdDistance) {
                  repCount++;
                  horizontal_input_tensor->data.f[0] = repDistance * 100.0;
                  horizontal_input_tensor->data.f[1] = horizontalTotalDistance * 100.0;
                  horizontal_input_tensor->data.f[2] = 1.0;
                  if (horizontal_interpreter->Invoke() != kTfLiteOk) {
                    Serial.println("Horizontal model invoke failed");
                  } else {
                    float max_score = horizontal_output_tensor->data.f[0];
                    int max_index = 0;
                    for (int i = 1; i < 5; i++) {
                      if (horizontal_output_tensor->data.f[i] > max_score) {
                        max_score = horizontal_output_tensor->data.f[i];
                        max_index = i;
                      }
                    }
                    String horizontalLabel = rep_quality_labels[max_index];
                    repCountCharacteristic.writeValue(repCount);
                    motionQualityCharacteristic.writeValue(horizontalLabel);
                    Serial.print("Horizontal Rep ");
                    Serial.print(repCount);
                    Serial.print(" (");
                    Serial.print(repDistance * 100.0, 4);
                    Serial.print(" cm): Classified as ");
                    Serial.println(horizontalLabel);
                  }
                }
                currentDisp = 0.0f;
                maxDisp = 0.0f;
                minDisp = 0.0f;
              }
            }
            distanceCountCharacteristic.writeValue((int)(horizontalTotalDistance * 100));
            Serial.print("Horizontal Total Distance (cm): ");
            Serial.println(horizontalTotalDistance * 100, 4);
            Serial.print("Horizontal Rep ");
            Serial.print(repCount);
          }
        }
      }
      
      BLE.poll();
      delay(50);
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}