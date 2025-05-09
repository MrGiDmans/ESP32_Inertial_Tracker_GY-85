#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <ITG3200.h>
#include <I2Cdev.h>
#include <math.h>
#include <WiFiUdp.h>
#include <EEPROM.h>

// Константы
#define SDA_PIN 22
#define SCL_PIN 23
#define EEPROM_SIZE 26
#define MAGIC_NUMBER (uint16_t)0xA5A5
#define SAMPLE_RATE 100  // Частота опроса датчиков (Гц)
#define SEND_RATE 50     // Частота отправки данных (Гц)
#define GYRO_SAMPLES 500 // Количество сэмплов для калибровки гироскопа
#define MAG_CALIB_TIME 10000 // Время калибровки магнитометра (мс)
#define ALPHA 0.96       // Коэффициент комплементарного фильтра (0-1)

// Семафоры для синхронизации
SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t cmdMutex = NULL;

// Датчики
Adafruit_ADXL345_Unified accel(12345);
Adafruit_HMC5883_Unified mag(54321);
ITG3200 gyro;

// Данные датчиков
struct SensorData {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
} sensorData;

// Углы ориентации
struct Orientation {
    float roll, pitch, yaw;
    float roll_gyro, pitch_gyro, yaw_gyro;  // Углы от гироскопа
    float roll_acc, pitch_acc, yaw_acc;     // Углы от акселерометра
    float roll_mag, pitch_mag, yaw_mag;     // Углы от магнитометра
} orientation;

// Wi-Fi и UDP
WiFiUDP udp;
const char* ssid = "ESP32_Tracker";
const char* password = "123456789";
IPAddress remote_ip(192, 168, 4, 2);
const unsigned int remote_port = 12345;
const unsigned int listen_port = 12346;

// Калибровка и режимы
volatile bool calibrate_gyro_flag = false;
volatile bool calibrate_mag_flag = false;
volatile bool precise_mode = false;

float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;
float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;

bool saveCalibration() {
    // Проверка размера
    if (EEPROM.length() < EEPROM_SIZE) {
        Serial.println("Ошибка: EEPROM слишком мал");
        return false;
    }

    // Запись данных
    EEPROM.put(0, MAGIC_NUMBER);
    EEPROM.put(2, gyro_offset_x);
    EEPROM.put(6, gyro_offset_y);
    EEPROM.put(10, gyro_offset_z);
    EEPROM.put(14, mag_offset_x);
    EEPROM.put(18, mag_offset_y);
    EEPROM.put(22, mag_offset_z);
    
    // Проверка успешности записи
    if (!EEPROM.commit()) {
        Serial.println("Ошибка записи в EEPROM");
        return false;
    }
    
    return true;
}

bool loadCalibration() {
    // Проверка размера
    if (EEPROM.length() < EEPROM_SIZE) {
        Serial.println("Ошибка: EEPROM слишком мал");
        return false;
    }

    uint16_t check;
    EEPROM.get(0, check);
    
    if (check == MAGIC_NUMBER) {
        EEPROM.get(2, gyro_offset_x);
        EEPROM.get(6, gyro_offset_y);
        EEPROM.get(10, gyro_offset_z);
        EEPROM.get(14, mag_offset_x);
        EEPROM.get(18, mag_offset_y);
        EEPROM.get(22, mag_offset_z);
        return true;
    } else {
        // Сброс калибровки при неверном MAGIC_NUMBER
        gyro_offset_x = gyro_offset_y = gyro_offset_z = 0;
        mag_offset_x = mag_offset_y = mag_offset_z = 0;
        return false;
    }
}

// Вычисление углов из акселерометра
void calculateAccelAngles() {
    float accX = sensorData.accX;
    float accY = sensorData.accY;
    float accZ = sensorData.accZ;
    
    // Вычисление углов из акселерометра
    orientation.roll_acc = atan2(accY, accZ) * RAD_TO_DEG;
    orientation.pitch_acc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
}

// Вычисление углов из магнитометра
void calculateMagAngles() {
    float magX = sensorData.magX;
    float magY = sensorData.magY;
    float magZ = sensorData.magZ;
    
    // Вычисление углов из магнитометра
    orientation.yaw_mag = atan2(magY, magX) * RAD_TO_DEG;
    if (orientation.yaw_mag < 0) orientation.yaw_mag += 360.0;
}

float interpolateAngle(float angle1, float angle2, float alpha) {
    float diff = angle2 - angle1;

    // Обработка переполнения
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    float result = angle1 + alpha * diff;

    // Нормализация в [0, 360)
    if (result < 0) result += 360.0f;
    if (result >= 360.0f) result -= 360.0f;

    return result;
}

// Комплементарный фильтр
void complementaryFilter(float dt) {
    // Получаем радианы для тригонометрии
    float rollRad = orientation.roll * DEG_TO_RAD;
    float pitchRad = orientation.pitch * DEG_TO_RAD;
    
    if (precise_mode) {
        // Точный режим — без инерции, чисто по акселерометру и магнитометру
        orientation.roll = orientation.roll_acc;
        orientation.pitch = orientation.pitch_acc;
        
        // yaw — только по магнитометру с компенсацией наклона
        float Xh = sensorData.magX * cos(pitchRad) + sensorData.magZ * sin(pitchRad);
        float Yh = sensorData.magX * sin(rollRad) * sin(pitchRad) + 
                  sensorData.magY * cos(rollRad) - 
                  sensorData.magZ * sin(rollRad) * cos(pitchRad);
        
        orientation.yaw = atan2(Yh, Xh) * RAD_TO_DEG;
        if (orientation.yaw < 0) orientation.yaw += 360.0;
    } else {
        // Обычный режим — комплементарный фильтр
        // Интегрирование углов от гироскопа
        orientation.roll_gyro += sensorData.gyroX * dt;
        orientation.pitch_gyro += sensorData.gyroY * dt;
        orientation.yaw_gyro += sensorData.gyroZ * dt;
        
        // Нормализация углов гироскопа
        if (orientation.roll_gyro > 180) orientation.roll_gyro -= 360;
        if (orientation.roll_gyro < -180) orientation.roll_gyro += 360;
        if (orientation.pitch_gyro > 180) orientation.pitch_gyro -= 360;
        if (orientation.pitch_gyro < -180) orientation.pitch_gyro += 360;
        if (orientation.yaw_gyro > 360) orientation.yaw_gyro -= 360;
        if (orientation.yaw_gyro < 0) orientation.yaw_gyro += 360;
        
        // Комплементарный фильтр
        orientation.roll = ALPHA * (orientation.roll + sensorData.gyroX * dt) + 
                         (1 - ALPHA) * orientation.roll_acc;
        orientation.pitch = ALPHA * (orientation.pitch + sensorData.gyroY * dt) + 
                          (1 - ALPHA) * orientation.pitch_acc;
        
        // yaw с инерцией и корректировкой магнитометром
        orientation.yaw += sensorData.gyroZ * dt;  // инерционная часть
        
        // Корректировка yaw магнитометром с компенсацией наклона
        float Xh = sensorData.magX * cos(pitchRad) + sensorData.magZ * sin(pitchRad);
        float Yh = sensorData.magX * sin(rollRad) * sin(pitchRad) + 
                  sensorData.magY * cos(rollRad) - 
                  sensorData.magZ * sin(rollRad) * cos(pitchRad);
        float magYaw = atan2(Yh, Xh) * RAD_TO_DEG;
        if (magYaw < 0) magYaw += 360.0;

        // Плавная коррекция yaw с учётом переполнения
        orientation.yaw = interpolateAngle(orientation.yaw, magYaw, 1.0f - ALPHA);
        // orientation.yaw = interpolateAngle(orientation.yaw, magYaw, 0.01);
    }
    
    // Нормализация финальных углов
    if (orientation.roll > 180) orientation.roll -= 360;
    if (orientation.roll < -180) orientation.roll += 360;
    if (orientation.pitch > 180) orientation.pitch -= 360;
    if (orientation.pitch < -180) orientation.pitch += 360;
    if (orientation.yaw > 360) orientation.yaw -= 360;
    if (orientation.yaw < 0) orientation.yaw += 360;
}

// Калибровка гироскопа
void calibrateGyro() {
    static int samples = 0;
    static float sum_x = 0, sum_y = 0, sum_z = 0;
    
    // Чтение гироскопа
    int16_t gx_raw, gy_raw, gz_raw;
    gyro.getRotation(&gx_raw, &gy_raw, &gz_raw);
    float gyroX = gx_raw / 14.375;
    float gyroY = gy_raw / 14.375;
    float gyroZ = gz_raw / 14.375;
    
    sum_x += gyroX;
    sum_y += gyroY;
    sum_z += gyroZ;
    samples++;
    
    if (samples >= GYRO_SAMPLES) {
        gyro_offset_x = sum_x / samples;
        gyro_offset_y = sum_y / samples;
        gyro_offset_z = sum_z / samples;
        calibrate_gyro_flag = false;
        saveCalibration();
        
        udp.beginPacket(remote_ip, remote_port);
        udp.print("end_calibrate_gyro");
        udp.endPacket();
        
        samples = 0;
        sum_x = sum_y = sum_z = 0;
    }
}

// Калибровка магнитометра
void calibrateMag() {
    static float min_x = 9999, min_y = 9999, min_z = 9999;
    static float max_x = -9999, max_y = -9999, max_z = -9999;
    static unsigned long start_time = millis();
    
    // Чтение магнитометра
    sensors_event_t m;
    mag.getEvent(&m);
    float magX = m.magnetic.x;
    float magY = m.magnetic.y;
    float magZ = m.magnetic.z;
    
    // Обновление минимумов и максимумов
    min_x = min(min_x, magX);
    max_x = max(max_x, magX);
    min_y = min(min_y, magY);
    max_y = max(max_y, magY);
    min_z = min(min_z, magZ);
    max_z = max(max_z, magZ);
    
    if (millis() - start_time > MAG_CALIB_TIME) {
        mag_offset_x = (min_x + max_x) / 2;
        mag_offset_y = (min_y + max_y) / 2;
        mag_offset_z = (min_z + max_z) / 2;
        calibrate_mag_flag = false;
        saveCalibration();
        
        udp.beginPacket(remote_ip, remote_port);
        udp.print("end_calibrate_mag");
        udp.endPacket();
        
        // Сброс переменных
        min_x = min_y = min_z = 9999;
        max_x = max_y = max_z = -9999;
        start_time = millis();
    }
}

// Задача чтения датчиков
void readSensorsTask(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000 / SAMPLE_RATE);
    unsigned long lastTime = 0;
    
    while (true) {
        if (calibrate_gyro_flag) {
            calibrateGyro();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        if (calibrate_mag_flag) {
            calibrateMag();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0;
        if (dt > 1.0) dt = 0.01;
        lastTime = now;
        
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            // Чтение акселерометра
            sensors_event_t a;
            accel.getEvent(&a);
            sensorData.accX = a.acceleration.x;
            sensorData.accY = a.acceleration.y;
            sensorData.accZ = a.acceleration.z;

            // Чтение гироскопа
            int16_t gx_raw, gy_raw, gz_raw;
            gyro.getRotation(&gx_raw, &gy_raw, &gz_raw);
            sensorData.gyroX = (gx_raw / 14.375) - gyro_offset_x;
            sensorData.gyroY = (gy_raw / 14.375) - gyro_offset_y;
            sensorData.gyroZ = (gz_raw / 14.375) - gyro_offset_z;

            // Чтение магнитометра
            sensors_event_t m;
            mag.getEvent(&m);
            sensorData.magX = m.magnetic.x - mag_offset_x;
            sensorData.magY = m.magnetic.y - mag_offset_y;
            sensorData.magZ = m.magnetic.z - mag_offset_z;

            // Вычисление углов
            calculateAccelAngles();
            calculateMagAngles();
            complementaryFilter(dt);

            xSemaphoreGive(dataMutex);
        }
        
        vTaskDelay(xDelay);
    }
}

// Задача отправки данных
void sendDataTask(void* parameter) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000 / SEND_RATE);
    
    while (true) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            // Формат: roll,pitch,yaw
            String data = String(orientation.roll, 2) + "," +
                         String(orientation.pitch, 2) + "," +
                         String(orientation.yaw, 2);
            
            udp.beginPacket(remote_ip, remote_port);
            udp.print(data);
            udp.endPacket();
            
            xSemaphoreGive(dataMutex);
        }
        
        vTaskDelay(xDelay);
    }
}

// Задача обработки команд
void handleCommandsTask(void* parameter) {
    while (true) {
        if (xSemaphoreTake(cmdMutex, portMAX_DELAY) == pdTRUE) {
            int packetSize = udp.parsePacket();
            if (packetSize) {
                char buf[64] = {0};
                udp.read(buf, sizeof(buf));
                String cmd = String(buf);
                cmd.trim();

                remote_ip = udp.remoteIP();
                
                if (cmd == "calibrate_gyro") {
                    calibrate_gyro_flag = true;
                    udp.beginPacket(remote_ip, remote_port);
                    udp.print("start_calibrate_gyro");
                    udp.endPacket();
                }
                else if (cmd == "calibrate_mag") {
                    calibrate_mag_flag = true;
                    udp.beginPacket(remote_ip, remote_port);
                    udp.print("start_calibrate_mag");
                    udp.endPacket();
                }
                else if (cmd == "precise_on") {
                    precise_mode = true;
                    udp.beginPacket(remote_ip, remote_port);
                    udp.print("mode_changed:1");
                    udp.endPacket();
                }
                else if (cmd == "precise_off") {
                    precise_mode = false;
                    udp.beginPacket(remote_ip, remote_port);
                    udp.print("mode_changed:0");
                    udp.endPacket();
                }
            }
            xSemaphoreGive(cmdMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    // Инициализация семафоров
    dataMutex = xSemaphoreCreateMutex();
    cmdMutex = xSemaphoreCreateMutex();
    
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.begin(115200);

    // Настройка WiFi
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(ssid, password, 1, false, 4)) {
        Serial.println("Ошибка создания точки доступа");
    }
    
    // Настройка UDP
    udp.begin(listen_port);
    
    // Инициализация датчиков
    if (!accel.begin()) { Serial.println("Акселерометр не найден"); while (1); }
    if (!gyro.testConnection()) { Serial.println("Гироскоп не найден"); while (1); }
    gyro.initialize();
    if (!mag.begin()) { Serial.println("Магнитометр не найден"); while (1); }

    // Загрузка калибровки
    EEPROM.begin(EEPROM_SIZE);
    if (!loadCalibration()) {
        Serial.println("Калибровка не найдена, используется нулевая калибровка");
    }

    // Создание задач
    xTaskCreatePinnedToCore(readSensorsTask, "Read Sensors", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sendDataTask, "Send Data", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(handleCommandsTask, "Handle Commands", 4096, NULL, 1, NULL, 1);
}

void loop() {
    // Основной цикл пустой, всё обрабатывается в задачах
    vTaskDelay(pdMS_TO_TICKS(1000));
}
