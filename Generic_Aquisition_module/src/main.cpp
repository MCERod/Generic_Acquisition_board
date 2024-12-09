/******************************/
/****       -Defines-      ****/
/******************************/

#define MPU6050 1
#define ADXL345 0
#define A01NYUB 1
#define SEN07024 0
#define SEN0413 0
#define vl53l0 0
#define lidar7 0




#define READINGS_NUM 6
#define IMU_N_READINGS 6  
/*******************************/
/****       -Includes-      ****/
/*******************************/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#if ADXL345
  #include <Adafruit_ADXL345_U.h>
#endif
#if MPU6050
  #include <Adafruit_MPU6050.h>
#endif

/*******************************************/
/****       -Function defenitions-      ****/
/*******************************************/

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void IRAM_ATTR onTimer();
void init_sensor();
void read_sensor(int j);
void ultrassonic_sensor();
float calcularMediana(float arr[], int tamanho);

/********************************/
/****       -Variables-      ****/
/********************************/

uint8_t requesterMAC[6];
hw_timer_t *timer = NULL;
#define TIMER_INTERVAL_US 150000
unsigned char data[4];
int distance = 0;
volatile int timer_en = 0;

unsigned long last_transmission = 0;

float Mediana_x = 0;
float Mediana_y = 0;
float Mediana_z = 0;


float imu_data_x[IMU_N_READINGS];
float imu_data_y[IMU_N_READINGS];
float imu_data_z[IMU_N_READINGS];


typedef struct struct_message
{
  float ax = 0;
  float ay = 0;
  float az = 0;
  float gx = 0;
  float gy = 0;
  float gz = 0;
  int distance = 0;
  int tempo = 0;
} struct_message;
  struct_message sensor_data[READINGS_NUM];
  esp_now_peer_info_t peerInfo;


/********************************/
/****       -Functions-      ****/
/********************************/

SoftwareSerial mySerial(4, 5);
#if MPU6050
Adafruit_MPU6050 mpu;
#endif
#if ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#endif
#if SEN07022
Adafruit_SGP30 sgp;
#endif


void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
  Serial.println("Starting setup...");
  WiFi.mode(WIFI_STA);
  
  Serial.println("WiFi Mode set to STA.");

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    while (1)
    {
      Serial.println("Error initializing ESP-NOW");
    }
    
    return;
  }
  Serial.println("ESP-NOW initialized");

  // Register the callback to receive data
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW started and callback registered");

  timer = timerBegin(0, 80, true);                 // Timer 0, prescaler of 80 (1 tick = 1 µs)
  timerAttachInterrupt(timer, &onTimer, true);     // Attach ISR to the timer
  timerAlarmWrite(timer, TIMER_INTERVAL_US, true); // Set alarm interval to 150 ms

}

void loop()
{
  //delay(1000);
  //Serial.println("Waiting for ESPNOW...");
  if(timer_en == 1){
    //Serial.println("Timer enabled");

      for (int i = 0; i < READINGS_NUM; i++)
      {
        read_sensor(i);
        sensor_data[i].ax = Mediana_x;
        sensor_data[i].ay = Mediana_y;
        sensor_data[i].az = Mediana_z;
        sensor_data[i].distance = distance;
        #if A01NYUB || SEN07024
        ultrassonic_sensor();
      #endif
      sensor_data[i].tempo = millis() - last_transmission;
      Serial.println(sensor_data[i].tempo);
      }

      
      esp_err_t result = esp_now_send(requesterMAC, (uint8_t *)&sensor_data, sizeof(sensor_data));
      timer_en = 0;
      Serial.println("Data sent");
      timerAlarmDisable(timer);
  }

}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  last_transmission = millis();
  Serial.println("Received signal to send data");
  memcpy(requesterMAC, mac_addr, 6); // Store the requester's MAC address
   
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, requesterMAC, 6);
  peerInfo.channel = 0; // Canal padrão (ajuste se necessário)
  peerInfo.encrypt = false;

  // Adiciona o peer se ainda não foi adicionado
  if (!esp_now_is_peer_exist(requesterMAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  timerAlarmEnable(timer);           // Enable the timer alarm
}

void IRAM_ATTR onTimer()
{
  timerAlarmDisable(timer); // Disable the timer alarm
  timer_en = 1;
}

void init_sensor()
{
#if MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(1000);
      Serial.println("Failed to find MPU6050 chip");
    }
  }
  Serial.println("MPU6050 Found!");
#endif
#if ADXL345
  if (!accel.begin())
  {
    Serial.println("Failed to find ADXL345 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("ADXL345 Found!");
#endif
}

void read_sensor(int j)
{ // store sensor data in struct_message

#if MPU6050
for(int i = 0; i < 6; i++){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("a.acceleration.x: ");
  Serial.println(a.acceleration.x);
  imu_data_x[i] = a.acceleration.x;
  imu_data_y[i] = a.acceleration.y;
  imu_data_z[i] = a.acceleration.z;
  delay(2);
}
Mediana_x = calcularMediana(imu_data_x, IMU_N_READINGS);
Mediana_y = calcularMediana(imu_data_y, IMU_N_READINGS);
Mediana_z = calcularMediana(imu_data_z, IMU_N_READINGS);
Serial.print("Mediana X: ");
Serial.println(Mediana_x);
Serial.print("Mediana Y: ");
Serial.println(Mediana_y);
Serial.print("Mediana Z: ");
Serial.println(Mediana_z);
#endif
#if ADXL345
  sensors_event_t event;
  accel.getEvent(&event);
  sensor_data.ax = event.acceleration.x;
  sensor_data.ay = event.acceleration.y;
  sensor_data.az = event.acceleration.z;
#endif
  //sensor_data.distance = 0; // distance sensor data
  //sensor_data.tempo = 0;    // tempo sensor data
}

void ultrassonic_sensor()
{
  while (mySerial.available() < 4)
  {
    Serial.println("Waiting for data...");
  }
 
  
    if (mySerial.available() >= 4)
    {
        for (int i = 0; i < 4; i++)
        {
            data[i] = mySerial.read();
        }
        int sum = (data[0] + data[1] + data[2]) & 0x00FF;
        if (sum == data[3])
        {
            distance = (data[1] << 8) + data[2];
            if (distance > 280)
            {
                Serial.print("distance=");
                Serial.print(distance / 10);
                Serial.println("cm");
            }
            else
            {
                Serial.println("Below the lower limit");
            }
        }
        else
        {
            Serial.println("ERROR");
        }
    }
}


// Função de comparação para qsort
int comparar(const void *a, const void *b) {
    if (*(float*)a < *(float*)b) return -1;
    if (*(float*)a > *(float*)b) return 1;
    return 0;
}

float calcularMediana(float arr[], int tamanho) {
    // Ordena o array
    qsort(arr, tamanho, sizeof(float), comparar);

    // Calcula a mediana
    if (tamanho % 2 == 0) {
        int meio1 = tamanho / 2 - 1;
        int meio2 = tamanho / 2;
        return (arr[meio1] + arr[meio2]) / 2.0;
    } else {
        return arr[tamanho / 2];
    }
}