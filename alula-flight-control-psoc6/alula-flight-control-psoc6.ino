#include <WiFi.h>
#include <WiFiUdp.h>
#include "secrets.h"
#include <ServoC.h>
#include <Dps3xx.h>
#include <TaskScheduler.h>

// WLAN access
const char* ssid     = NET_SECRET_SSID;
const char* password = NET_SECRET_PASSWORD;

// UDP settings
WiFiUDP udp;
unsigned int localPort = 5005;

// Servos
ServoC servo_left;
ServoC servo_right;
unsigned long lastServoUpdate = 0;
const int SERVO_INTERVAL = 50;

// DPS368 Temperature/Pressure Sensor
Dps3xx Dps3xxPressureSensor = Dps3xx();

// Shared state (updated from UDP packets)
volatile uint8_t target_left  = 90;
volatile uint8_t target_right = 90;
volatile uint8_t target_pitch = 90;  // byte[4]: pitch / up-down (90 = neutral)
volatile bool    steer_active = false;

// Smoothing state
int current_left  = 90;
int current_right = 90;

// Max servo step per 15 ms
// Absorbs jitter
const int MAX_STEP = 4;

//Failsafe variables
unsigned long lastPackageTime = millis();
const unsigned long TIMEOUT_CONNECTION_LOST = 500;
bool hasEverReceived = false;




// ─────────────────────────────────────────────────────────────────────
//  Elevon mixer:
//    Roll  → left and right move in opposite direction
//    Pitch → left and right move in the same direction
//
//  Android app sends:
//    byte[2]  left_servo  = 90 – roll*90   (pure roll)
//    byte[3]  right_servo = 90 + roll*90   (pure roll)
//    byte[4]  pitch       = 90 + pitch*90  (pure pitch, 90 = neutral)
//
//  Mixing:
//    pitch_offset = pitch_byte – 90        (range: –90 … +90)
//    final_left   = left_byte  + pitch_offset
//    final_right  = right_byte + pitch_offset
//
//  Result: roll deflection remains differential, pitch shifts both
//          servos by the same amount simultaneously.
// ─────────────────────────────────────────────────────────────────────
static inline int mixedAngle(uint8_t roll_byte, uint8_t pitch_byte) {
  int pitch_offset = (int)pitch_byte - 90;
  int mixed = (int)roll_byte + pitch_offset;
  if (mixed < 0)   mixed = 0;
  if (mixed > 180) mixed = 180;
  return mixed;
}



// Failsafe navigation (circle downwards)
void Failsafe(){
    target_left = 100;
    target_right = 80;
    target_pitch = 95;
    steer_active = 1;
    //printf("Connection Lost. Initialize safe downward spiral");
}

// Tasks
/* void autopilot();
Scheduler scheduler;
Task AutoPilotTask(, TASK_FOREVER, &autopilot); */






void setup() {
  Serial.begin(115200);
  delay(2000);

  Dps3xxPressureSensor.begin(Wire);

  Serial.println("--- PSoC 6 SoftAP & UDP Server ---");

  // WiFi first
  WiFi.beginAP(ssid, password);
  delay(500);

  // Attach servos after WiFi
  servo_left.attach(0);
  servo_right.attach(1);

  Serial.print("AP active! SSID: ");
  Serial.println(ssid);
  Serial.print("PSoC IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.print("UDP server ready on port: ");
  Serial.println(localPort);
  Serial.println("Packet format: [0xAA, armed, left, right, pitch]");
  Serial.println("----------------------------------");

/*   scheduler.init();
  scheduler.addTask(AutoPilotTask); */
}


/* void autopilot()
{
  float temperature;
  float pressure;
  uint oversampling = 2;

  int result_temp = Dps3xxPressureSensor.measureTempOnce(temperature, oversampling);
  int result_pressure = Dps3xxPressureSensor.measurePressureOnce(pressure, oversampling);
  if(result_temp && result_pressure != 0)
  {
    
  }
} */



void loop() {
  // DSP368 variables 
  float temperature;
  float pressure;
  uint8_t oversampling = 7;


  // Drain UDP buffer - keep only the latest packet to avoid command lag
  uint8_t last_steer = 0, last_left = 90, last_right = 90, last_pitch = 90;
  bool got_packet = false;


  while (true) {
          int n = udp.parsePacket();
          if (n < 5) break; 

          uint8_t header = udp.read();
          if (header != 0xAA) continue;  // misaligned - discard

          last_steer = udp.read();   // byte[1]: steering active flag
          last_left  = udp.read();   // byte[2]: roll left  servo angle (0–180)
          last_right = udp.read();   // byte[3]: roll right servo angle (0–180)
          last_pitch = udp.read();   // byte[4]: pitch angle (0–180, 90 = neutral)
          got_packet = true;
          lastPackageTime = millis();
          hasEverReceived = true;
  }

  if(hasEverReceived && (millis() - lastPackageTime >= TIMEOUT_CONNECTION_LOST) )
      {
        Failsafe();
      }

  if (got_packet) 
      {
          steer_active  = (last_steer != 0);
          target_left   = last_left;
          target_right  = last_right;
          target_pitch  = last_pitch;

        Serial.print("RX: armed=");
        Serial.print(steer_active ? 1 : 0);
        Serial.print(" L="); Serial.print(target_left);
        Serial.print(" R="); Serial.print(target_right);
        Serial.print(" P="); Serial.println(target_pitch);
      }

  if(steer_active)
  {

  }

  unsigned long currentMillis = millis();
  if(currentMillis - lastServoUpdate >= SERVO_INTERVAL)
      {
        lastServoUpdate = currentMillis;

          // Resolve angles with elevon mixing
          int goal_left, goal_right;
          if (steer_active) 
              {
                  goal_left  = mixedAngle(target_left,  target_pitch);
                  goal_right = mixedAngle(target_right, target_pitch);
              } 
          else 
              {
                  goal_left  = 90;
                  goal_right = 90;
              }

          // Smooth movement - limit rate of change to MAX_STEP
          if      (current_left < goal_left)  current_left  = min(current_left  + MAX_STEP, goal_left);
          else if (current_left > goal_left)  current_left  = max(current_left  - MAX_STEP, goal_left);

          if      (current_right < goal_right) current_right = min(current_right + MAX_STEP, goal_right);
          else if (current_right > goal_right) current_right = max(current_right - MAX_STEP, goal_right);

          // update signals
          servo_left.write(current_left);
          servo_right.write(current_right);
      }
}