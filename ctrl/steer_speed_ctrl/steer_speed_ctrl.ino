#include <NeoPixelBrightnessBus.h>

/******************************************************************************
 * @file    steer_speed_ctrl.ino
 * @author  Rémi Pincent - INRIA
 * @date    07/08/2018
 *
 * @brief Control a moving object either :
 *  -connecting to a Sensortag Server over BLE, it gets sensortag orientation and convert it
 *   to a (speed,steer). In this case, user can remotely control a moving object
 *  - getting ADC values from a joystick and convert it
 *   to a (speed,steer)
 * 
 * ((int16_t)steer,(int16_t)speed) sent over UART2,
 * it can be used to do a remote control for some hoverboard based moving objects :
 * https://github.com/NiklasFauth/hoverboard-firmware-hack
 *
 * Project : steer_speed_controller
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * code example got from : https://github.com/hitokuno/ESP32-SensorTag.git
 * Insert github reference
 *
 * LICENSE :
 * steer_speed_controller (c) by Rémi Pincent
 * steer_speed_controller is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "BLEDevice.h"
#include <limits.h>
#include <NeoPixelAnimator.h>
#include <NeoPixelBus.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/
#define MODE_ADC_PIN   (32U) 
#define STEER_ADC_PIN  (34U) 
#define SPEED_ADC_PIN  (35U) 
#define STRIP_DATA_PIN (23U)

#define NB_SPEED_LED   (2U)
#define NB_STEER_LED   (2U)
#define NOT_USED_LED   (NB_SPEED_LED)
#define PWR_LED        (NOT_USED_LED + 1)
#define MODE_LED       (PWR_LED + 1)
#define CTRL_TYPE_LED  (MODE_LED + 1)
#define NB_NEOPIX      (11U)
 
/** Address of Sensortag server */ 
#define ST_ADD "b0:91:22:f6:71:05"
 
#define CMD_PERIOD (50U)
#define ADC_RESOLUTION (12U)
#define ADC_MAX (2 << ADC_RESOLUTION-1)
#define ADC_MID_POINT (ADC_MAX >> 1)
 
#define NB_CAL_SAMPLES (100U)

#define MODE_LPF_ALPHA (0.3)

/** below this angle on roll and pirch: steer, speed will be set to 0 */
#define DEAD_ANGLE (10.0)

/** below abs(adc - ADC_MID_POINT) < DEAD_ADC  -  values, ADC will be set to 0*/
#define DEAD_ADC (150U)

/** above this angle, steer, speed are at their max values */
#define MAX_ANGLE (45.0)
/** below this angle, steer, speed are at their max values */
#define MIN_ANGLE (-MAX_ANGLE)

#define MAX_SPEED  (1000)
#define MIN_SPEED (-MAX_SPEED)

#define CAL_DONE_CMD  (SHRT_MAX - 1)

#define MAX_STEER  (1000)
#define MIN_STEER (-MAX_STEER)

#define MIN_ALPHA (0.03)
#define MIN_ALPHA (0.1)

static const BLEUUID        deviceUUID("0000aa80-0000-1000-8000-00805f9b34fb");

/** movement service UUIDs */
static BLEUUID        movServiceUUID("f000aa80-0451-4000-b000-000000000000");
static BLEUUID           movDataUUID("f000aa81-0451-4000-b000-000000000000");
static BLEUUID  movConfigurationUUID("f000aa82-0451-4000-b000-000000000000");
static BLEUUID         movPeriodUUID("f000aa83-0451-4000-b000-000000000000");

/**************************************************************************
 * Type Definitions
 **************************************************************************/
typedef struct
{
  float roll;
  float pitch;
  float yaw;
}Angles;

typedef enum
{
  IDLE,
  DISCOVERING,
  DISCOVERED,
  CONNECTED_CALIBRATING,
  CONNECTED
}State;

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Static Variables
 **************************************************************************/
static BLEAddress serverAddress(ST_ADD);
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pConfigurationCharacteristic;
static BLERemoteCharacteristic* pPeriodCharacteristic;
static BLERemoteCharacteristic* pBatteryCharacteristic;
static BLEScan* pBLEScan;
static BLEClient*  pClient;

static int calIndex;
static Angles calAngles;
static Angles angles;
static State state = IDLE;
static uint16_t adcSteer;
static uint16_t adcSpeed;
static int16_t speedCmd;
static int16_t steerCmd;
static float steerCoef = 1.0;
static float speedCoef = 1.0;
static uint8_t mode = 0;
static float alphaSpeedLPF = 0.03;
static float alphaSteerLPF = 0.03;

static NeoPixelBus<NeoGrbFeature, NeoEsp32BitBang800KbpsMethod> strip(NB_NEOPIX, STRIP_DATA_PIN);

/**************************************************************************
 * Macros
 **************************************************************************/
#define ALPHA_LPF_FROM_FC(samplingPeriod, fc) ((2*PI*samplingPeriod*fc)/(1 + 2*PI*samplingPeriod*fc))

/**************************************************************************
 * Local Functions Declarations
 **************************************************************************/
static void setLedSTControl(void);
static void setLedADCControl(void);
static void setLedPower(void);
static void setLedSpeed(int16_t speed);
static void setLedSteer(int16_t steer);
static void setLedMode(void);
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
static bool connectToServer(BLEAddress pAddress);
static void scanBle(void);
static bool getAngles(void);
static void sendCmd(int16_t steer, int16_t speed);
static void sendCalDoneCmd(void);
static void printAdc(void);
static int16_t rollToSpeed(void);
static int16_t pitchToSteer(void);
static int16_t adcToSteer(void);
static int16_t adcToSpeed(void);
static void sendSteerSpeedCmd(int16_t steer, int16_t speed);
static void analogCtrlTask( void * parameter );
static void stCtrlTask( void * parameter );
static void updateMode(void);
static float lpf(float alpha, float newVal, float oldVal);
static float fmap(float x, float in_min, float in_max, float out_min, float out_max);
/**************************************************************************
 * Global Functions Definitions
 **************************************************************************/

 /**************************************************************************
 * Local Functions Definitions
 **************************************************************************/
 static void setLedSTControl(void)
{
  strip.SetPixelColor(CTRL_TYPE_LED, RgbColor(0x00, 0xFF, 0x00));
  strip.Show();  
}

static void setLedADCControl(void)
{
  strip.SetPixelColor(CTRL_TYPE_LED, RgbColor(0x00, 0x00, 0xFF));
  strip.Show();  
}

static void setLedPower(void)
{
  strip.SetPixelColor(PWR_LED, RgbColor(0x00, 0xFF, 0x00));
  strip.Show();  
}

static void setLedSpeed(int16_t speed)
{
  for(uint8_t i = NB_NEOPIX - 1; i >= NB_NEOPIX - NB_SPEED_LED; i--){    
     strip.SetPixelColor(i, HslColor(fmap(fabs(speed), 0, MAX_SPEED*speedCoef - 50, 1, 0)*120/360, 1.0f, 0.5)); 
  }
  strip.Show();  
}

static void setLedSteer(int16_t steer)
{
  for(uint8_t i = 0; i < NB_STEER_LED; i++){
     strip.SetPixelColor(i, HslColor(fmap(fabs(steer), 0, MAX_STEER*steerCoef - 50, 1, 0)*120/360, 1.0f, 0.5)); 
  }
  strip.Show();  
}

static void setLedMode(void)
{
  strip.SetPixelColor(MODE_LED, HslColor(fmap(mode, 0, 0x80, 0, 1)*0.33, 1.0f, 0.5)); 
  strip.Show();  
}

class MyBLEClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *pClient)
  {
    Serial.println("onConnect");  
  }
  
  void onDisconnect(BLEClient *pClient)
  {
    Serial.println("BLE disconnection");

    /** Force disconnonection, workaround to connect multiple times */
    pClient->disconnect();
    sendSteerSpeedCmd(0, 0);
    setLedADCControl();
    state = IDLE;
  }
};

/**
 * Scan for BLE servers and find the strongest RSSI one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    BLEAddress add(advertisedDevice.getAddress());
    if (advertisedDevice.haveServiceUUID() 
      && advertisedDevice.getServiceUUID().equals(deviceUUID)
      && strcmp(ST_ADD, add.toString().c_str()) == 0) {
      int thisRssi = advertisedDevice.getRSSI();
      Serial.print("Found our device!  address: ");
      Serial.print(add.toString().c_str());
      //advertisedDevice.getScan()->stop();
      Serial.print("  RSSI: ");
      Serial.println(thisRssi);
      pBLEScan->stop();
      state = DISCOVERED;
      setLedSTControl();
    }
  }
};

 
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
}

static bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());

    if(!pClient){
      pClient  = BLEDevice::createClient();
      pClient->setClientCallbacks(new MyBLEClientCallbacks());
      Serial.println(" - Created client");
    }

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(movServiceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(movServiceUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our service");
    Serial.println(pRemoteService->toString().c_str());

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(movDataUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(movDataUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our characteristic");

    //  Enable Accelerometer
    pConfigurationCharacteristic = pRemoteService->getCharacteristic(movConfigurationUUID);
    if (pConfigurationCharacteristic == nullptr) {
      Serial.print("Failed to find our Configuration: ");
      Serial.println(movConfigurationUUID.toString().c_str());
      return false;
    }
    // Gyroscope must be enabled to get Accelerometer due to unknown reason..
    uint8_t accelerometerEnabled[2] = {B00111111, B00000000};
    pConfigurationCharacteristic->writeValue( accelerometerEnabled, 2);
    // Read the value of the characteristic.
    uint16_t valueInt16 = pConfigurationCharacteristic->readUInt16();
    Serial.print(" - Enabled accelerometer: ");
    Serial.println(valueInt16, BIN);

    // Set update period.
    //Resolution 10 ms. Range 100 ms (0x0A) to 2.55 sec (0xFF). Default 1 second (0x64).
    uint8_t accelerometerPeriod = 10;
    pPeriodCharacteristic = pRemoteService->getCharacteristic(movPeriodUUID);
    if (pPeriodCharacteristic == nullptr) {
      Serial.print("Failed to find our period: ");
      Serial.println(movPeriodUUID.toString().c_str());
      return false;
    }
    pPeriodCharacteristic->writeValue( accelerometerPeriod, 1);
    Serial.print(" - Accelerometer period: ");
    uint8_t valueInt8 = pPeriodCharacteristic->readUInt8();
    Serial.print(valueInt8);
    Serial.println("*10ms");
    
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    return true;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(19200);
  strip.Begin();
  setLedADCControl();
  setLedPower();
  Serial.println("Starting st_moving_object_ctrl");
  mode = lpf(MODE_LPF_ALPHA, analogRead(MODE_ADC_PIN) >> 5, mode);
  updateMode();
    
  xTaskCreate(
                    analogCtrlTask,          
                    "AnalogCtrlTask",      
                    10000,            
                    NULL,           
                    1,               
                    NULL);    

  xTaskCreate(
                    stCtrlTask,          
                    "stCtrlTask",      
                    10000,            
                    NULL,           
                    1,               
                    NULL);  
                             
}

static void scanBle(void) {
  Serial.println("Scanning BLE devices...");
  state = DISCOVERING;
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
  Serial.println("end of scan");
}

static bool getAngles(void)
{
  std::string value = pRemoteCharacteristic->readValue();
  String strings = value.c_str();
  
  long *ptr = (long*)&angles.roll;
  *ptr = (int)strings.charAt( 0) | (int)strings.charAt(1) << 8 | (int)strings.charAt(2) << 16 | (int)strings.charAt(3) << 24;
  ptr = (long*)&angles.pitch;
  *ptr = strings.charAt( 4) | strings.charAt(5) << 8 | strings.charAt(6) << 16 | strings.charAt(7) << 24;

  if(angles.roll == 0 && angles.pitch == 0){
    Serial.println("IMU not ready");
    return false; 
  }
  else{
    return true;
  }
}

static void sendCmd(int16_t steer, int16_t speed)
{
  setLedSpeed(speed);
  setLedSteer(steer);
  
  Serial2.write((uint8_t) steer);
  Serial2.write((uint8_t) (steer >> 8));

  Serial2.write((uint8_t) speed);
  Serial2.write((uint8_t) (speed >> 8));
}

static void sendCalDoneCmd(void){
  sendCmd(CAL_DONE_CMD, CAL_DONE_CMD);
}

static void printAngles(void) {
  Serial.print("roll = ");
  Serial.print(angles.roll);
  Serial.print(" pitch = ");
  Serial.print("   ");
  Serial.println(angles.pitch);
}

static void printAdc(void)
{
  Serial.print("ADC_STEER = ");
  Serial.print(adcSteer);
  Serial.print(" ADC_SPEED = ");
  Serial.println(adcSpeed);
}

static int16_t rollToSpeed(void)
{
  float speed = 0;
  float roll = angles.roll - calAngles.roll;
  
  if(fabs(roll) < DEAD_ANGLE ){
     speed = 0;
  }
  else if(roll > MAX_ANGLE){
    speed = MAX_SPEED;
  }
  else if(roll < MIN_ANGLE){
    speed = MIN_SPEED;
  }
  else{
    /** convert angle to speed 
     *  [MIN_ANGLE; -DEAD_ANGLE] -> [MIN_SPEED; 0]
     *  [DEAD_ANGLE; MAX_ANGLE]   -> [0; MAX_SPEED]
     */
     if(roll >= 0){
       speed = fmap(roll, DEAD_ANGLE, MAX_ANGLE, 0, MAX_SPEED);
     }
     else{
       speed = fmap(roll, MIN_ANGLE, -DEAD_ANGLE, MIN_SPEED, 0);
     }
  }
  return speed;
}

static int16_t pitchToSteer(void)
{
  float steer = 0;
  float pitch = angles.pitch - calAngles.pitch;
  
  if(fabs(pitch) < DEAD_ANGLE ){
     steer = 0;
  }
  else if(pitch > MAX_ANGLE){
    steer = MAX_STEER;
  }
  else if(pitch < MIN_ANGLE){
    steer = MIN_STEER;
  }
  else{
    /** convert angle to speed 
     *  [MIN_ANGLE; -DEAD_ANGLE] -> [MIN_STEER; 0]
     *  [DEAD_ANGLE; MAX_ANGLE]   -> [0; MAX_STEER]
     */
     if(pitch >= 0){
      steer = fmap(pitch, DEAD_ANGLE, MAX_ANGLE, 0, MAX_STEER);
     }
     else{
      steer = fmap(pitch, MIN_ANGLE, -DEAD_ANGLE, MIN_STEER, 0);
     }
  }
  return steer;
}

static int16_t adcToSteer(void)
{
   if(abs(adcSteer - ADC_MID_POINT) < DEAD_ADC){
    return 0;
   }
   else{
    if(adcSteer - ADC_MID_POINT > 0){
     /*
     *  [(ADC_MAX >> 2) + DEAD_ADC; ADC_MAX] -> [0; MAX_STEER]
     */      
      return map(adcSteer, ADC_MID_POINT + DEAD_ADC, ADC_MAX, 0, MAX_STEER);
    }
    else{
     /*
     *  [0 ; (ADC_MAX >> 2) - DEAD_ADC] -> [MIN_STEER; 0]
     */      
      return map(adcSteer, 0, ADC_MID_POINT - DEAD_ADC, MIN_STEER, 0);
    }
   } 
}

static int16_t adcToSpeed(void)
{
   if(abs(adcSpeed - ADC_MID_POINT) < DEAD_ADC){
    return 0;
   }
   else{
    if(adcSpeed - ADC_MID_POINT > 0){
     /*
     *  [(ADC_MAX >> 2) + DEAD_ADC; ADC_MAX] -> [0; MAX_SPEED]
     */      
      return map(adcSpeed, ADC_MID_POINT + DEAD_ADC, ADC_MAX, 0, MAX_SPEED);
    }
    else{
     /*
     *  [0 ; (ADC_MAX >> 2) - DEAD_ADC] -> [MIN_SPEED; 0]
     */      
      return map(adcSpeed, 0, ADC_MID_POINT - DEAD_ADC, MIN_SPEED, 0);
    }
   } 
}

static void sendSteerSpeedCmd(int16_t steer, int16_t speed)
{  
  int16_t newSteerCmd = lpf(alphaSteerLPF, steer*steerCoef, steerCmd);
  int16_t newSpeedCmd = lpf(alphaSpeedLPF, speed*steerCoef, speedCmd); 

  /** Do not stop sending some commands, if no command received by hoverboard 
   *  it sets its speed to 0
   */
  if(newSteerCmd != steerCmd || newSpeedCmd != speedCmd){
    steerCmd = newSteerCmd;
    speedCmd = newSpeedCmd;

    if(state == DISCOVERING
      || state == IDLE){
      Serial.print("ADC ");
    }
    else{
      Serial.print("ST ");
    }
    Serial.print("Control > After filter speedCmd = ");
    Serial.print(speedCmd);
    Serial.print(" steerCmd = ");
    Serial.println(steerCmd);
  }
  sendCmd(steerCmd, speedCmd);
}

static void analogCtrlTask( void * parameter )
{
  while(1){
    if(state == IDLE || state == DISCOVERING){
      adcSteer = analogRead(STEER_ADC_PIN);
      adcSpeed = analogRead(SPEED_ADC_PIN);
      //printAdc();
      sendSteerSpeedCmd(adcToSteer(), adcToSpeed());
    }
    delay(CMD_PERIOD);
  }
}

static void stCtrlTask( void * parameter )
{
  while(1){
    if(state == IDLE){
      calAngles.roll = 0;
      calAngles.pitch = 0;
      calAngles.yaw = 0;
      
      angles.roll = 0;
      angles.pitch = 0;
      angles.yaw = 0;
      calIndex = 0;
      
      scanBle();
    }
    else if(state == DISCOVERING)
    {
      /** try a new scan */
      scanBle();
    }
    else if(state == DISCOVERED){
      Serial.println("Connecting to the BLE Server...");
      if (connectToServer(serverAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        Serial.println("Starting calibration...");
        state = CONNECTED_CALIBRATING;
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
        state = IDLE;
        setLedADCControl();
      }
    }
    else if(state == CONNECTED_CALIBRATING){
      if(getAngles()){
        if(calIndex == NB_CAL_SAMPLES){
          calAngles.roll = calAngles.roll/NB_CAL_SAMPLES;
          calAngles.pitch = calAngles.pitch/NB_CAL_SAMPLES;
          calAngles.yaw = calAngles.yaw/NB_CAL_SAMPLES;
          Serial.print("calibrated "); 
          Serial.print("roll = "); 
          Serial.print(calAngles.roll); 
          Serial.print(" pitch = "); 
          Serial.print(calAngles.pitch); 
          Serial.print(" yaw = "); 
          Serial.println(calAngles.yaw); 
          sendCalDoneCmd();
          state = CONNECTED;
        }
        else{
          calAngles.roll += angles.roll;
          calAngles.pitch += angles.pitch;
          calIndex++;
        }
      }
    }
    else if(state == CONNECTED){ 
      if(getAngles()){
        //printAngles();
        sendSteerSpeedCmd(pitchToSteer(), rollToSpeed());
      }
    }
    delay(CMD_PERIOD);
  }
}

static void updateMode(void)
{
  steerCoef = fmap(mode, 0, 0x80, 1.0, 0.2);
  speedCoef = fmap(mode, 0, 0x80, 0.4, 0.1);
  alphaSpeedLPF = fmap(mode, 0, 0x80,ALPHA_LPF_FROM_FC(CMD_PERIOD/1000.0, 1.0/8), ALPHA_LPF_FROM_FC(CMD_PERIOD/1000.0, 1.0/20));
  alphaSteerLPF = fmap(mode, 0, 0x80, ALPHA_LPF_FROM_FC(CMD_PERIOD/1000.0, 1.0/8), ALPHA_LPF_FROM_FC(CMD_PERIOD/1000.0, 1.0/15));
  setLedMode();
  Serial.print("Mode update > steerCoef = ");
  Serial.print(steerCoef, 3);
  Serial.print(" speedCoef = ");
  Serial.print(speedCoef, 3);
  Serial.print(" alphaSteerLPF = ");
  Serial.println(alphaSteerLPF, 4);  
  Serial.print(" alphaSpeedLPF = ");
  Serial.println(alphaSpeedLPF, 4);
}

static float lpf(float alpha, float newVal, float oldVal)
{
  return alpha*newVal + (1 - alpha)*oldVal;
}

static float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  uint16_t newMode = lpf(MODE_LPF_ALPHA, analogRead(MODE_ADC_PIN) >> 5, mode);
  if(newMode != mode){
    mode = newMode;
    updateMode();
  }
  
  delay(100);

} // End of loop
