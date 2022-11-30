//by Graham Clifford

//Credit to Joop Brokking for my MPU6050 code. www.brokking.net/

//initializing libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include "RF24.h"
#include "ArduPID.h"

//PID globals
ArduPID pid_y;
ArduPID pid_p;
ArduPID pid_r;
ArduPID pid_az;
double p = 1;
double i = 0;
double d = 0;

//receiver globals
RF24 radio(A0, 10); // radio(CE_pin,CSN_pin)
uint8_t address[][6] = {"1Node", "2Node"}; // Let these addresses be used for the pair
char data[33] = {}; //32 bytes is max transmission size, +1 bite for NULL byte at end of string.
char ack[33] = {};

//motor globals
const uint8_t motor[] = {3, 5, 6, 9};
uint8_t ground[] = {A1, 4, 7, 8};
double thrust_mod = 0;
uint16_t thrust[] = {0, 0, 0, 0};

//gyro globals
int gyro_x, gyro_y, gyro_z;
double acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
double angle_pitch_output, angle_roll_output;
double pitch_set, roll_set, acc_z_set = 0;


// ================================================================
// ===                   INTERRUPT DETECTION                    ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// join I2C bus (I2Cdev library doesn't do this automatically

// ================================================================
// ===                  Radio Initialization                 ===
// ================================================================
void radioInit() {
  while (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }
  Serial.println(F("radio initialized"));
  radio.enableAckPayload(); //Allow ack(nowledgement) payloads. This will let us send data back to transmitter without manually changing the radio modes on both Arduinos.
  radio.enableDynamicPayloads(); //Need for sending ack payloads on pipes other than 0 and 1. This enables it on all pipes.
  radio.setRetries(5, 15);
  radio.openWritingPipe(address[1]);     // always uses pipe 0
  radio.openReadingPipe(1, address[0]); // using pipe 1

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the number of bytes we need to transmit a float

  // additional setup specific to the node's role
  radio.startListening(); // put radio in RX mode
}

// ================================================================
// ===                Motor Driver Initialization               ===
// ================================================================

void motorDriverInit() {
  Serial.println("Initializing motor pins...");
  for (int i = 0; i < 4; i++) {
    pinMode(motor[i], OUTPUT);
    analogWrite(motor[i], 0);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(ground[i], OUTPUT);
    digitalWrite(ground[i], LOW);
  }
}

// ================================================================
// ===        Gyroscope and Accelerometer Initialization        ===
// ================================================================
void gyroInit() {
  Wire.begin();                                                        //Start I2C as master

  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
    if (cal_int % 1000 == 0) Serial.print(".");                        //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

  loop_timer = micros();
}

void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void transmitString(String s) {
  uint8_t len = s.length() + 1;
  char data[len];
  s.toCharArray(data, len);
  if (radio.write(&data, sizeof(data))) {
  }
}

void pidInit(int j, ArduPID pid, int k) {
  if (k == 0) {
    pid.begin(&_ypr[j], &thrust_mod, &ypr_setpoint[j], p, i, d);
    pid.setOutputLimits(0, 255);
    pid.setBias(255.0 / 2.0);
    pid.setWindUpLimits(-5, 5);
    pid.setSampleTime(10);
    pid.start();
  }
  else {
    pid.begin(&az, &thrust_mod, &az_setpoint, p, i, d);
    pid.setOutputLimits(0, 255);
    pid.setWindUpLimits(-10, 10);
    pid.setBias(255.0 / 2.0);
    pid.setSampleTime(20);
    pid.start();
  }
}

void setup() {
  Serial.begin(19200);
  radioInit();
  motorDriverInit();
  Serial.println("Waiting for transmission from controller to begin MPU6050 intitialization");  //wait to receive a tranmission from the controller before initializing the gyroscope accelerometer
  gyroInit();

  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration                                             //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  pitch_set =  angle_pitch_output;                                              //Accelerometer calibration value for pitch
  roll_set = angle_roll_output;

  angle_roll_acc -= 0;
  angle_pitch_acc -= 0;

  pid_p.begin(&angle_pitch_output, &thrust_mod, &pitch_set, p, i, d);
  pid_r.begin(&angle_roll_output, &thrust_mod, &roll_set, p, i, d);
  pid_az.begin(&acc_z, &thrust_mod, &acc_z_set, p, i, d);

  /*pidInit(0, pid_y, 0);
    pidInit(1, pid_p, 0);
    pidInit(2, pid_r, 0);
    pidInit(2, pid_az, 1);*/

  for (int i = 0; i < 4; i ++) {
    thrust[i] += 50;
  }
}

void loop() {
  // ================================================================
  // ===               Gyroscope/Accelerometer Loop               ===
  // ================================================================
  read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  Serial.print("pitch: "); Serial.print(angle_pitch_output); Serial.print("\troll: "); Serial.println(angle_roll_output);

  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer

  if (radio.available()) {
    int bytes = radio.getPayloadSize();
    radio.read(&data, bytes);
    radio.writeAckPayload(0, &ack, bytes);
  }

  // ================================================================
  // ===                        Flight Loop                       ===
  // ================================================================
  //Leveling Logic
  /*pid_y.compute();
    thrust[0] -= thrust_mod;
    thrust[1] += thrust_mod;
    thrust[2] += thrust_mod;
    thrust[3] -= thrust_mod;*/

  pid_p.compute();

  thrust[0] -= thrust_mod;
  thrust[1] -= thrust_mod;
  thrust[2] += thrust_mod;
  thrust[3] += thrust_mod;

  pid_r.compute();

  thrust[0] -= thrust_mod;
  thrust[1] += thrust_mod;
  thrust[2] -= thrust_mod;
  thrust[3] += thrust_mod;

  pid_az.compute();
  thrust[0] += thrust_mod;
  thrust[1] += thrust_mod;
  thrust[2] += thrust_mod;
  thrust[3] += thrust_mod;

  for (int i = 0; i < 4; i++) {
    if (thrust[i] > 255) {
      thrust[i] = 200;
    }
  }

  //Serial.print("thrust mod: "); Serial.println(thrust_mod);
  for (int i = 0; i < 4; i++) {
    analogWrite(motor[i], thrust[i]);
    //Serial.print("thrust "); Serial.print(i); Serial.print(": "); Serial.println(thrust[0]);
  }

  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}
