#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// switch control setting
const int SWITCH_ON_STATE = LOW; 
const int SWITCH_OFF_STATE = HIGH; 


const int BASE_SPEED = 150;
const int SLOW_SPEED = 100;
const int PARKING_SPEED = 80;

const float WALL_CENTER_KP = 1.5;           
const int CENTER_STEERING_ANGLE = 90;       
const int OBSTACLE_STEERING_FORCE = 35;     
const int MAX_STEER_LEFT = 45;
const int MAX_STEER_RIGHT = 135;

const int MIN_OBSTACLE_AREA = 500;          
const int SAFE_DISTANCE_CM = 15;            
const unsigned long MAX_RACE_TIME_SECONDS = 180; 

const int CORNERS_PER_LAP = 4;
const int TARGET_CORNER_COUNT = 12;         
const unsigned long MIN_TIME_BEFORE_PARKING_SEARCH = 30000; 
const int PARKING_CENTER_TOLERANCE_CM = 5;  
const unsigned long PARKING_STABLE_TIME_SECONDS = 3; 

// oled settings
const uint8_t OLED_ADDR = 0x3C;
const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;

// pin map
#define I2C_SDA 8
#define I2C_SCL 9

#define RX_PIN 37
#define TX_PIN 36
#define SERVO_PIN 12


#define MOTOR_PWM 40
#define MOTOR_DIR1 16
#define MOTOR_DIR2 33
#define DRV8833_STBY_PIN 11 


#define START_BUTTON_PIN 1          // start switch
#define MODE_BUTTON_PIN 2           // mode switch
#define DIRECTION_BUTTON_PIN 4      // direction switch
#define GYRO_RESET_BUTTON_PIN 6     // gyro reset
#define SERVO_CENTER_BUTTON_PIN 10  // servo center
#define MOTOR_TEST_BUTTON_PIN 13    // motor test
#define UART_CHECK_BUTTON_PIN 14    // uart check
#define SENSOR_INFO_BUTTON_PIN 21   // sensor screen

// sensor pins
#define LEFT_DIST_PIN 7   
#define FRONT_DIST_PIN 5
#define RIGHT_DIST_PIN 3  
#define HALL_EFFECT_PIN 15 // hall effect input

// ============================================================================
// 4. global objects and variables
// ============================================================================
enum RaceState { WAITING_START, RUNNING, PARKING_SEARCH, PARKING, FINISHED, EMERGENCY_STOP };
enum RaceMode { OPEN_RACE, OBSTACLE_RACE };
enum RaceDirection { CLOCKWISE, COUNTER_CLOCKWISE }; 

RaceState currentState = WAITING_START;
RaceMode currentMode = OPEN_RACE;
RaceDirection currentDirection = CLOCKWISE; 

HardwareSerial PiSerial(1); 
Servo steeringServo;
Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

float left_distance = 0;
float right_distance = 0;
float front_distance = 0;

String pi_buffer = ""; 
String pi_steering_command = "go_straight";
String pi_speed_command = "normal_speed";
String pi_obstacle_color = "none";
String pi_obstacle_position = "none";
int pi_obstacle_area = 0;
int pi_obstacle_center_x = 0;

unsigned long race_start_time = 0;
unsigned long stable_straight_start_time = 0;
unsigned long last_pi_packet_time = 0; 
unsigned long last_display_time = 0; 
int corner_count = 0;
bool is_currently_in_corner = false;
float yaw_angle = 0;
unsigned long last_imu_time = 0;

int current_target_steering = CENTER_STEERING_ANGLE;
int current_target_speed = 0;

// ============================================================================
// 5. functions
// ============================================================================

void read_distance_sensors() {
    left_distance = analogRead(LEFT_DIST_PIN) * 0.1; 
    front_distance = analogRead(FRONT_DIST_PIN) * 0.1;
    right_distance = analogRead(RIGHT_DIST_PIN) * 0.1;
    
    if(left_distance <= 0) left_distance = 150; 
    if(right_distance <= 0) right_distance = 150;
}

void read_pi_camera_data() {
    int bytes_read = 0;
    while (PiSerial.available() && bytes_read < 30) {
        char c = PiSerial.read();
        bytes_read++;
        
        if (c == '\n') {
            pi_buffer.trim();
            int firstComma = pi_buffer.indexOf(',');
            int secComma = pi_buffer.indexOf(',', firstComma + 1);
            int thirdComma = pi_buffer.indexOf(',', secComma + 1);
            int fourthComma = pi_buffer.indexOf(',', thirdComma + 1);
            int fifthComma = pi_buffer.indexOf(',', fourthComma + 1);
            
            if (firstComma > 0 && secComma > firstComma && thirdComma > secComma && fourthComma > thirdComma && fifthComma > fourthComma) {
                pi_steering_command = pi_buffer.substring(0, firstComma);
                pi_speed_command = pi_buffer.substring(firstComma + 1, secComma);
                pi_obstacle_color = pi_buffer.substring(secComma + 1, thirdComma);
                pi_obstacle_position = pi_buffer.substring(thirdComma + 1, fourthComma);
                pi_obstacle_area = pi_buffer.substring(fourthComma + 1, fifthComma).toInt();
                pi_obstacle_center_x = pi_buffer.substring(fifthComma + 1).toInt();
                
                last_pi_packet_time = millis(); 
            }
            pi_buffer = ""; 
        } 
        else if (c != '\r') {
            pi_buffer += c; 
        }
        if (pi_buffer.length() > 80) pi_buffer = "";
    }
}

void update_imu_heading() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long current_time = millis();
    float dt = (current_time - last_imu_time) / 1000.0;
    yaw_angle += (g.gyro.z * 57.2958) * dt; 
    last_imu_time = current_time;
}

int calculate_wall_centering_steering() {
    float distance_error = left_distance - right_distance;
    int steering_correction = (int)(WALL_CENTER_KP * distance_error);
    return CENTER_STEERING_ANGLE + steering_correction;
}

int calculate_obstacle_avoidance(int current_base_steering) {
    if (pi_obstacle_area < MIN_OBSTACLE_AREA) return current_base_steering;
    if (pi_obstacle_color == "red") return CENTER_STEERING_ANGLE - OBSTACLE_STEERING_FORCE;
    if (pi_obstacle_color == "green") return CENTER_STEERING_ANGLE + OBSTACLE_STEERING_FORCE;
    return current_base_steering;
}

int combine_steering_decisions() {
    int wall_steer = calculate_wall_centering_steering();
    if (currentMode == OBSTACLE_RACE) return calculate_obstacle_avoidance(wall_steer);
    return wall_steer;
}

void estimate_corner_passed() {
    bool corner_condition = false;

    if (currentDirection == COUNTER_CLOCKWISE) {
        if (yaw_angle > 75) corner_condition = true;
    } else {
        if (yaw_angle < -75) corner_condition = true;
    }

    if (corner_condition) {
        if (!is_currently_in_corner) {
            corner_count++;
            is_currently_in_corner = true;
            yaw_angle = 0; 
        }
    } else {
        if (currentDirection == COUNTER_CLOCKWISE && yaw_angle <= 20) is_currently_in_corner = false;
        if (currentDirection == CLOCKWISE && yaw_angle >= -20) is_currently_in_corner = false;
    }
}

bool detect_starting_section_again() {
    float width_difference = abs(left_distance - right_distance);
    if (width_difference < 30 && front_distance > 100) { 
        if (stable_straight_start_time == 0) stable_straight_start_time = millis();
        else if ((millis() - stable_straight_start_time) > (PARKING_STABLE_TIME_SECONDS * 1000)) return true; 
    } else {
        stable_straight_start_time = 0; 
    }
    return false;
}

// ============================================================================
// 6. SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    PiSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    // set all buttons as pullups
    pinMode(RX_PIN, INPUT_PULLUP);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DIRECTION_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(GYRO_RESET_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SERVO_CENTER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MOTOR_TEST_BUTTON_PIN, INPUT_PULLUP);
    pinMode(UART_CHECK_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SENSOR_INFO_BUTTON_PIN, INPUT_PULLUP);
    
    pinMode(HALL_EFFECT_PIN, INPUT_PULLUP);
    
    pinMode(MOTOR_DIR1, OUTPUT);
    pinMode(MOTOR_DIR2, OUTPUT);
    pinMode(MOTOR_PWM, OUTPUT);
    pinMode(DRV8833_STBY_PIN, OUTPUT); 
    
    digitalWrite(DRV8833_STBY_PIN, LOW);
    digitalWrite(MOTOR_DIR1, LOW);
    digitalWrite(MOTOR_DIR2, LOW);
    analogWrite(MOTOR_PWM, 0);

    steeringServo.attach(SERVO_PIN);
    steeringServo.write(CENTER_STEERING_ANGLE);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); 
    mpu.begin();
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

// ============================================================================
// 7. main loop
// ============================================================================

void loop() {
    unsigned long current_time = millis();
    
    read_distance_sensors();
    read_pi_camera_data(); 
    update_imu_heading();

    // gyro reset button
    if (digitalRead(GYRO_RESET_BUTTON_PIN) == SWITCH_ON_STATE) {
        yaw_angle = 0;
    }

    // main switch check
    bool isStartSwitchOn = (digitalRead(START_BUTTON_PIN) == SWITCH_ON_STATE);

    if (!isStartSwitchOn && currentState != WAITING_START) {
        currentState = WAITING_START;
        current_target_speed = 0;
        digitalWrite(DRV8833_STBY_PIN, LOW); 
    }

    // main state machine
    switch (currentState) {
        case WAITING_START:
            current_target_speed = 0;
            current_target_steering = CENTER_STEERING_ANGLE;
            digitalWrite(DRV8833_STBY_PIN, LOW); 
            
            // direction switch
            currentDirection = (digitalRead(DIRECTION_BUTTON_PIN) == SWITCH_ON_STATE) ? COUNTER_CLOCKWISE : CLOCKWISE;
            
            // mode switch
            currentMode = (digitalRead(MODE_BUTTON_PIN) == SWITCH_ON_STATE) ? OBSTACLE_RACE : OPEN_RACE;
            
            // start switch
            if (isStartSwitchOn) {
                race_start_time = current_time;
                last_imu_time = current_time;
                corner_count = 0;
                yaw_angle = 0;
                digitalWrite(DRV8833_STBY_PIN, HIGH); 
                currentState = RUNNING;
            }
            break;

        case RUNNING:
            if (front_distance < SAFE_DISTANCE_CM) { currentState = EMERGENCY_STOP; break; }
            if ((current_time - race_start_time) > (MAX_RACE_TIME_SECONDS * 1000)) { currentState = FINISHED; break; }

            estimate_corner_passed(); 
            if (corner_count >= TARGET_CORNER_COUNT) {
                if ((current_time - race_start_time) > MIN_TIME_BEFORE_PARKING_SEARCH) currentState = PARKING_SEARCH;
            }

            current_target_steering = combine_steering_decisions();
            current_target_speed = (pi_speed_command == "slow_down" && currentMode == OBSTACLE_RACE) ? SLOW_SPEED : BASE_SPEED;
            break;

        case PARKING_SEARCH:
            current_target_steering = combine_steering_decisions();
            current_target_speed = SLOW_SPEED; 
            if (detect_starting_section_again()) currentState = PARKING;
            break;

        case PARKING:
            current_target_steering = calculate_wall_centering_steering();
            current_target_speed = PARKING_SPEED;
            if (abs(left_distance - right_distance) <= PARKING_CENTER_TOLERANCE_CM && front_distance < 40) currentState = FINISHED;
            break;

        case FINISHED:
            current_target_speed = 0;
            current_target_steering = CENTER_STEERING_ANGLE;
            digitalWrite(DRV8833_STBY_PIN, LOW); 
            break;

        case EMERGENCY_STOP:
            current_target_speed = 0;
            if (front_distance > (SAFE_DISTANCE_CM * 2)) currentState = RUNNING; 
            break;
    }

    // servo center button
    if (digitalRead(SERVO_CENTER_BUTTON_PIN) == SWITCH_ON_STATE) {
        steeringServo.write(90);
    } else {
        steeringServo.write(constrain(current_target_steering, MAX_STEER_LEFT, MAX_STEER_RIGHT));
    }
    
    // manual motor test button
    if (digitalRead(MOTOR_TEST_BUTTON_PIN) == SWITCH_ON_STATE) {
        digitalWrite(DRV8833_STBY_PIN, HIGH);
        // forward test
        digitalWrite(MOTOR_DIR1, HIGH); digitalWrite(MOTOR_DIR2, LOW); analogWrite(MOTOR_PWM, BASE_SPEED);
        delay(2000);
        // reverse test
        digitalWrite(MOTOR_DIR1, LOW); digitalWrite(MOTOR_DIR2, HIGH); analogWrite(MOTOR_PWM, BASE_SPEED);
        delay(2000);
        // stop motor
        digitalWrite(MOTOR_DIR1, LOW); digitalWrite(MOTOR_DIR2, LOW); analogWrite(MOTOR_PWM, 0);
    } 
    // normal motor output
    else {
        if (current_target_speed == 0) {
            digitalWrite(MOTOR_DIR1, LOW);
            digitalWrite(MOTOR_DIR2, LOW);
            analogWrite(MOTOR_PWM, 0);
        } else {
            digitalWrite(MOTOR_DIR1, HIGH);
            digitalWrite(MOTOR_DIR2, LOW);
            analogWrite(MOTOR_PWM, current_target_speed);
        }
    }

    // oled refresh block
    if (current_time - last_display_time >= 150) {
        display.clearDisplay();
        display.setCursor(0, 0);
        
        // uart check screen
        if (digitalRead(UART_CHECK_BUTTON_PIN) == SWITCH_ON_STATE) {
            display.setTextSize(1);
            display.println("--- UART KONTROL ---");
            display.println("");
            display.println("Pi Baglantisi:");
            
            if (last_pi_packet_time == 0 || (current_time - last_pi_packet_time > 1000)) {
                display.setTextSize(2);
                display.println("BAGLANTI");
                display.println("YOK");
            } else {
                display.setTextSize(2);
                display.println("AKTIF");
            }
        } 
        // sensor info screen
        else if (digitalRead(SENSOR_INFO_BUTTON_PIN) == SWITCH_ON_STATE) {
            display.setTextSize(1);
            display.println("--- SENSORLER ---");
            display.print("Sol IR : "); display.println(left_distance);
            display.print("Orta IR: "); display.println(front_distance);
            display.print("Sag IR : "); display.println(right_distance);
            display.print("HallEfc: "); display.println(digitalRead(HALL_EFFECT_PIN) == LOW ? "MIKNATIS VAR" : "YOK");
        }
        // normal telemetry screens
        else {
            display.setTextSize(1);
            if (currentState == WAITING_START) {
                display.println("WRO BEKLEMEDE");
                display.println("===============");
                display.print("Mod: "); display.println(currentMode == OPEN_RACE ? "OBSTACLE" : "OPEN");
                display.print("Yon: "); display.println(currentDirection == CLOCKWISE ? "SAAT YONU" : "S.Y. TERSI");
                display.println("Kontak Switch 1'i Ac!");
            } 
            else if (currentState == RUNNING) {
                display.setTextSize(2);
                display.println("YARISIYOR");
                display.setTextSize(1);
                display.print("Yon: "); display.println(currentDirection == CLOCKWISE ? "SAAT YONU" : "S.Y. TERSI");
                display.print("Tur Kosesi: "); display.print(corner_count); display.print("/"); display.println(TARGET_CORNER_COUNT);
            }
            else if (currentState == PARKING_SEARCH || currentState == PARKING) {
                display.setTextSize(2);
                display.println("PARK");
                display.println("ARANIYOR");
            }
            else if (currentState == FINISHED) {
                display.setTextSize(2);
                display.println("YARIS");
                display.println("BITTI");
                display.setTextSize(1);
                display.println("Sifirlamak Icin");
                display.println("Switch 1'i Kapat");
            }
            else if (currentState == EMERGENCY_STOP) {
                display.setTextSize(2);
                display.println("ACIL FREN");
            }
            
            // gyro data keeps updating under every normal screen
            display.setTextSize(1);
            display.print("\nGyro Acisi: ");
            display.print(yaw_angle);
        }
        
        display.display();
        last_display_time = current_time; 
    }
    
    delay(5); 
}