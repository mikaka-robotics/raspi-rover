#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif

#define STR_SIZE (100) //最大の受信文字数

TaskHandle_t xTask1;

rcl_publisher_t publisher_right_float64;
rcl_publisher_t publisher_left_float64;
rcl_publisher_t odom_publisher;

nav_msgs__msg__Odometry msg_odom;
std_msgs__msg__Float64 msg_left_speed_float64;
std_msgs__msg__Float64 msg_right_speed_float64;

rcl_subscription_t sub_target_left_speed_float64;
rcl_subscription_t sub_target_right_speed_float64;
std_msgs__msg__Float64 msg_target_left_speed_float64;
std_msgs__msg__Float64 msg_target_right_speed_float64;

rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

Servo servo_left;
Servo servo_right;

ESP32Encoder encoder_left;
ESP32Encoder encoder_right;

// 左右のモータPWM信号に使用するピンを定義
const int LEFT_MOTOR_PWM_PIN = 33;
const int RIGHT_MOTOR_PWM_PIN = 32;


// エンコーダカウントとラジアン/度の変換係数を定義
const double DEGREE_PER_ENCODER_COUNT = 3.75;
const double RADIAN_PER_ENCODER_COUNT = 0.06544985;

// ロボットのホイールベースを定義
const double WHEEL_BASE = 0.1;

// PID制御器のゲインを定義
const double KP_GAIN_right = 1.9;
const double KI_GAIN_right = 0.3;
const double KD_GAIN_right = 0.007;

// PID制御器のゲインを定義
const double KP_GAIN_left = 1.9;
const double KI_GAIN_left = 0.3;
const double KD_GAIN_left = 0.007;

double target_left_radsec=0;
double target_right_radsec=0;

double left_radsec=0;
double right_radsec=0;

double tire_radius = 0.03;
double dt=1;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void setup() {

   Serial.begin(115200);
   delay(100);
   Serial.println("Start Program!");
   
   servo_left.setPeriodHertz(50);
   servo_right.setPeriodHertz(50);
   servo_left.attach(LEFT_MOTOR_PWM_PIN, 500, 2500); 
   servo_right.attach(RIGHT_MOTOR_PWM_PIN, 500, 2500); 

   servo_left.write(90);
   servo_right.write(90);

   ESP32Encoder::useInternalWeakPullResistors=UP;
   encoder_left.attachHalfQuad(2, 15);
   encoder_right.attachHalfQuad(14, 12);
  
  set_microros_wifi_transports("ASANOwifi", "yukiwarisou", "192.168.37.199", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_right_float64,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "right_radsec"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_left_float64,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "left_radsec"));

  RCCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom_asano"));
    
  // Subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &sub_target_left_speed_float64, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "target_left_radsec")
   );

  // Subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &sub_target_right_speed_float64, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "target_right_radsec")
   );

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_target_left_speed_float64, &msg_target_left_speed_float64, 
                   &callback_target_left_speed, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_target_right_speed_float64, &msg_target_right_speed_float64, 
                   &callback_target_right_speed, ON_NEW_DATA);


  xTaskCreatePinnedToCore(
      task_controller,
      "task_controller",
      8192,
      NULL,
      1,
      &xTask1,
      1
    );

}

unsigned long now;
unsigned long starttime1 = 0;
static long leftPwm = 90;
static long rightPwm = 90;

void task_controller(void *pvParameters){
  while (1) {
    now = millis();

    dt = (now - starttime1)/1000.; //sec
    static long prevLeftEncoderCount = 0;
    static long prevRightEncoderCount = 0;
    long leftEncoderCount = -encoder_left.getCount();
    long rightEncoderCount = encoder_right.getCount();   

    // 左右のエンコーダカウント数の差分から、左右のホイールの速度を算出
    long deltaLeftEncoderCount = leftEncoderCount - prevLeftEncoderCount;
    long deltaRightEncoderCount = rightEncoderCount - prevRightEncoderCount;

    prevLeftEncoderCount = leftEncoderCount;
    prevRightEncoderCount = rightEncoderCount;
    left_radsec = (deltaLeftEncoderCount * RADIAN_PER_ENCODER_COUNT) / dt;
    right_radsec = (deltaRightEncoderCount * RADIAN_PER_ENCODER_COUNT) / dt;

    // 左右のホイール速度と目標角速度の差分から、左右のモータ制御に使用するエラーを算出
    double leftError = target_left_radsec - left_radsec;
    double rightError = target_right_radsec - right_radsec;

    // PID制御
    // 前回のエラーを保存
    static double leftPrevError = 0.0;
    static double rightPrevError = 0.0;

    // 前回のエラーと積分値から、微分値を算出
    static double leftIntegral = 0.0;
    static double rightIntegral = 0.0;
    leftIntegral += leftError * dt;
    rightIntegral += rightError * dt;
    double leftDerivative = (leftError - leftPrevError) / dt;
    double rightDerivative = (rightError - rightPrevError) / dt;
    leftPrevError = leftError;
    rightPrevError = rightError;

    // PID制御によって算出された制御値から、左右のモータのPWM値を計算
    double leftControl = KP_GAIN_left * leftError + KI_GAIN_left * leftIntegral + KD_GAIN_left * leftDerivative;
    double rightControl = KP_GAIN_right * rightError + KI_GAIN_right * rightIntegral + KD_GAIN_right * rightDerivative;
    leftPwm = leftPwm + leftControl;
    rightPwm = rightPwm - rightControl;

    // PWM値を所定の範囲に制限
    leftPwm = constrain(leftPwm, 0, 180);
    rightPwm = constrain(rightPwm, 0, 180);

    

    // 左右のモータにPWM値を出力
    servo_left.write(leftPwm);
    servo_right.write(rightPwm); 

    starttime1 = now;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void callback_target_left_speed(const void *raw_msg){
    std_msgs__msg__Float64 *msg = (std_msgs__msg__Float64 *)raw_msg;
    target_left_radsec = msg->data;
}

void callback_target_right_speed(const void *raw_msg){
    std_msgs__msg__Float64 *msg = (std_msgs__msg__Float64 *)raw_msg;
    target_right_radsec = msg->data;
}




void loop() {

    delay(10);

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200)));

    // オドメトリの更新
    double leftDistance = left_radsec * dt;
    double rightDistance = right_radsec * dt;
    double deltaDistance = (leftDistance + rightDistance) / 2.0;
    double deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE;

    // 現在の位置と向きを更新
    static double xPos = 0.0;
    static double yPos = 0.0;
    static double theta = 0.0;
    xPos += deltaDistance * cos(theta);
    yPos += deltaDistance * sin(theta);
    theta += deltaTheta;

    // オドメトリメッセージを更新
    msg_odom.pose.pose.position.x = xPos;
    msg_odom.pose.pose.position.y = yPos;
    msg_odom.pose.pose.position.z = 0.0;
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;
    msg_odom.pose.pose.orientation.z = sin(theta / 2.0);
    msg_odom.pose.pose.orientation.w = cos(theta / 2.0);
    msg_odom.twist.twist.linear.x = deltaDistance / dt;
    msg_odom.twist.twist.angular.z = deltaDistance / dt;
          

    msg_left_speed_float64.data =  left_radsec;
    msg_right_speed_float64.data = right_radsec;
        
    RCSOFTCHECK(rcl_publish(&publisher_right_float64, &msg_right_speed_float64, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_left_float64, &msg_left_speed_float64, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &msg_odom, NULL));
      

}
