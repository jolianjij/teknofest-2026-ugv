#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>

#include "pins.h"
#include "config.h"
#include "sbus.h"
#include "motor_mixer.h"
#include "led_status.h"
#include "oled_display.h"

// ---- Peripherals ----
static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
static Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40, Wire);
static SbusReceiver sbus;

// ---- micro-ROS ----
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t cmd_vel_sub;
static rcl_publisher_t imu_pub;
static rcl_publisher_t odom_pub;
static rcl_publisher_t diag_pub;
static rcl_timer_t publish_timer;

static geometry_msgs__msg__Twist cmd_vel_msg;
static sensor_msgs__msg__Imu imu_msg;
static nav_msgs__msg__Odometry odom_msg;
static std_msgs__msg__Float32MultiArray diag_msg;
static float diag_data[3]; // [0]=loop_hz [1]=sbus_ok(0/1) [2]=armed(0/1)

static volatile float cmd_vel_linear = 0.0f;
static volatile float cmd_vel_angular = 0.0f;
static volatile unsigned long last_cmd_vel_ms = 0;
static bool ros_agent_connected = false;

#define RCCHECK(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { errorLoop(); } }
#define RCSOFTCHECK(fn) { (void)(fn); }

static void errorLoop() {
    // micro-ROS init failed - blink both LED pins fast forever, do not proceed
    pinMode(PIN_LED_A, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    while (true) {
        digitalWrite(PIN_LED_A, !digitalRead(PIN_LED_A));
        digitalWrite(PIN_LED_B, !digitalRead(PIN_LED_A));
        delay(100);
    }
}

static void cmdVelCallback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    cmd_vel_linear = (float)msg->linear.x;
    cmd_vel_angular = (float)msg->angular.z;
    last_cmd_vel_ms = millis();
}

static void applyMotorOutputs(const MotorOutputs &m, bool armed) {
    digitalWrite(PIN_PCA9685_OE, armed ? LOW : HIGH); // hardware-level cut when disarmed
    if (!armed) {
        pca.writeMicroseconds(PCA_CH_MOTOR_LEFT, ESC_PULSE_NEUTRAL_US);
        pca.writeMicroseconds(PCA_CH_MOTOR_RIGHT, ESC_PULSE_NEUTRAL_US);
        return;
    }
    pca.writeMicroseconds(PCA_CH_MOTOR_LEFT, normalizedToPulseUs(m.left));
    pca.writeMicroseconds(PCA_CH_MOTOR_RIGHT, normalizedToPulseUs(m.right));
}

static float sbusToNormalized(uint16_t raw) {
    float v = (float)(raw - SBUS_MID) / (float)(SBUS_MAX - SBUS_MID);
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

static void publishTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Quaternion q = bno.getQuat();

    imu_msg.orientation.w = q.w();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.angular_velocity.x = gyro.x();
    imu_msg.angular_velocity.y = gyro.y();
    imu_msg.angular_velocity.z = gyro.z();
    imu_msg.linear_acceleration.x = accel.x();
    imu_msg.linear_acceleration.y = accel.y();
    imu_msg.linear_acceleration.z = accel.z();
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

    // No wheel encoders yet (see CLAUDE.md Open Decisions) - odom carries
    // orientation + commanded twist only; the Pi-side EKF fuses this with
    // lidar scan-matching for position, per the documented architecture.
    odom_msg.pose.pose.orientation = imu_msg.orientation;
    odom_msg.twist.twist.linear.x = cmd_vel_linear;
    odom_msg.twist.twist.angular.z = cmd_vel_angular;
    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));

    static unsigned long lastLoopMs = 0;
    unsigned long now = millis();
    diag_data[0] = lastLoopMs > 0 ? 1000.0f / (float)(now - lastLoopMs) : 0.0f;
    diag_data[1] = sbus.failsafe() ? 0.0f : 1.0f;
    lastLoopMs = now;
    // diag_data[2] (armed) is set in loop()
    RCSOFTCHECK(rcl_publish(&diag_pub, &diag_msg, NULL));
}

void setup() {
    pinMode(PIN_PCA9685_OE, OUTPUT);
    digitalWrite(PIN_PCA9685_OE, HIGH); // disabled until armed

    ledStatusBegin();
    ledStatusUpdate(RobotState::BOOT);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    oledBegin();

    if (!bno.begin()) {
        errorLoop(); // IMU is safety-relevant (heading for EKF) - do not run without it
    }
    bno.setExtCrystalUse(true);

    pca.begin();
    pca.setPWMFreq(PCA9685_PWM_FREQ_HZ);

    sbus.begin(Serial2, PIN_SBUS_RX, PIN_SBUS_TX);

    // ---- micro-ROS (USB serial transport) ----
    set_microros_serial_transports(Serial);
    delay(2000); // let the agent-side connection settle before init

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/esp32/imu"));

    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/esp32/odom"));

    RCCHECK(rclc_publisher_init_default(
        &diag_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/esp32/diagnostics"));

    diag_msg.data.data = diag_data;
    diag_msg.data.size = 3;
    diag_msg.data.capacity = 3;

    RCCHECK(rclc_timer_init_default(
        &publish_timer, &support,
        RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
        publishTimerCallback));

    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));

    ros_agent_connected = true;
}

void loop() {
    sbus.update();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    bool sbusOk = !sbus.failsafe();
    bool armed = sbusOk && sbusToNormalized(sbus.channel(SBUS_CH_ARM)) > 0.0f;
    // ^ SBUS_ARM_THRESHOLD is in raw units; sbusToNormalized(SBUS_ARM_THRESHOLD) > 0 by construction of SBUS_MID.
    bool autonomousMode = sbusOk && sbus.channel(SBUS_CH_MODE) > SBUS_MODE_THRESHOLD;

    RobotState state;
    MotorOutputs outputs{0.0f, 0.0f};

    if (!sbusOk || !armed) {
        state = sbusOk ? RobotState::DISARMED : RobotState::FAILSAFE;
    } else if (autonomousMode) {
        bool cmdFresh = (millis() - last_cmd_vel_ms) < CMD_VEL_TIMEOUT_MS;
        state = RobotState::AUTONOMOUS;
        outputs = cmdFresh ? mixDifferential(cmd_vel_linear, cmd_vel_angular)
                            : MotorOutputs{0.0f, 0.0f};
    } else {
        state = RobotState::MANUAL;
        float steer = sbusToNormalized(sbus.channel(SBUS_CH_STEER));
        float throttle = sbusToNormalized(sbus.channel(SBUS_CH_THROTTLE));
        outputs = mixDifferential(throttle, steer);
    }

    applyMotorOutputs(outputs, armed && sbusOk);
    diag_data[2] = (armed && sbusOk) ? 1.0f : 0.0f;

    ledStatusUpdate(state);

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    oledUpdate(state, euler.x(), sbusOk, ros_agent_connected);
}
