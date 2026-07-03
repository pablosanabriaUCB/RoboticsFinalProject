/*
 * =============================================================
 * Firmware: Eye Tracker Servo Controller
 * Microcontrolador: ESP32 DevKit
 * Comunicación: micro-ROS sobre WiFi UDP
 * =============================================================
 * 
 * Recibe ángulos articulares en radianes desde ROS2 y
 * controla 6 servomotores PWM estándar (SG90/MG996R).
 * 
 * Incluye interpolación suave para evitar movimientos bruscos.
 * =============================================================
 */

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <ESP32Servo.h>

// ==========================================
// ==========================================


// ==========================================
// PINES DE LOS SERVOS
// ==========================================

const int SERVO_PINS[4] = {
    27,  // Servo 1 - Base (Yaw)
    14,  // Servo 2 - Hombro (Pitch)
    12,  // Servo 3 - Codo (Pitch)
    13   // Servo 4 - Muñeca (Roll/Torsión)
};

// ==========================================
// CONFIGURACIÓN DE SERVOS
// ==========================================

struct ServoConfig {
    float min_angle_rad;
    float max_angle_rad;
    int   min_pulse_us;
    int   max_pulse_us;
    float home_angle_rad;
    bool  inverted;
};

// Límites articulares coinciden con URDF (todos ±π/2 = ±1.57 para evitar romper nada)
ServoConfig servo_config[4] = {
    // min_rad,   max_rad,  min_us, max_us, home_rad, inverted
    { -1.5708,    1.5708,   500,    2500,   0.0,      false },  // Joint 1 - Base Yaw
    { -1.5708,    1.5708,   500,    2500,   0.0,      false },  // Joint 2 - Hombro Pitch
    { -1.5708,    1.5708,   500,    2500,   0.0,      false },  // Joint 3 - Codo Pitch
    { -1.5708,    1.5708,   500,    2500,   0.0,      false }   // Joint 4 - Muñeca Roll
};

// ==========================================
// OBJETOS SERVO
// ==========================================

Servo servos[4];

// ==========================================
// INTERPOLACIÓN SUAVE
// ==========================================

float current_angles[4];
float target_angles[4];

const float INTERP_SPEED = 0.15;
const unsigned long SERVO_UPDATE_INTERVAL = 20;
unsigned long last_servo_update = 0;

// ==========================================
// MICRO-ROS
// ==========================================

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray sub_msg;

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray pub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool micro_ros_connected = false;
int spin_fail_count = 0;

// ==========================================
// TIMEOUT DE SEGURIDAD
// ==========================================

unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 2000;

// ==========================================
// FUNCIONES DE SERVO
// ==========================================

int angleToPulse(int servo_idx, float angle_rad) {
    ServoConfig& cfg = servo_config[servo_idx];
    float clamped = constrain(angle_rad, cfg.min_angle_rad, cfg.max_angle_rad);
    if (cfg.inverted) {
        clamped = cfg.max_angle_rad - (clamped - cfg.min_angle_rad);
    }
    float ratio = (clamped - cfg.min_angle_rad) / (cfg.max_angle_rad - cfg.min_angle_rad);
    int pulse = cfg.min_pulse_us + (int)(ratio * (cfg.max_pulse_us - cfg.min_pulse_us));
    return constrain(pulse, cfg.min_pulse_us, cfg.max_pulse_us);
}

void setServoAngle(int servo_idx, float angle_rad) {
    if (servo_idx < 0 || servo_idx >= 4) return;
    int pulse = angleToPulse(servo_idx, angle_rad);
    servos[servo_idx].writeMicroseconds(pulse);
}

void initServosHome() {
    for (int i = 0; i < 4; i++) {
        current_angles[i] = servo_config[i].home_angle_rad;
        target_angles[i] = servo_config[i].home_angle_rad;
        setServoAngle(i, current_angles[i]);
    }
}

void updateServosSmooth() {
    for (int i = 0; i < 4; i++) {
        float diff = target_angles[i] - current_angles[i];
        if (abs(diff) > 0.001) {
            current_angles[i] += diff * INTERP_SPEED;
            setServoAngle(i, current_angles[i]);
        }
    }
}

void goHome() {
    for (int i = 0; i < 4; i++) {
        target_angles[i] = servo_config[i].home_angle_rad;
    }
}

// ==========================================
// CALLBACK DE MICRO-ROS
// ==========================================

void joint_commands_callback(const void* msgin) {
    const std_msgs__msg__Float32MultiArray* msg = 
        (const std_msgs__msg__Float32MultiArray*)msgin;
    
    if (msg->data.size >= 4) {
        for (int i = 0; i < 4; i++) {
            target_angles[i] = msg->data.data[i];
        }
        last_cmd_time = millis();
    }
}

// ==========================================
// PUBLICAR FEEDBACK
// ==========================================

void publishFeedback() {
    if (!micro_ros_connected) return;
    for (int i = 0; i < 4; i++) {
        pub_msg.data.data[i] = current_angles[i];
    }
    pub_msg.data.size = 4;
    rcl_publish(&publisher, &pub_msg, NULL);
}

// ==========================================
// CREAR ENTIDADES MICRO-ROS
// ==========================================

bool create_entities() {
    allocator = rcl_get_default_allocator();
    
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    delay(500);
    
    if (rclc_node_init_default(&node, "arm_servo_controller", "", &support) != RCL_RET_OK) {
        return false;
    }
    delay(500);
    
    if (rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "arm/joint_commands") != RCL_RET_OK) {
        return false;
    }
    delay(500);
    
    if (rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "arm/servo_feedback") != RCL_RET_OK) {
        return false;
    }
    delay(500);
    
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        return false;
    }
    delay(200);
    
    if (rclc_executor_add_subscription(
            &executor, &subscriber, &sub_msg, 
            &joint_commands_callback, ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    
    return true;
}

// ==========================================
// DESTRUIR ENTIDADES
// ==========================================

void destroy_entities() {
    rcl_subscription_fini(&subscriber, &node);
    rcl_publisher_fini(&publisher, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ==========================================
// SETUP
// ==========================================

void setup() {
    delay(2000);
    
    
    // Inicializar Servos
    for (int i = 0; i < 4; i++) {
        servos[i].setPeriodHertz(50);
        servos[i].attach(
            SERVO_PINS[i], 
            servo_config[i].min_pulse_us, 
            servo_config[i].max_pulse_us
        );
    }
    initServosHome();
    delay(1000);
    
    // Inicializar memoria para mensajes Float32MultiArray
    sub_msg.data.capacity = 4;
    sub_msg.data.size = 0;
    sub_msg.data.data = (float*)malloc(4 * sizeof(float));
    
    pub_msg.data.capacity = 4;
    pub_msg.data.size = 4;
    pub_msg.data.data = (float*)malloc(4 * sizeof(float));
    
    if (!sub_msg.data.data || !pub_msg.data.data) {
        while(1) delay(1000);
    }
    
    // Inicializar micro-ROS para usar UART/Serial nativo
    set_microros_transports();
    
    micro_ros_connected = false;
    spin_fail_count = 0;
    last_servo_update = millis();
    last_cmd_time = millis();
    
}

// ==========================================
// LOOP
// ==========================================

void loop() {
    if (!micro_ros_connected) {
        
        if (create_entities()) {
            last_cmd_time = millis();
            micro_ros_connected = true;
            spin_fail_count = 0;
        } else {
            destroy_entities();
            goHome();
            delay(2000);
        }
        return;
    }
    
    rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    
    if (rc != RCL_RET_OK) {
        spin_fail_count++;
        if (spin_fail_count > 100) {
            destroy_entities();
            goHome();
            micro_ros_connected = false;
            spin_fail_count = 0;
            delay(2000);
            return;
        }
    } else {
        spin_fail_count = 0;
    }
    
    if (millis() - last_cmd_time > CMD_TIMEOUT) {
        goHome();
    }
    
    if (millis() - last_servo_update >= SERVO_UPDATE_INTERVAL) {
        last_servo_update = millis();
        updateServosSmooth();
    }
    
    static unsigned long last_fb_time = 0;
    if (millis() - last_fb_time >= 1000) {
        last_fb_time = millis();
        publishFeedback();
    }
}
