#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

#define BOTON 33
#define LED_ADELANTE 2
#define LED_RETROCEDER 4
#define LED_IZQUIERDA 5
#define LED_DERECHA 18
#define LED_STOP 19
#define LINEAR_MAX 0.5
#define ANGULAR_MAX 1.0
#define TIMEOUT 1000

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__String feedback_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool emergency_stop = false;
unsigned long last_command_time = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    String gesture = msg->data.data;
    last_command_time = millis();
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    digitalWrite(LED_ADELANTE, LOW);
    digitalWrite(LED_RETROCEDER, LOW);
    digitalWrite(LED_IZQUIERDA, LOW);
    digitalWrite(LED_DERECHA, LOW);
    digitalWrite(LED_STOP, LOW);
    if (emergency_stop) {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        digitalWrite(LED_STOP, HIGH);
        return;
    }
    if (gesture == "ADELANTE") { twist_msg.linear.x = LINEAR_MAX; digitalWrite(LED_ADELANTE, HIGH); }
    else if (gesture == "RETROCEDER") { twist_msg.linear.x = -LINEAR_MAX; digitalWrite(LED_RETROCEDER, HIGH); }
    else if (gesture == "IZQUIERDA") { twist_msg.angular.z = ANGULAR_MAX; digitalWrite(LED_IZQUIERDA, HIGH); }
    else if (gesture == "DERECHA") { twist_msg.angular.z = -ANGULAR_MAX; digitalWrite(LED_DERECHA, HIGH); }
    else if (gesture == "ALTO") { twist_msg.linear.x = 0.0; twist_msg.angular.z = 0.0; digitalWrite(LED_STOP, HIGH); }
    RCSOFTCHECK(rcl_publish(&publisher, &twist_msg, NULL));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        emergency_stop = digitalRead(BOTON);
        unsigned long now = millis();
        if (!emergency_stop && (now - last_command_time > TIMEOUT)) {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            digitalWrite(LED_ADELANTE, LOW);
            digitalWrite(LED_RETROCEDER, LOW);
            digitalWrite(LED_IZQUIERDA, LOW);
            digitalWrite(LED_DERECHA, LOW);
            digitalWrite(LED_STOP, HIGH);
        }
        if (emergency_stop) {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            digitalWrite(LED_ADELANTE, LOW);
            digitalWrite(LED_RETROCEDER, LOW);
            digitalWrite(LED_IZQUIERDA, LOW);
            digitalWrite(LED_DERECHA, LOW);
            digitalWrite(LED_STOP, HIGH);
            Serial.println("EMERGENCY STOP ACTIVATED!");
        }
        RCSOFTCHECK(rcl_publish(&publisher, &twist_msg, NULL));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_ADELANTE, OUTPUT);
    pinMode(LED_RETROCEDER, OUTPUT);
    pinMode(LED_IZQUIERDA, OUTPUT);
    pinMode(LED_DERECHA, OUTPUT);
    pinMode(LED_STOP, OUTPUT);
    pinMode(BOTON, INPUT_PULLDOWN);
    set_microros_transports();
    delay(2000);
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "microros_gesture_controller", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "gesture_command"));
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &feedback_msg, &subscription_callback, ON_NEW_DATA));
    feedback_msg.data.data = (char *) malloc(100 * sizeof(char));
    feedback_msg.data.size = 0;
    feedback_msg.data.capacity = 100;
    last_command_time = millis();
    Serial.println("micro-ROS gesture node initialized!");
}

void loop() {
    delay(10);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
