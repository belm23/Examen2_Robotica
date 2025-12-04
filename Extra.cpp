#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#define LINEAR_MAX 0.5
#define ANGULAR_MAX 0.5
#define TIMEOUT 1000

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32MultiArray proximity_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

unsigned long last_command_time = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    float left = msg->data.data[0];
    float center = msg->data.data[1];
    float right = msg->data.data[2];

    Serial.print("Recibido: ");
    Serial.print(left); Serial.print(", ");
    Serial.print(center); Serial.print(", ");
    Serial.println(right);

    float linear = LINEAR_MAX;
    float angular = 0.0;

    if(center < 0.5){
        linear = 0.0;
    } else if(center < 1.0){
        linear *= 0.5;
    }

    if(left < 0.7){
        angular += ANGULAR_MAX;
    }
    if(right < 0.7){
        angular -= ANGULAR_MAX;
    }

    twist_msg.linear.x = linear;
    twist_msg.angular.z = angular;

    RCSOFTCHECK(rcl_publish(&publisher, &twist_msg, NULL));
    last_command_time = millis();
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if(timer != NULL){
        unsigned long now = millis();
        if(now - last_command_time > TIMEOUT){
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            RCSOFTCHECK(rcl_publish(&publisher, &twist_msg, NULL));
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    set_microros_transports();

    // Inicializar array de proximidad
    proximity_msg.data.capacity = 3;
    proximity_msg.data.size = 3;
    proximity_msg.data.data = (float*)malloc(sizeof(float) * 3);

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_proximity_controller", "", &support));
    RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "proximity_zones"));
    
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &proximity_msg, &subscription_callback, ON_NEW_DATA));

    last_command_time = millis();
    Serial.println("micro-ROS Proximity Controller listo");
}

void loop() {
    delay(10);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
