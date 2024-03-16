#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_publisher_t publisher_speed, publisher_pwm, publisher_normalized_speed;
rcl_subscription_t subscriber_pwm;
std_msgs__msg__Int32 msg, msg_pwm_normalized;
std_msgs__msg__Float32 msg_speed, msg_pwm, msg_normalized_speed;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer1, timer2;

#define LED_PIN 13
#define IN_1 26 
#define IN_2 27 
#define EN_A 18 //Canal A del encoder
#define EN_B 19 //Canal B del encoder

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define PWM_CH1 0
#define FREQ1 5000
#define RES1 8

#define PWM_CH2 1
#define FREQ2 5000
#define RES2 8

//tiempo de muestreo
#define DT 20000

//Caracteristicas del motor
#define REDUCTOR 34
#define CPR 12

//Variables del programa
double pos = 0;
double pos_ant = 0;
double timer = 0;
float vel = 0;

//**********
float kp = 19.68;
float ki = 70.58;
float kd = 0.013;
//float T = 0.01; //periodo de muestreo
float y[2] = {0,0}; //salida y[0] salida actual y[1] anterior
float e[3] = {0,0,0}; //e[0] error actual e[1] error anterior y e[2] error 2 pasos anterior

float K1, K2, K3;
//***********

//Función que se ejecuta cada vez que hay un cambio (FALLING) en el canal A del encoder
void IRAM_ATTR int_callback(){
  if (digitalRead(EN_B)==0){
    pos = pos + 1;
  } else {
    pos = pos - 1;
  }
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void controller_callback(rcl_timer_t * timer1, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if(timer1 != NULL) {
    vel = (((pos - pos_ant)/DT)/(CPR*REDUCTOR))*(1000000 * 2.0 * 3.1416); // rad/s
    
    pos_ant = pos;

    msg_speed.data = vel;

    e[0] = msg_pwm.data - vel;

    // Programación del controlador PID
    y[0] = K1 * e[0] + K2 * e[1] + K3 * e[2] + y[1];

    // limitar la salida
    if(y[0] > 255) y[0] = 255;
    if(y[0] < -255) y[0] = -255; //maximo PWM de Arduino

    //desplazamos las muestras para las anteriores
    e[2] = e[1];
    e[1] = e[0];
    y[1] = y[0];

    if(y[0] >= 0){
      ledcWrite(PWM_CH2, int(y[0]));
      ledcWrite(PWM_CH1, 0);
    }
    else{
      ledcWrite(PWM_CH1, int(-y[0]));
      ledcWrite(PWM_CH2, 0);
    }
    msg_pwm_normalized.data = int(y[0]);
    msg_normalized_speed.data = vel/16.94;

    RCSOFTCHECK(rcl_publish(&publisher_speed, &msg_speed, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pwm, &msg_pwm_normalized, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_normalized_speed, &msg_normalized_speed, NULL));    

  }
  
}

void pwm_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg_data = (const std_msgs__msg__Float32 *)msgin;
    msg_pwm.data = msg_data->data * 16.94; 

}


void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Controller", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "angular_speed"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "pwm_value"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_normalized_speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "normalized_speed"));

  RCCHECK(rclc_subscription_init_default(
    &subscriber_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "setpoint"));

  // create timer,
  const unsigned int timer_1 = 20;
  RCCHECK(rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(timer_1),
    controller_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_pwm, &msg_pwm, &pwm_subscription_callback, ON_NEW_DATA));

  pinMode (IN_1, OUTPUT);
  pinMode (IN_2, OUTPUT);

  //Configurar el canal de PWM para in1
  ledcSetup(PWM_CH1, FREQ1, RES1);
  ledcAttachPin(IN_1, PWM_CH1);

    //Configurar el canal de PWM para in2
  ledcSetup(PWM_CH2, FREQ2, RES2);
  ledcAttachPin(IN_2, PWM_CH2);

  pinMode (EN_A, INPUT);
  pinMode (EN_B, INPUT);

  //Definimos la interrupción la cual se ejecutara en cada flanco de bajada del canal A del encoder
  attachInterrupt(EN_A, int_callback, FALLING);

  K1 = kp + (DT/1000000.0)*ki + kd/(DT/1000000.0);
  K2 = - kp - 2.0*(kd/(DT/1000000.0));
  K3 = kd/(DT/1000000.0);
}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
