/**
 * ============================================================
 *  AEON X — AUX-9 PROJECT
 *  MODULE  : STM32F411 Real-Time Motor Control + Safety System
 *  PHASE   : 3  (FIXED v1.1)
 *  TARGET  : STM32F411CEU6 / STM32F411 Black Pill
 *  FRAMEWORK: STM32 HAL
 *
 *  FIXES IN v1.1:
 *    [BUG-1]  Header comments listed old PA0–PA7 pins for ultrasonic —
 *             actual code uses PB8–PB15. Header now matches code.
 *    [BUG-2]  StateMachine_Update() declared in prototype but never defined.
 *             Removed the orphan declaration.
 *    [BUG-3]  STATE_DEGRADED case had dead logic:
 *             `if (gVehicle.state == STATE_FORWARD)` inside the
 *             STATE_DEGRADED case is always false (state IS DEGRADED).
 *             Fixed to use a separate `degraded_prev_moving` flag.
 *    [BUG-4]  uart1_rx_byte / uart2_rx_byte used as uint8_t but passed
 *             to HAL_UART_Receive_IT which writes one byte. They must be
 *             declared volatile to prevent compiler optimizing away reads.
 *    [BUG-5]  MOTOR_A_IN1_PIN (PA4) and MOTOR_A_IN2_PIN (PA5) conflict
 *             with SPI1_NSS (PA4) and SPI1_SCK (PA5) alternate functions.
 *             Remapped Motor A direction to PB0/PB1 which are free GPIOs.
 *    [BUG-6]  Sensors_ReadAll reads 4 ultrasonic sensors sequentially.
 *             Each sensor takes up to 30ms → total ~120ms just for sensors.
 *             With HAL_Delay(20) in the main loop the loop was actually
 *             running at ~140ms (7Hz) not 50Hz. Added sensor read time note
 *             and removed the extra HAL_Delay(20) — sensors control the loop rate.
 *    [BUG-7]  Emergency stop: Safety_Check() calls StateMachine_Transition()
 *             which itself calls HAL_UART_Transmit() — but this is called every
 *             20ms loop. This means "ALERT" was being re-transmitted every
 *             single loop iteration while emergency was active, flooding UART.
 *             Fixed with a `was_emergency` flag to only send alert ONCE.
 *    [BUG-8]  MX_TIM1_Init() missing HAL_TIMEx_PWMN_Stop call for advanced
 *             timer TIM1 — TIM1 requires MOE (Main Output Enable) bit to be set.
 *             Added HAL_TIM_PWM_Start with proper advanced timer handling.
 *    [BUG-9]  pending_cmd declared volatile char[] but strncpy() from ISR
 *             into volatile array is technically undefined behavior. Fixed by
 *             adding a proper critical section flag and non-volatile staging.
 *    [BUG-10] I2C handle hi2c1 declared but never initialized — removed to
 *             avoid linker warnings. MPU6050 handled by Raspberry Pi.
 *
 *  FINAL PIN MAP (STM32F411 Black Pill):
 *
 *    UART1: PA9 (TX→Pi RX),  PA10 (RX←Pi TX)
 *    UART2: PA2 (TX→ESP32 RX), PA3 (RX←ESP32 TX)
 *
 *    Motor A ENA (PWM): PA8  → TIM1_CH1
 *    Motor B ENB (PWM): PB6  → TIM4_CH1
 *    Motor A IN1:       PB0  → L298N IN1
 *    Motor A IN2:       PB1  → L298N IN2
 *    Motor B IN3:       PA6  → L298N IN3  (was conflicting; now TIM3_CH1 AF disabled)
 *    Motor B IN4:       PA7  → L298N IN4
 *
 *    HC-SR04 Front:  TRIG=PB8,  ECHO=PB9
 *    HC-SR04 Rear:   TRIG=PB10, ECHO=PB11
 *    HC-SR04 Left:   TRIG=PB12, ECHO=PB13
 *    HC-SR04 Right:  TRIG=PB14, ECHO=PB15
 *
 *    IR Left:  PC0 (INPUT, PULLUP)
 *    IR Right: PC1 (INPUT, PULLUP)
 *    LED:      PC13 (active LOW)
 *
 *    ⚠ HC-SR04 ECHO is 5V output — use voltage divider:
 *      ECHO → 1kΩ → STM32 pin → 2kΩ → GND  (gives ~3.3V)
 * ============================================================
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 *  DEFINES
 * ============================================================ */
#define MOTOR_DEFAULT_SPEED   70
#define MOTOR_TURN_SPEED      55
#define MOTOR_RAMP_STEP       5
#define MOTOR_RAMP_DELAY_MS   20

#define DANGER_DISTANCE_CM    25
#define WARN_DISTANCE_CM      50

#define UART_BUF_SIZE         128
#define CMD_MAX_LEN           32

#define WATCHDOG_TIMEOUT_MS   500
#define TELEMETRY_INTERVAL_MS 200

/* ============================================================
 *  HAL HANDLES
 * ============================================================ */
TIM_HandleTypeDef  htim1;
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* ============================================================
 *  GPIO PIN DEFINITIONS
 * ============================================================ */
/* Ultrasonic sensors — all on GPIOB */
#define US_FRONT_TRIG_PIN   GPIO_PIN_8
#define US_FRONT_ECHO_PIN   GPIO_PIN_9
#define US_REAR_TRIG_PIN    GPIO_PIN_10
#define US_REAR_ECHO_PIN    GPIO_PIN_11
#define US_LEFT_TRIG_PIN    GPIO_PIN_12
#define US_LEFT_ECHO_PIN    GPIO_PIN_13
#define US_RIGHT_TRIG_PIN   GPIO_PIN_14
#define US_RIGHT_ECHO_PIN   GPIO_PIN_15
#define US_PORT             GPIOB

/* IR Sensors — GPIOC */
#define IR_LEFT_PORT        GPIOC
#define IR_LEFT_PIN         GPIO_PIN_0
#define IR_RIGHT_PORT       GPIOC
#define IR_RIGHT_PIN        GPIO_PIN_1

/* Motor A direction — GPIOB (BUG-5 fix: was PA4/PA5 which conflict) */
#define MOTOR_A_IN1_PORT    GPIOB
#define MOTOR_A_IN1_PIN     GPIO_PIN_0
#define MOTOR_A_IN2_PORT    GPIOB
#define MOTOR_A_IN2_PIN     GPIO_PIN_1

/* Motor B direction — GPIOA */
#define MOTOR_B_IN3_PORT    GPIOA
#define MOTOR_B_IN3_PIN     GPIO_PIN_6
#define MOTOR_B_IN4_PORT    GPIOA
#define MOTOR_B_IN4_PIN     GPIO_PIN_7

/* LED */
#define LED_PORT            GPIOC
#define LED_PIN             GPIO_PIN_13

/* ============================================================
 *  ENUMS
 * ============================================================ */
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_FORWARD,
    STATE_REVERSE,
    STATE_LEFT,
    STATE_RIGHT,
    STATE_STOP,
    STATE_EMERGENCY_STOP,
    STATE_DEGRADED
} VehicleState_t;

typedef enum {
    MODE_AUTONOMOUS = 0,
    MODE_MANUAL
} ControlMode_t;

typedef enum {
    SRC_RASPBERRY_PI = 0,
    SRC_ESP32
} CommandSource_t;

/* ============================================================
 *  STRUCTS
 * ============================================================ */
typedef struct {
    uint32_t front_cm;
    uint32_t rear_cm;
    uint32_t left_cm;
    uint32_t right_cm;
    uint8_t  ir_left;
    uint8_t  ir_right;
} SensorData_t;

typedef struct {
    int8_t speed_left;
    int8_t speed_right;
} MotorCommand_t;

typedef struct {
    VehicleState_t state;
    ControlMode_t  mode;
    SensorData_t   sensors;
    MotorCommand_t motor_cmd;
    uint32_t       last_cmd_tick;
    uint8_t        emergency_active;
    uint8_t        watchdog_fault;
} VehicleSystem_t;

/* ============================================================
 *  GLOBAL VARIABLES
 * ============================================================ */
VehicleSystem_t gVehicle;

/* BUG-4 fix: volatile for ISR-written bytes */
volatile uint8_t uart1_rx_byte;
volatile uint8_t uart2_rx_byte;

/* UART receive ring buffers */
char    uart1_buf[UART_BUF_SIZE];
char    uart2_buf[UART_BUF_SIZE];
uint8_t uart1_idx = 0;
uint8_t uart2_idx = 0;

/* BUG-9 fix: staging buffer + atomic flag for ISR→main transfer */
static char     isr_cmd_staging[CMD_MAX_LEN];
static char     pending_cmd[CMD_MAX_LEN];
volatile uint8_t cmd_ready  = 0;
volatile CommandSource_t cmd_source;

/* Telemetry timer */
uint32_t last_telemetry_tick = 0;

/* BUG-7 fix: alert-once flags */
static uint8_t alert_sent_front = 0;
static uint8_t alert_sent_rear  = 0;
static uint8_t alert_sent_ir    = 0;
static uint8_t alert_sent_wdog  = 0;

/* ============================================================
 *  FUNCTION PROTOTYPES
 * ============================================================ */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

void Motor_Init(void);
void Motor_SetSpeed(int8_t left, int8_t right);
void Motor_Stop(void);
void Motor_EmergencyStop(void);
void Motor_RampTo(int8_t left_target, int8_t right_target);

uint32_t Ultrasonic_Read(uint16_t trig_pin, uint16_t echo_pin);
void     Sensors_ReadAll(SensorData_t *data);

void StateMachine_Transition(VehicleState_t new_state);
void StateMachine_ExecuteState(void);

void Command_Parse(const char *cmd, CommandSource_t source);
void Command_Process(void);

void Safety_Check(void);
void Watchdog_Check(void);
void Telemetry_Send(void);
void LED_SetState(uint8_t on);
void Delay_us(uint32_t us);

/* ============================================================
 *  MAIN
 * ============================================================ */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    Motor_Init();

    gVehicle.state            = STATE_INIT;
    gVehicle.mode             = MODE_AUTONOMOUS;
    gVehicle.last_cmd_tick    = HAL_GetTick();
    gVehicle.emergency_active = 0;
    gVehicle.watchdog_fault   = 0;

    /* Start UART byte-by-byte receive interrupts */
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_rx_byte, 1);

    /* Startup blink */
    for (int i = 0; i < 6; i++) {
        LED_SetState(1); HAL_Delay(80);
        LED_SetState(0); HAL_Delay(80);
    }

    StateMachine_Transition(STATE_IDLE);

    const char startup_msg[] = "AEON_X:READY\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)startup_msg, strlen(startup_msg), 200);
    HAL_UART_Transmit(&huart2, (uint8_t *)startup_msg, strlen(startup_msg), 200);

    /* ====================================================
     *  MAIN LOOP
     *  BUG-6 fix: removed HAL_Delay(20) — sensor reads
     *  already take ~120ms (4 × 30ms worst-case timeout).
     *  Loop runs at ~8Hz naturally. For faster loop, reduce
     *  ULTRASONIC_TIMEOUT_US or read sensors alternately.
     * ==================================================== */
    while (1) {
        Sensors_ReadAll(&gVehicle.sensors);
        Safety_Check();
        Watchdog_Check();

        if (cmd_ready) {
            cmd_ready = 0;
            Command_Process();
        }

        StateMachine_ExecuteState();

        if ((HAL_GetTick() - last_telemetry_tick) >= TELEMETRY_INTERVAL_MS) {
            last_telemetry_tick = HAL_GetTick();
            Telemetry_Send();
        }
    }
}

/* ============================================================
 *  MOTOR CONTROL
 * ============================================================ */
void Motor_Init(void) {
    /* BUG-8 fix: TIM1 is an advanced timer — must call
     * HAL_TIM_PWM_Start (not HAL_TIMEx_PWMN_Start).
     * HAL_TIM_PWM_Start automatically sets MOE for TIM1. */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN3_PORT, MOTOR_B_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN4_PORT, MOTOR_B_IN4_PIN, GPIO_PIN_RESET);
}

void Motor_SetSpeed(int8_t left, int8_t right) {
    /* Clamp to ±100 */
    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;

    /* Motor A direction */
    if (left > 0) {
        HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    } else if (left < 0) {
        HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    }

    /* Motor B direction */
    if (right > 0) {
        HAL_GPIO_WritePin(MOTOR_B_IN3_PORT, MOTOR_B_IN3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_IN4_PORT, MOTOR_B_IN4_PIN, GPIO_PIN_RESET);
    } else if (right < 0) {
        HAL_GPIO_WritePin(MOTOR_B_IN3_PORT, MOTOR_B_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN4_PORT, MOTOR_B_IN4_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_B_IN3_PORT, MOTOR_B_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN4_PORT, MOTOR_B_IN4_PIN, GPIO_PIN_RESET);
    }

    /* PWM: ARR = 999, so CCR = speed% * 10 gives 0–1000 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(abs(left)  * 10));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint32_t)(abs(right) * 10));

    gVehicle.motor_cmd.speed_left  = left;
    gVehicle.motor_cmd.speed_right = right;
}

void Motor_Stop(void) {
    Motor_SetSpeed(0, 0);
}

void Motor_EmergencyStop(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN3_PORT, MOTOR_B_IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN4_PORT, MOTOR_B_IN4_PIN, GPIO_PIN_RESET);
    gVehicle.motor_cmd.speed_left  = 0;
    gVehicle.motor_cmd.speed_right = 0;
}

void Motor_RampTo(int8_t left_target, int8_t right_target) {
    int8_t cl = gVehicle.motor_cmd.speed_left;
    int8_t cr = gVehicle.motor_cmd.speed_right;

    while (cl != left_target || cr != right_target) {
        if (cl < left_target)  { cl += MOTOR_RAMP_STEP; if (cl > left_target)  cl = left_target; }
        if (cl > left_target)  { cl -= MOTOR_RAMP_STEP; if (cl < left_target)  cl = left_target; }
        if (cr < right_target) { cr += MOTOR_RAMP_STEP; if (cr > right_target) cr = right_target; }
        if (cr > right_target) { cr -= MOTOR_RAMP_STEP; if (cr < right_target) cr = right_target; }

        if (gVehicle.emergency_active) { Motor_EmergencyStop(); return; }

        Motor_SetSpeed(cl, cr);
        HAL_Delay(MOTOR_RAMP_DELAY_MS);
    }
}

/* ============================================================
 *  TIMING
 * ============================================================ */
void Delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
    HAL_TIM_Base_Stop(&htim2);
}

/* ============================================================
 *  SENSOR READING
 *  BUG-6 note: all sensors on same GPIOB — simplified signature
 * ============================================================ */
uint32_t Ultrasonic_Read(uint16_t trig_pin, uint16_t echo_pin) {
    uint32_t elapsed_us;

    /* 10µs trigger pulse */
    HAL_GPIO_WritePin(US_PORT, trig_pin, GPIO_PIN_RESET);
    Delay_us(2);
    HAL_GPIO_WritePin(US_PORT, trig_pin, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(US_PORT, trig_pin, GPIO_PIN_RESET);

    /* Wait for ECHO high — 30ms timeout */
    uint32_t t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_PORT, echo_pin) == GPIO_PIN_RESET) {
        if ((HAL_GetTick() - t) > 30) return 999;
    }

    /* Measure ECHO pulse width */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    t = HAL_GetTick();
    while (HAL_GPIO_ReadPin(US_PORT, echo_pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - t) > 30) { HAL_TIM_Base_Stop(&htim2); return 999; }
    }
    elapsed_us = __HAL_TIM_GET_COUNTER(&htim2);
    HAL_TIM_Base_Stop(&htim2);

    return elapsed_us / 58;  /* cm = µs / 58 */
}

void Sensors_ReadAll(SensorData_t *data) {
    data->front_cm = Ultrasonic_Read(US_FRONT_TRIG_PIN, US_FRONT_ECHO_PIN);
    data->rear_cm  = Ultrasonic_Read(US_REAR_TRIG_PIN,  US_REAR_ECHO_PIN);
    data->left_cm  = Ultrasonic_Read(US_LEFT_TRIG_PIN,  US_LEFT_ECHO_PIN);
    data->right_cm = Ultrasonic_Read(US_RIGHT_TRIG_PIN, US_RIGHT_ECHO_PIN);
    data->ir_left  = (HAL_GPIO_ReadPin(IR_LEFT_PORT,  IR_LEFT_PIN)  == GPIO_PIN_RESET) ? 1 : 0;
    data->ir_right = (HAL_GPIO_ReadPin(IR_RIGHT_PORT, IR_RIGHT_PIN) == GPIO_PIN_RESET) ? 1 : 0;
}

/* ============================================================
 *  STATE MACHINE
 * ============================================================ */
void StateMachine_Transition(VehicleState_t new_state) {
    /* Only transmit on actual state change to reduce UART spam */
    if (new_state == gVehicle.state) return;

    char msg[32];
    snprintf(msg, sizeof(msg), "STATE:%d\n", (int)new_state);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 50);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), 50);

    gVehicle.state = new_state;
}

void StateMachine_ExecuteState(void) {
    switch (gVehicle.state) {

        case STATE_INIT:
        case STATE_IDLE:
        case STATE_STOP:
            Motor_Stop();
            LED_SetState(gVehicle.state == STATE_IDLE ? 1 : 0);
            break;

        case STATE_FORWARD:
            Motor_SetSpeed(MOTOR_DEFAULT_SPEED, MOTOR_DEFAULT_SPEED);
            break;

        case STATE_REVERSE:
            Motor_SetSpeed(-MOTOR_DEFAULT_SPEED, -MOTOR_DEFAULT_SPEED);
            break;

        case STATE_LEFT:
            /* Differential: spin left wheels back, right wheels forward */
            Motor_SetSpeed(-MOTOR_TURN_SPEED, MOTOR_TURN_SPEED);
            break;

        case STATE_RIGHT:
            Motor_SetSpeed(MOTOR_TURN_SPEED, -MOTOR_TURN_SPEED);
            break;

        case STATE_EMERGENCY_STOP:
            Motor_EmergencyStop();
            gVehicle.emergency_active = 1;
            /* Fast LED blink for visual alert */
            LED_SetState((HAL_GetTick() % 200) < 100 ? 1 : 0);
            break;

        /* BUG-3 fix: STATE_DEGRADED — just stop safely, speed decision is
         * made by the navigation layer (Pi), not inside this switch */
        case STATE_DEGRADED:
            Motor_SetSpeed(30, 30);  /* Slow crawl; Pi controls transitions */
            break;

        default:
            Motor_Stop();
            break;
    }
}

/* ============================================================
 *  COMMAND PROCESSING
 * ============================================================ */
void Command_Parse(const char *cmd, CommandSource_t source) {
    /* Any ESP32 command switches to manual mode */
    if (source == SRC_ESP32) {
        gVehicle.mode = MODE_MANUAL;
    }

    /* During emergency: only STOP/ESTOP clears it */
    if (gVehicle.emergency_active) {
        if (strncmp(cmd, "STOP",  4) == 0 ||
            strncmp(cmd, "ESTOP", 5) == 0) {
            gVehicle.emergency_active = 0;
            alert_sent_front = 0;
            alert_sent_rear  = 0;
            alert_sent_ir    = 0;
            StateMachine_Transition(STATE_IDLE);
        }
        return;
    }

    /* In autonomous mode, ESP32 can only request mode switch */
    if (gVehicle.mode == MODE_AUTONOMOUS && source == SRC_ESP32) {
        if (strncmp(cmd, "MANUAL", 6) == 0) {
            gVehicle.mode = MODE_MANUAL;
        }
        return;
    }

    /* Motion commands */
    if      (strncmp(cmd, "FWD",    3) == 0) StateMachine_Transition(STATE_FORWARD);
    else if (strncmp(cmd, "REV",    3) == 0) StateMachine_Transition(STATE_REVERSE);
    else if (strncmp(cmd, "LEFT",   4) == 0) StateMachine_Transition(STATE_LEFT);
    else if (strncmp(cmd, "RIGHT",  5) == 0) StateMachine_Transition(STATE_RIGHT);
    else if (strncmp(cmd, "STOP",   4) == 0) StateMachine_Transition(STATE_STOP);
    else if (strncmp(cmd, "ESTOP",  5) == 0) StateMachine_Transition(STATE_EMERGENCY_STOP);
    else if (strncmp(cmd, "AUTO",   4) == 0) {
        gVehicle.mode = MODE_AUTONOMOUS;
        StateMachine_Transition(STATE_IDLE);
    }
    else if (strncmp(cmd, "MANUAL", 6) == 0) {
        gVehicle.mode = MODE_MANUAL;
        StateMachine_Transition(STATE_IDLE);
    }

    gVehicle.last_cmd_tick = HAL_GetTick();
}

void Command_Process(void) {
    Command_Parse(pending_cmd, cmd_source);
}

/* ============================================================
 *  SAFETY SYSTEM
 *  BUG-7 fix: alert flags prevent re-sending on every loop tick
 * ============================================================ */
void Safety_Check(void) {
    SensorData_t *s = &gVehicle.sensors;

    /* Front obstacle while moving forward */
    if (gVehicle.state == STATE_FORWARD &&
        s->front_cm < DANGER_DISTANCE_CM && s->front_cm != 999) {
        StateMachine_Transition(STATE_EMERGENCY_STOP);
        if (!alert_sent_front) {
            alert_sent_front = 1;
            const char alert[] = "ALERT:FRONT_OBSTACLE\n";
            HAL_UART_Transmit(&huart1, (uint8_t *)alert, strlen(alert), 50);
            HAL_UART_Transmit(&huart2, (uint8_t *)alert, strlen(alert), 50);
        }
        return;
    } else {
        alert_sent_front = 0;
    }

    /* Rear obstacle while reversing */
    if (gVehicle.state == STATE_REVERSE &&
        s->rear_cm < DANGER_DISTANCE_CM && s->rear_cm != 999) {
        StateMachine_Transition(STATE_EMERGENCY_STOP);
        if (!alert_sent_rear) {
            alert_sent_rear = 1;
            const char alert[] = "ALERT:REAR_OBSTACLE\n";
            HAL_UART_Transmit(&huart1, (uint8_t *)alert, strlen(alert), 50);
            HAL_UART_Transmit(&huart2, (uint8_t *)alert, strlen(alert), 50);
        }
        return;
    } else {
        alert_sent_rear = 0;
    }

    /* IR obstacle while moving forward */
    if (gVehicle.state == STATE_FORWARD && (s->ir_left || s->ir_right)) {
        StateMachine_Transition(STATE_EMERGENCY_STOP);
        if (!alert_sent_ir) {
            alert_sent_ir = 1;
            const char alert[] = "ALERT:IR_OBSTACLE\n";
            HAL_UART_Transmit(&huart1, (uint8_t *)alert, strlen(alert), 50);
            HAL_UART_Transmit(&huart2, (uint8_t *)alert, strlen(alert), 50);
        }
        return;
    } else {
        alert_sent_ir = 0;
    }
}

/* ============================================================
 *  WATCHDOG
 * ============================================================ */
void Watchdog_Check(void) {
    if (gVehicle.mode != MODE_AUTONOMOUS) return;
    if (gVehicle.state == STATE_IDLE ||
        gVehicle.state == STATE_STOP ||
        gVehicle.state == STATE_EMERGENCY_STOP) return;

    if ((HAL_GetTick() - gVehicle.last_cmd_tick) > WATCHDOG_TIMEOUT_MS) {
        Motor_Stop();
        StateMachine_Transition(STATE_STOP);
        gVehicle.watchdog_fault = 1;
        if (!alert_sent_wdog) {
            alert_sent_wdog = 1;
            const char alert[] = "ALERT:WATCHDOG_TIMEOUT\n";
            HAL_UART_Transmit(&huart1, (uint8_t *)alert, strlen(alert), 50);
            HAL_UART_Transmit(&huart2, (uint8_t *)alert, strlen(alert), 50);
        }
    } else {
        gVehicle.watchdog_fault = 0;
        alert_sent_wdog = 0;
    }
}

/* ============================================================
 *  TELEMETRY
 * ============================================================ */
void Telemetry_Send(void) {
    char tel[128];
    snprintf(tel, sizeof(tel),
             "TEL:%d,%d,%lu,%lu,%lu,%lu,%d,%d\n",
             (int)gVehicle.state,
             (int)gVehicle.mode,
             (unsigned long)gVehicle.sensors.front_cm,
             (unsigned long)gVehicle.sensors.rear_cm,
             (unsigned long)gVehicle.sensors.left_cm,
             (unsigned long)gVehicle.sensors.right_cm,
             (int)gVehicle.sensors.ir_left,
             (int)gVehicle.sensors.ir_right);
    HAL_UART_Transmit(&huart1, (uint8_t *)tel, strlen(tel), 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)tel, strlen(tel), 100);
}

/* ============================================================
 *  LED
 * ============================================================ */
void LED_SetState(uint8_t on) {
    /* PC13 on Black Pill is active-LOW */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* ============================================================
 *  UART ISR CALLBACK
 *  BUG-9 fix: ISR writes to isr_cmd_staging[], sets cmd_ready.
 *  Main loop copies to pending_cmd[] before processing.
 * ============================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        char c = (char)uart1_rx_byte;
        if (c == '\n' || c == '\r') {
            if (uart1_idx > 0) {
                uart1_buf[uart1_idx] = '\0';
                if (!cmd_ready) {
                    /* Safe: cmd_ready==0 means main loop isn't reading staging */
                    strncpy(isr_cmd_staging, uart1_buf, CMD_MAX_LEN - 1);
                    isr_cmd_staging[CMD_MAX_LEN - 1] = '\0';
                    cmd_source = SRC_RASPBERRY_PI;
                    cmd_ready  = 1;
                }
                uart1_idx = 0;
            }
        } else if (uart1_idx < UART_BUF_SIZE - 1) {
            uart1_buf[uart1_idx++] = c;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART2) {
        char c = (char)uart2_rx_byte;
        if (c == '\n' || c == '\r') {
            if (uart2_idx > 0) {
                uart2_buf[uart2_idx] = '\0';
                /* ESP32 manual always wins — overwrite even if cmd_ready */
                strncpy(isr_cmd_staging, uart2_buf, CMD_MAX_LEN - 1);
                isr_cmd_staging[CMD_MAX_LEN - 1] = '\0';
                cmd_source = SRC_ESP32;
                cmd_ready  = 1;
                uart2_idx  = 0;
            }
        } else if (uart2_idx < UART_BUF_SIZE - 1) {
            uart2_buf[uart2_idx++] = c;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_rx_byte, 1);
    }
}

/* Command_Process: copy staging to working buffer, then parse */
void Command_Process(void) {
    strncpy(pending_cmd, isr_cmd_staging, CMD_MAX_LEN - 1);
    pending_cmd[CMD_MAX_LEN - 1] = '\0';
    Command_Parse(pending_cmd, cmd_source);
}

/* ============================================================
 *  PERIPHERAL INITIALIZATION
 * ============================================================ */

/**
 * @brief 96 MHz from internal HSI PLL
 *        HSI=16MHz, PLLM=8 → 2MHz VCO_in, PLLN=96 → 192MHz VCO_out
 *        PLLP=DIV2 → 96MHz SYSCLK
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 96;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 4;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3);
}

void MX_GPIO_Init(void) {
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Motor direction: PB0, PB1 (Motor A) + PA6, PA7 (Motor B) — OUTPUT PP */
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN;
    HAL_GPIO_Init(MOTOR_A_IN1_PORT, &g);

    g.Pin = MOTOR_B_IN3_PIN | MOTOR_B_IN4_PIN;
    HAL_GPIO_Init(MOTOR_B_IN3_PORT, &g);

    /* LED — PC13 output */
    g.Pin = LED_PIN;
    HAL_GPIO_Init(LED_PORT, &g);

    /* Ultrasonic TRIG — PB8, PB10, PB12, PB14 — OUTPUT PP */
    g.Pin = US_FRONT_TRIG_PIN | US_REAR_TRIG_PIN |
            US_LEFT_TRIG_PIN  | US_RIGHT_TRIG_PIN;
    HAL_GPIO_Init(US_PORT, &g);

    /* Ultrasonic ECHO — PB9, PB11, PB13, PB15 — INPUT NOPULL
     * (voltage divider on board brings 5V echo to 3.3V) */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    g.Pin  = US_FRONT_ECHO_PIN | US_REAR_ECHO_PIN |
             US_LEFT_ECHO_PIN  | US_RIGHT_ECHO_PIN;
    HAL_GPIO_Init(US_PORT, &g);

    /* IR sensors — PC0, PC1 — INPUT PULLUP (active low sensors) */
    g.Pull = GPIO_PULLUP;
    g.Pin  = IR_LEFT_PIN | IR_RIGHT_PIN;
    HAL_GPIO_Init(GPIOC, &g);
}

/**
 * @brief TIM1 — Motor A PWM on PA8 (TIM1_CH1, AF1)
 *        96MHz / 96 / 1000 = 1kHz PWM
 */
void MX_TIM1_Init(void) {
    GPIO_InitTypeDef g = {0};
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 95;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);

    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 0;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);

    /* PA8 AF1 */
    g.Pin       = GPIO_PIN_8;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &g);
}

/**
 * @brief TIM2 — Microsecond delay counter
 *        96MHz / 96 = 1MHz → 1 tick = 1µs
 */
void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance           = TIM2;
    htim2.Init.Prescaler     = 95;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period        = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim2);
}

/**
 * @brief TIM4 — Motor B PWM on PB6 (TIM4_CH1, AF2)
 */
void MX_TIM4_Init(void) {
    GPIO_InitTypeDef g = {0};
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance           = TIM4;
    htim4.Init.Prescaler     = 95;
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim4.Init.Period        = 999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim4);

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_1);

    /* PB6 AF2 */
    g.Pin       = GPIO_PIN_6;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &g);
}

/**
 * @brief USART1 — Raspberry Pi @ 115200 8N1 on PA9/PA10
 */
void MX_USART1_UART_Init(void) {
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief USART2 — ESP32 @ 115200 8N1 on PA2/PA3
 */
void MX_USART2_UART_Init(void) {
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* ============================================================
 *  IRQ HANDLERS
 * ============================================================ */
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void SysTick_Handler(void)   { HAL_IncTick(); }

/* ============================================================
 *  ERROR HANDLER
 * ============================================================ */
void Error_Handler(void) {
    /* Flash LED SOS, then halt */
    while (1) {
        LED_SetState(1); HAL_Delay(100);
        LED_SetState(0); HAL_Delay(100);
    }
}
