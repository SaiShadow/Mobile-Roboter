/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ERROR_SIZE 10

#define BLACK_THRESHOLD 1000


#define ENCODER_UPPER_BOUND_RIGHT 2500
#define ENCODER_LOWER_BOUND_RIGHT 1500

#define ENCODER_UPPER_BOUND_LEFT 3100
#define ENCODER_LOWER_BOUND_LEFT 2600

#define LINE_SENSOR_BLACK 3000

// Values at home
//#define LINE_SENSOR_LEFT_OFFSET 800
//#define LINE_SENSOR_MIDDLE_OFFSET 400
//#define LINE_SENSOR_RIGHT_OFFSET 1400

#define LINE_SENSOR_LEFT_OFFSET 900
#define LINE_SENSOR_MIDDLE_OFFSET 400
#define LINE_SENSOR_RIGHT_OFFSET 1500

#define MAX_SPEED 65536 // = 2^16

#define max(a, b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

typedef enum {
    LEFT, RIGHT
} directionEnum; // represents direction

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int speedLeft = 0;
static int speedRight = 0;

int EncoderTicksLeft = 0;
int EncoderTicksRight = 0;

static int EncoderStateLeft = 0;
static int EncoderStateRight = 0;

static uint64_t encoderTimeoutTicks = 0;
const int encoderDelay = 1;

volatile uint32_t adc[6];
uint32_t buffer[6];
volatile uint8_t conversion_done_flag = 1;

const int equalizerDelay = 5;
const int tickSpeedRatio = 300;
const int rightAttenuation = 7; // 65535 pwm = 58 Ticks/s => 1130 pwm = 1 Tick/s
static uint64_t rightCounter = 0;
static uint64_t equalizerTimeoutTicks = 0;

static uint64_t straightTimeoutTicks = 0;
const int straightDelay = 100;
const float centimetersPerTick = 0.52359f; // 4*PI / 24
const float millimetersPerTick = 5.2359f;

static int executedDriveStraight = 0;

const float degreesPerTick = 7.7922; //with wheel Distance of 7.7cm
const float attenuation = 1.f;
const float tolerance = 0.5f;

uint64_t turnTimeoutTicks = 0;
int turnDelay = 20;
int stopEncoder = 0;

static int executedTurnDegrees = 0;

static int ticksToTurn = 0;
static int executedTurnDegreesSingle = 0;

const float kp = 1 / 2;
const float ki = 0;
const float kd = 0;
int prevErrors[ERROR_SIZE] = {0};
int onlyMiddles[ERROR_SIZE] = {0};
static int prevMiddleCount = 0;
static int previError = 0;
static int errorIdx = 0;
static int errorThresh = 1500;

static int searchState = 0;
static directionEnum firstDirection = RIGHT;
const int middleThresh = 3;

static int avoidState = 0;

static int courseState = 0;

uint64_t followTimeoutTicks = 0;
int followDelay = 5;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Checks if robot is on line, that is if any sensor sees the line.
 *
 * @return 1 if robot is on line otherwise 0.
 */
int isOnLine() {
    int left = adc[5] - LINE_SENSOR_LEFT_OFFSET;
    int middle = adc[0] - LINE_SENSOR_MIDDLE_OFFSET;
    int right = adc[2] - LINE_SENSOR_RIGHT_OFFSET;

    int leftState = left < BLACK_THRESHOLD;
    int middleState = middle < BLACK_THRESHOLD;
    int rightState = right < BLACK_THRESHOLD;

    // if no sensor sees the line then return 0.
    return !(leftState && middleState && rightState);
}

/**
 * Function to convert the adc array to CSV format with "," as the delimiter.
 *
 * @param str must be large enough to convert the entire adc array.
 * @return Total length of the CSV data (including null terminator)
 */
uint16_t adcToCsv(char *str) {
    uint16_t len = 0;
    char buf[16];
    str[0] = '\0';
    for (int i = 0; i < 6; i++) {
        len += sprintf((char *) buf, "%lu", adc[i]) + 1;
        strcat(str, buf);
        strcat(str, ",");
    }
    str[len - 1] = '\n';
    return len;
}

/**
 * Function to help to print the encoder ticks to show in HTerm.
 *
 * @param str stores the output
 * @return Total length of the data (including null terminator)
 */
uint16_t EncoderTicksToString(char *str) {
    uint16_t len = 0;
    char buf[50];

    len += sprintf((char *) str, "Left Encoder Ticks: %d, ", EncoderTicksLeft);
    len += sprintf((char *) buf, "Right Encoder Ticks: %d, ", EncoderTicksRight);
    strcat(str, buf);
    len += sprintf((char *) buf, "%d,", EncoderTicksLeft / 24);
    strcat(str, buf);
    len += sprintf((char *) buf, "%d", EncoderTicksRight / 24);
    strcat(str, buf);
    str[len] = '\n';
    return len + 1;
}

/**
 * Function to help to print the encoder values to show in HTerm.
 *
 * @param str stores the output
 * @return Total length of the data (including null terminator)
 */
uint16_t EncoderToString(char *str) {
    uint16_t len = 0;
    char buf[50];

    len += sprintf((char *) str, "%lu,", adc[1]);
    len += sprintf((char *) buf, "%lu", adc[4]);
    strcat(str, buf);
    str[len] = '\n';
    return len + 1;
}

/**
 * Function to help to print the line sensor values to show in HTerm.
 *
 * @param str stores the output
 * @return Total length of the data (including null terminator)
 */
uint16_t LineSensorToString(char *str) {
    uint16_t len = 0;
    char buf[50];

    len += sprintf((char *) str, "%lu, ", adc[5]);
    len += sprintf((char *) buf, "%lu, ", adc[0]);
    strcat(str, buf);
    len += sprintf((char *) buf, "%lu", adc[2]);
    strcat(str, buf);
    str[len] = '\n';
    return len + 1;
}

/**
 * Function to help to print the line sensor states, i.e. white or black, to show in HTerm.
 *
 * @param str stores the output
 * @return Total length of the data (including null terminator)
 */
uint16_t LineSensorStateToString(char *str) {
    int ileft = adc[5] - LINE_SENSOR_LEFT_OFFSET;
    int imiddle = adc[0] - LINE_SENSOR_MIDDLE_OFFSET;
    int iright = adc[2] - LINE_SENSOR_RIGHT_OFFSET;

    char left = ileft < BLACK_THRESHOLD ? 'w' : 'b';
    char middle = imiddle < BLACK_THRESHOLD ? 'w' : 'b';
    char right = iright < BLACK_THRESHOLD ? 'w' : 'b';

    uint16_t len = 0;
    char buf[50];

    len += sprintf((char *) str, "%c, ", left);
    len += sprintf((char *) buf, "%c, ", middle);
    strcat(str, buf);
    len += sprintf((char *) buf, "%c, ", right);
    strcat(str, buf);
    len += sprintf((char *) buf, "%d, ", ileft);
    strcat(str, buf);
    len += sprintf((char *) buf, "%d, ", imiddle);
    strcat(str, buf);
    len += sprintf((char *) buf, "%d", iright);
    strcat(str, buf);
    str[len] = '\n';
    return len + 1;

}

/**
 * Function for motor control. Sets how fast the left wheel must turn.
 *
 * @param speed if negative then drive backwards, otherwise forwards. Must be in range of (-2^16, 2^16).
 */
void setSpeedLeft(int speed) {
    speedLeft = speed;
    if (speed == 0) { // brake / stop
        HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, 0);
        TIM1->CCR2 = 0;
        return;
    }
    if (speed > 0) { // forwards
        HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, 1);
        TIM1->CCR2 = MAX_SPEED - speed;
        return;
    }
    // backwards
    HAL_GPIO_WritePin(GPIOA, phase2_L_Pin, 0);
    TIM1->CCR2 = -speed;
}

/**
 * Function for motor control.
 * Sets how fast the right wheel must turn.
 *
 * @param speed if negative then drive backwards, otherwise forwards. Must be in range of (-2^16, 2^16).
 */
void setSpeedRight(int speed) {
    speedRight = speed;
    if (speed == 0) {
        HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, 0);
        TIM1->CCR3 = 0;
        return;
    }
    if (speed > 0) {
        HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, 0);
        TIM1->CCR3 = speed;
        return;
    }
    // backwards
    HAL_GPIO_WritePin(GPIOB, phase2_R_Pin, 1);
    TIM1->CCR3 = MAX_SPEED + speed;
}

/**
 * Sets speed for left and right tire with the same value.
 * @param speed see the individual methods for more info.
 */
void setSpeed(int speed) {
    setSpeedLeft(speed);
    setSpeedRight(speed);
}


// Kopiert aus Übungsblatt 3
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc1) {
    for (int i = 0; i < 6; i++) {
        adc[i] = buffer[i];
    }
    conversion_done_flag = 1;
}

/**
 * Process encoder ticks, by adding ticks to EncoderTicksLeft and EncoderTicksRight.
 */
void processEncoderTicks() {
    switch (EncoderStateLeft) {
        case 0:
            if (adc[1] >= ENCODER_UPPER_BOUND_LEFT) {
                EncoderTicksLeft++;
                EncoderStateLeft = 1;
            }
            break;
        case 1:
            if (adc[1] <= ENCODER_LOWER_BOUND_LEFT) {
                EncoderTicksLeft++;
                EncoderStateLeft = 0;
            }
            break;
    }
    switch (EncoderStateRight) {
        case 0:
            if (adc[4] >= ENCODER_UPPER_BOUND_RIGHT) {
                EncoderTicksRight++;
                EncoderStateRight = 1;
            }
            break;
        case 1:
            if (adc[4] <= ENCODER_LOWER_BOUND_RIGHT) {
                EncoderTicksRight++;
                EncoderStateRight = 0;
            }
            break;
    }
}

/**
 * Made the method @processEncoderTicks() cooperative Multitasking friendly (kooperatives
 * Multitasking) hence the name processEncoderTicksCoop.
 */
void processEncoderTicksCoop() {
    uint64_t currentTicks = HAL_GetTick();
    if (currentTicks - encoderTimeoutTicks < encoderDelay)
        return;
    encoderTimeoutTicks = currentTicks;
    processEncoderTicks();
}

/**
 * Implements "Positionsregelung der Motoren" from Versuch 6.
 * Since the left encoder sensor and wheel are not functional, movement checks are performed
 * solely using the right wheel. It is assumed that the left wheel drives straight without
 * checking.
 *
 * Previously, the program incorrectly increased the speed of the left wheel based on the
 * assumption that it wasn't rotating as much as the right wheel, which was not accurate.
 * So this change was needed.
 *
 * @param speed how fast the wheels are supposed to go
 */
void regulateSpeed(int speed) {
    int difference = EncoderTicksRight - EncoderTicksLeft;
    int correction = difference * tickSpeedRatio;

    rightCounter++;

    // Set new speeds.
    setSpeedLeft(speed + correction);
    setSpeedRight(speed - correction);
}

/**
 * Made the method @regulateSpeed cooperative Multitasking friendly.
 *
 * @param speed how fast the motors are supposed to turn.
 */
void regulateSpeedCoop(int speed) {
    uint64_t ticks = HAL_GetTick();
    if (ticks - equalizerTimeoutTicks < equalizerDelay)
        return;
    equalizerTimeoutTicks = ticks;
    regulateSpeed(speed);
}

/**
 * Check how much you traveled in a straight line while considering only the right wheel
 * encoder ticks.
 * Multiply this by how many mms you get per tick and check if you traveled more or
 * equal to the distance.
 *
 * @param distance amount needed to travel
 * @return 1 if travelled at least distance amount, otherwise 0.
 */
int checkDistanceTraveledStraight(int distance) {
    return (((float) (EncoderTicksRight)) * millimetersPerTick
            >= (float) distance);
}

/**
 * Makes the robot drive straight with a speed for the specified distance and then stops.
 *
 * @param distance amount the robot needs to drive in millimeters.
 * @param speed how fast the robot has to drive/move, i.e. how fast the motors need to
 * rotate.
 * @return 1 if successfully drove distance, otherwise 0.
 */
int driveStraight(int distance, int speed) {
    if (!executedDriveStraight) {
        EncoderTicksLeft = 0;
        EncoderTicksRight = 0;
        setSpeed(speed);
        executedDriveStraight = 1;
    }

    if (!checkDistanceTraveledStraight(distance))
        // didn't travel enough distance
        return 0;

    setSpeed(0);
    return 1;
}

/**
 * Made the method @regulateSpeed cooperative Multitasking friendly.
 *
 * @param distance
 * @param speed
 * @return
 */
int driveStraightCoop(int distance, int speed) {
    uint64_t ticks = HAL_GetTick();
    if (ticks - straightTimeoutTicks < straightDelay)
        return 0;

    straightTimeoutTicks = ticks;
    return driveStraight(distance, speed);
}

/**
 * Check how many degrees you already turned.
 * If turned enough then brake.
 *
 * Don't check left, because left sensor is not working for some reason, do all
 * movement checks based on right wheel
 *
 * @param ticksPerTurn represents how many encoder ticks are needed to turn the certain
 * amount of degrees.
 * @param leftBias not needed
 * @param rightBias adds extra ticks to turn if it motor doesn't turn enough.
 * @return 1 and brake if turned enough, otherwise 0.
 */
int checkAngleTurned(int ticksPerTurn, int leftBias, int rightBias) {
    // Don't check left, because left sensor is not working for some reason, do all
    // movement checks based on right wheel
    if (EncoderTicksRight >= ticksPerTurn + rightBias) {
        setSpeed(0);
        return 1;
    }
    return 0;
}

/**
 * Turns the robot by the specified angle.
 *
 * @param degrees       Angle to turn in degrees.
 * @param leftBias      Left bias for checking angle turned.
 * @param rightBias     Right bias for checking angle turned.
 * @param direction     Direction of turn (LEFT or RIGHT).
 * @param speed         Speed of the turn.
 * @return              Returns 1 when the turn is completed, 0 otherwise.
 */
int turn(float degrees, int leftBias, int rightBias,
         directionEnum direction, int speed) {
    static int ticksPerTurn = 0;

    if (!executedTurnDegrees) {
        EncoderTicksLeft = 0;
        EncoderTicksRight = 0;
        switch (direction) {
            case LEFT:
                setSpeedLeft(-speed);
                setSpeedRight(speed);
                break;
            case RIGHT:
                setSpeedLeft(speed);
                setSpeedRight(-speed);
                break;
        }

        // Calculate how many ticks are needed to turn the specified amount of degrees.
        ticksPerTurn = (int) ceil(degrees / degreesPerTick);
        executedTurnDegrees = 1;
    }
    if (!checkAngleTurned(ticksPerTurn, leftBias, rightBias)) {
        return 0;
    }
    setSpeed(0);
    return 1;
}

/**
 * Made the method @turn cooperative Multitasking friendly.
 *
 * @param degrees
 * @param leftBias
 * @param rightBias
 * @param direction
 * @param speed
 * @return
 */
int turnCoop(float degrees, int leftBias, int rightBias,
             directionEnum direction, int speed) {
    uint64_t ticks = HAL_GetTick();
    if (ticks - turnTimeoutTicks < turnDelay)
        return 0;
    turnTimeoutTicks = ticks;

    return turn(degrees, leftBias, rightBias, direction, speed);
}

/**
 * Turns only one wheel instead of both wheels as in @turn.
 * This method has complications as left encoder isn't working.
 *
 * @param degrees
 * @param leftBias
 * @param rightBias
 * @param direction
 * @param speed
 * @return
 */
int turnOneWheel(float degrees, int leftBias, int rightBias,
                 directionEnum direction, int speed) {
    uint64_t ticks = HAL_GetTick();
    if (ticks - turnTimeoutTicks < turnDelay)
        return 0;
    turnTimeoutTicks = ticks;

    if (!executedTurnDegreesSingle) {
        EncoderTicksLeft = 0;
        EncoderTicksRight = 0;
        switch (direction) {
            case LEFT:
                setSpeedRight(speed);
                break;
            case RIGHT:
                setSpeedLeft(speed);
                break;
        }
        ticksToTurn = 2 * (int) ceil(degrees / degreesPerTick);
        executedTurnDegreesSingle = 1;
    }

    if ((EncoderTicksLeft >= ticksToTurn + leftBias)
        || (EncoderTicksRight >= ticksToTurn + rightBias)) {
        setSpeed(0);
        executedTurnDegreesSingle = 0;
        return 1;
    }
    return 0;
}

/**
 * Hard codes the yellow part of the parkour.
 *
 * @param speed
 * @return 1 if succesfully completed, otherwise 0.
 */
int drivePreprogrammedRouteCoop(int speed) {
    static int step = 1;
    switch (step) {
        case 1:
            // Drive straight for 500mm
            if (!driveStraightCoop(500, speed))
                return 0;
            step++;
            executedDriveStraight = 0;
            break;
        case 2:
            // Turn 150 degrees
            if (!turnCoop(140.f, 0, 0, RIGHT, speed))
                return 0;
            step++;
            executedTurnDegrees = 0;
            break;
        case 3:
            // Drive straight for 443mm
            if (!driveStraightCoop(443, speed))
                return 0;
            step++;
            executedDriveStraight = 0;
            break;
        case 4:
            // Turn 90 degrees
            if (!turnCoop(110.f, 0, 0, LEFT, speed))
                return 0;
            step++;
            executedTurnDegrees = 0;
            break;
        case 5:
            // Drive stright for 245mm
            if (!driveStraightCoop(245, speed))
                return 0;
            step++;
            executedDriveStraight = 0;
            break;
        case 6:
            // Completed
            return 1;
    }
    // Something wrong happened
    return 0;
}

/**
 * Implemented a PID Regler for the follow line, but only used P regler (the other k values are set to 0)
 * due to multiple complications.
 *
 * @param speed
 * @return 0 when no line was found, otherwise 1.
 */
int followLine(int speed) {
    int left = adc[5] - LINE_SENSOR_LEFT_OFFSET;
    int middle = adc[0] - LINE_SENSOR_MIDDLE_OFFSET;
    int right = adc[2] - LINE_SENSOR_RIGHT_OFFSET;

    int leftState = left < BLACK_THRESHOLD;
    int middleState = middle < BLACK_THRESHOLD;
    int rightState = right < BLACK_THRESHOLD;

    if (leftState && middleState && rightState && (errorIdx == 0))
        return 0;

    // Falls alle Werte ~ 0 sind, dann findTrack()

    int pError = left - right;

    // Integral error calculation
    int iError = previError + pError - prevErrors[errorIdx];

    // Update the previous error
    previError = iError;

    // Store the current proportional error
    prevErrors[errorIdx] = pError;

    int currentMiddle = (!middleState && leftState && rightState);
    prevMiddleCount = prevMiddleCount - onlyMiddles[errorIdx] + currentMiddle;
    onlyMiddles[errorIdx] = currentMiddle;

    // Update the error index using modulo to wrap around
    errorIdx = (errorIdx + 1) % ERROR_SIZE;

    // Derivative error calculation
    int prevErrorIdx = (errorIdx == 0) ? (ERROR_SIZE - 1) : (errorIdx - 1);
    int dError = pError - prevErrors[prevErrorIdx];

    // Calculate the correction
    int correction = (int) (kp * (float) pError) + (int) (ki * (float) iError)
                     + (int) (kd * (float) dError);

    // Set motor speeds
    if (pError >= errorThresh) {
        setSpeedLeft(0);
        setSpeedRight(speed + correction);
    } else if (pError <= -errorThresh) {
        setSpeedLeft(speed - correction);
        setSpeedRight(0);

    } else {
        setSpeedLeft(speed - correction);
        setSpeedRight(speed + correction);
    }
    return 1;
}

/**
 * Implements the execution of the robot if line is lost.
 * Searches for line if lost. If middle sensor is the last sensor to see the line then assumes that there is a
 * gap and goes to overcomeGap() Protocol.
 * Otherwise, turn left and right depending on which sensor saw the line last.
 *
 * @param speed
 * @return 1 if line is found, then brake/stop and return. Otherwise 0.
 */
int searchLine(int speed) {
    switch (searchState) {
        case 0:
            setSpeed(0); // brake

            if (prevMiddleCount > middleThresh) { // if the middle sensor was the last sensor to see the line then assume
                // that you reached the Gap and move on to overcomeGap() part in searchState 5.
                searchState = 5;
                break;
            }
            // Start with searching for the line now.
            firstDirection = previError > 0 ? LEFT : RIGHT;
            searchState = 1;
            break;
        case 1:
            // While turning 100 degrees, search for line.
            if (!turnOneWheel(100.f, 0, 0, firstDirection, speed)) {
                if (isOnLine()) {
                    setSpeed(0);
                    executedTurnDegreesSingle = 0;
                    return 1;
                }
                return 0;
            }
            searchState++;
            break;
        case 2:
            // Turn back to original position.
            if (!turnOneWheel(100.f, 0, 0, firstDirection, -speed)) {
                return 0;
            }
            searchState++;
            break;
        case 3:
            // While turning 100 degrees in the other direction, search for line.
            if (!turnOneWheel(100.f, 0, 0, (firstDirection + 1) % 2, speed)) {
                if (isOnLine()) {
                    setSpeed(0);
                    executedTurnDegreesSingle = 0;
                    return 1;
                }
                return 0;
            }
            searchState++;
            break;
        case 4:
            // Turn back to original position.
            if (!turnOneWheel(100.f, 0, 0, (firstDirection + 1) % 2, -speed)) {
                return 0;
            }
            searchState++;
            break;

        case 5:
            // Drive straight.
            if (!driveStraight(100, speed)) {
                if (isOnLine()) {
                    setSpeed(0);
                    executedDriveStraight = 0;
                    return 1;
                }
                return 0;
            }
            executedDriveStraight = 0;
            searchState++;
            break;
        default:
            return 1;
    }
    return 0;
}

/**
 * Checks if robot hits an obstacle, by checking if any of the buttons/taster get hit by the obstacle.
 *
 * @return 0 if right and middle button gets hit by obstacle, otherwise 1.
 */
int touchesButton() {
    if (!HAL_GPIO_ReadPin(GPIOA, switch_left_Pin))
        return 1;
    if (!HAL_GPIO_ReadPin(GPIOA, switch_middle_Pin))
        return 0;
    if (!HAL_GPIO_ReadPin(GPIOA, switch_right_Pin))
        return 0;
    else
        return 2;
}

/**
 * If robot encounters an obstacle, execute this pre-programmed route to avoid and go around the obstacle.
 *
 * @param direction which way it should turn first, depends on which button the obstacle hit.
 * @param speed how fast it should go around the obstacle
 * @return 1 if completed successfully otherwise 0.
 */
int avoidObstacle(directionEnum direction, int speed) {
    int turnSpeed = 50000;
    switch (avoidState) {
        case 0:
            // Drive backwards
            if (!driveStraight(10, -speed)) {
                return 0;
            }
            executedDriveStraight = 0;
            avoidState++;
            break;
        case 1:
            // Turn 50 degrees
            if (!turn(50.f, 0, 0, direction, turnSpeed))
                return 0;

            executedTurnDegrees = 0;
            avoidState++;
            break;
        case 2:
            // Drive straight
            if (!driveStraight(160, speed)) {
                return 0;
            }
            executedDriveStraight = 0;
            avoidState++;
            direction = (direction + 1) % 2;
            break;
        case 3:
            // turn
            if (!turn(50.f, 0, 0, direction, turnSpeed)) {
                return 0;
            }
            executedTurnDegrees = 0;
            avoidState++;
            break;
        case 4:
            // drive straight
            if (!driveStraight(100, speed)) {
                return 0;
            }
            executedDriveStraight = 0;
            avoidState++;
            break;

        case 5:
            // turn
            if (!turn(50.f, 0, 0, direction, turnSpeed)) {
                return 0;
            }
            executedTurnDegrees = 0;
            avoidState++;
            break;

        case 6:
            // drive straight and check if you find line
            if (!driveStraight(200, speed)) {
                if (isOnLine()) {
                    setSpeed(0);
                    executedDriveStraight = 0;
                    return 1;
                }
                return 0;
            }
            executedDriveStraight = 0;
            avoidState++;
            break;
        case 7:
            return 1;
    }
    return 0;
}

/**
 * Made the method @followLine cooperative Multitasking friendly.
 * @param speed
 */
void followLineCoop(int speed) {
    uint64_t ticks = HAL_GetTick();
    if (ticks - followTimeoutTicks < followDelay)
        return;
    turnTimeoutTicks = ticks;
    followLine(speed);
}

/**
 * Implements task_followLine() as seen in Versuch 8.
 * This controls every state after the yellow part of the parkour.
 *
 * @param speed
 */
void taskFollowLine(int speed) {
    static int followState = 0;

    int touchState = touchesButton();
    static directionEnum direction = LEFT;
    if (touchState != 2) {
        followState = 2;
        direction = touchState;

    }
    switch (followState) {
        case 0:
            if (!followLine(speed))
                followState++;
            break;
        case 1:
            // if line is lost
            if (searchLine(speed) == 0)
                break;
            else {
                followState = 0;
                searchState = 0;
            }
            break;
        case 2:
            // if button gets hit by obstacle
            if (!avoidObstacle(direction, speed))
                break;
            followState = 0;
            avoidState = 0;
            break;
    }
}


/**
 * Method drives the parkour course.
 *
 * @param speed
 */
void driveCourse(int speed) {

    switch (courseState) {
        case 0:
            // yellow part of parkour
            if (!drivePreprogrammedRouteCoop(40000))
                break;
            courseState++;
            break;
        case 1:
            // follow the lines and other problems
            taskFollowLine(speed);
            break;
    }
}

/**
 * Run this if left button is pressed at start.
 * Used to find out different metrics on HTerm.
 */
void leftButton() {
    processEncoderTicks();
    setSpeed(30000);
    char sendbuf[500];
    uint16_t size = EncoderToString(sendbuf);
    HAL_UART_Transmit(&huart2, (uint8_t *) sendbuf, size, 10000);
    HAL_Delay(2);
}

/**
 * Run this if middle button is pressed at start.
 * This is the main method to start driving the parkour.
 * Left LED lights up if the robot is on line.
 */
void middleButton() {
    processEncoderTicks();
    driveCourse(60000);
    HAL_GPIO_WritePin(GPIOB, LED_left_Pin, isOnLine());
}

/**
 * Run this if left button is pressed at start.
 * This doesn't contain the yellow part, used for testing.
 * Left LED lights up if the robot is on line.
 */
void rightButton() {
    processEncoderTicks();
    taskFollowLine(60000);
    HAL_GPIO_WritePin(GPIOB, LED_left_Pin, isOnLine());
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    void (*taskPtr)(void) = 0;

    while (1) {
        if (!HAL_GPIO_ReadPin(GPIOA, switch_left_Pin)) {
            taskPtr = &leftButton;
            break;
        }
        if (!HAL_GPIO_ReadPin(GPIOA, switch_middle_Pin)) {
            // Main method for parkour
            taskPtr = &middleButton;
            break;
        }
        if (!HAL_GPIO_ReadPin(GPIOA, switch_right_Pin)) {
            taskPtr = &rightButton;
            break;
        }
    }
    // waits 1,5sec before running the wanted program.
    HAL_Delay(1500);

    // Keep running the selected program.
    while (1) {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (conversion_done_flag) {
            conversion_done_flag = 0;
            HAL_ADC_Start_DMA(&hadc1, buffer, 6);
        }
        taskPtr();
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
        != HAL_OK) {
        Error_Handler();
    }

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
                                       | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 16;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    /** Enable MSI Auto calibration
     */
    HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
