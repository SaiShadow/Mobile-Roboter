
#define KP 0.5       // Proportional gain
int brightLeft = 0;
int brightCenter = 0;
int brightRight = 0;

// Function to handle line sensor trigger
void lineSensorTrigger(volatile uint32_t *adc_values) {
    brightLeft = adc_values[5];
    brightCenter = adc_values[0];
    brightRight = adc_values[2];
}
// Schmidt trigger function to determine sensor state
int getSchmidtValue(int brightness) {
    if (brightness <= SW_LOW) {
        return 0; // Sensor is off the line
    } else if (brightness >= SW_HIGH) {
        return 1; // Sensor is on the line
    } else {
        return -1; // Undefined state
    }
}
// P-Controller implementation
void pRegler() {
    int valLeft = getSchmidtValue(brightLeft);
    int valCenter = getSchmidtValue(brightCenter);
    int valRight = getSchmidtValue(brightRight);
    
    switch (valLeft + valCenter + valRight) {
    case 3: // All sensors on the line
        break;
    case 2: // One sensor out of bounds
        if (valLeft == 0) {
            // Left sensor is out of bounds
            driveLeft(speedLeft / 2, 1);
            driveRight(speedRight / 4, 1);
        } else if (valRight == 0) {
            // Right sensor is out of bounds
            driveLeft(speedLeft / 4, 1);
            driveRight(speedRight / 2, 1);
        } else if (valCenter == 0) {
            drive(0);
        }
        break;
    case 1: // Only one sensor in bounds
        if (valLeft == 1) {
            // Only left is in bounds
            driveLeft(speedLeft / 4, 1);
            driveRight(speedRight, 1);
        } else if (valRight == 1) {
            driveLeft(speedLeft, 1);
            driveRight(speedRight / 4, 1);
        }
        break;
    case 0: // No sensors on the line
        drive(0);
        lostPath();
        break;
    default:
        drive(0);
        break;
    }
}
