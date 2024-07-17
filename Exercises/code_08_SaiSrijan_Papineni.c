
#define ENCODER_UPPER_BOUND 2500
#define ENCODER_LOWER_BOUND 1500

// weißwerte der Sensoren aus tests
#define LINE_SENSOR_LEFT_OFFSET 1200;
#define LINE_SENSOR_MIDDLE_OFFSET 2000;
#define LINE_SENSOR_RIGHT_OFFSET 1000;

#define LINE_SENSOR_BLACK_THRESH 1000;

typedef enum { LEFT, RIGHT } directionEnum;
static int speedLeft = 0;
static int speedRight = 0;

uint64_t EncoderTicksLeft = 0;
uint64_t EncoderTicksRight = 0;

static int EncoderStateLeft = 0;
static int EncoderStateRight = 0;

int isOnLine() {
  int left = adc[5] - LINE_SENSOR_LEFT_OFFSET;
  int middle = adc[0] - LINE_SENSOR_MIDDLE_OFFSET;
  int right = adc[2] - LINE_SENSOR_RIGHT_OFFSET;

  int leftState = left < LINE_SENSOR_BLACK_THRESH;
  int middleState = middle < LINE_SENSOR_BLACK_THRESH;
  int rightState = right < LINE_SENSOR_BLACK_THRESH;

  return !(leftState && middleState && rightState);
}

// Aufgabe 2.1
static int searchState = 0;
int searchLine(int speed) {
  switch (searchState) {
  case 0:
    setSpeed(0);
    searchState = 1;
    break;
  case 1:

    if (!turnDegreesSingle(100.f, 0, 0, RIGHT, speed)) {
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

    if (!turnDegreesSingle(100.f, 0, 0, RIGHT, -speed)) {
      return 0;
    }
    searchState++;
    break;
  case 3:

    if (!turnDegreesSingle(100.f, 0, 0, LEFT, speed)) {
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

    if (!turnDegreesSingle(100.f, 0, 0, LEFT, -speed)) {
      return 0;
    }
    searchState++;
    break;

  case 5:
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

// Aufgabe 3
static int avoidState = 0;
int avoidObstacle(directionEnum direction, int speed) {

  switch (avoidState) {
  case 0:
    if (!driveStraight(10, -speed)) {
      return 0;
    }
    executedDriveStraight = 0;
    avoidState++;
    break;
  case 1:
    if (!turnDegrees(45.f, 0, 0, direction, speed)) {
      return 0;
    }
    executedTurnDegrees = 0;
    avoidState++;
    break;
  case 2:
    if (!driveStraight(160, speed)) {
      return 0;
    }
    executedDriveStraight = 0;
    avoidState++;
    break;
  case 3:
    if (!turnDegrees(90.f, 0, 0, (direction + 1) % 2, speed)) {
      return 0;
    }
    executedTurnDegrees = 0;
    avoidState++;
    break;
  case 4:
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
  }
  return 0;
}

int touchesSensor() {
  if (!HAL_GPIO_ReadPin(GPIOA, switch_left_Pin))
    return 1;
  if (!HAL_GPIO_ReadPin(GPIOA, switch_middle_Pin))
    return 0;
  if (!HAL_GPIO_ReadPin(GPIOA, switch_right_Pin))
    return 0;
  else
    return 2;
}

int main(void) {
  while (1) {
    // Verarbeite Rad Encoder Werte hier

    // Hier eigentlich Übergeordneter Zustandsautomat
    int touchState = touchesSensor();
    static directionEnum direction = LEFT;
    if (touchState != 2) {
      direction = touchState;
    }
    if (touchState)
      avoidObstacle(directionEnum direction, 40000);
    else if (!isOnLine())
      searchLine(40000);
    else
    // Fahre Linie Hinterher
  }
