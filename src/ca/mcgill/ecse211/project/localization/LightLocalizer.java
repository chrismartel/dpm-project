package ca.mcgill.ecse211.project.Localization;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.game.Navigation;

/**
 * This class implements the methods used to execute the light localization using 2 sensors at the back of the robot
 */
public class LightLocalizer {

  /**
   * last and current sensor color readings used for comparison in each line detection loop
   */
  private float currentLeftColorValue;
  private float lastLeftColorValue;

  private float currentRightColorValue;
  private float lastRightColorValue;

  // Variables and values to operate color sensor
  private float[] leftSensorData = new float[leftColorSensor.sampleSize()]; // array of sensor readings
  private float[] rightSensorData = new float[rightColorSensor.sampleSize()]; // array of sensor readings

  /**
   * Booleans indicating if either sensor is currently detecting a line.
   */

  boolean leftCurrentlyOnLine = false;
  boolean rightCurrentlyOnLine = false;

  /**
   * Start and end times to accurately time our correction.
   */
  long startTime;
  long endTime;

  double[] leftValues = null;
  double[] rightValues = null;

  private static LightLocalizer loc; // Returned as singleton



  /**
   * Constructor of the light localization class
   */
  public LightLocalizer() {

    // set sensor to use red light alone
    leftColorSensor.setCurrentMode("Red");
    rightColorSensor.setCurrentMode("Red");

    // initiate the current readings of the sensor
    leftColorSensor.fetchSample(leftSensorData, 0);
    rightColorSensor.fetchSample(rightSensorData, 0);

    // initialize the sensor readings
    currentLeftColorValue = (leftSensorData[0] * 100);
    lastLeftColorValue = currentLeftColorValue;

    currentRightColorValue = (rightSensorData[0] * 100);
    lastRightColorValue = currentRightColorValue;
  }


  /**
   * Returns the LightLocalizer Object. Use this method to obtain an instance of LightLocalizer. Method used to make
   * sure there is just one instance of LightLocalizer throughout the code
   * 
   * @return the LightLocalizer Object
   */
  public synchronized static LightLocalizer getLightLocalizer() {
    if (loc == null) {
      loc = new LightLocalizer();
    }

    return loc;
  }

  /**
   * Method in which the robot travels forward until one of its 2 light sensors detects a line. When a sensor detects a
   * line, the corresponding motor stops, and the other motor turns until its light sensor also detects a line. When 2
   * lines are detected, the robot stops.
   */
  public static void twoLineDetection() {
    float lastLeftValue = -1000;
    float currentLeftValue = 0;
    float lastRightValue = -1000;
    float currentRightValue = 0;
    leftColorSensor.setCurrentMode("Red");
    rightColorSensor.setCurrentMode("Red");
    boolean left = false, right = false;
    float[] leftSensorData = new float[3];
    float[] rightSensorData = new float[3];
    Navigation.travelForward(FORWARD_SPEED_NORMAL);
    while (!(left && right)) {
      leftColorSensor.fetchSample(leftSensorData, 0);
      rightColorSensor.fetchSample(rightSensorData, 0);
      currentLeftValue = leftSensorData[0] * 100;
      currentRightValue = rightSensorData[0] * 100;

      if (-currentLeftValue + lastLeftValue >= DIFFERENTIAL_LINE_THRESHOLD) {
        left = true;
        leftMotor.setSpeed(0);
      }
      if (-currentRightValue + lastRightValue >= DIFFERENTIAL_LINE_THRESHOLD) {
        right = true;
        rightMotor.setSpeed(0);
      }
      lastLeftValue = currentLeftValue;
      lastRightValue = currentRightValue;
      try {
        Thread.sleep(75);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

    }
  }

  /**
   * Method that checks if lines are detected and stops the corresponding motors accordingly
   * 
   * @return : false if both motors are stopped, true if 0 or 1 motor is stopped
   */
  public boolean lightLocalize() {
    int lines = lineDetected();
    if (lines == 1) {
      // only left sensor detected
      leftMotor.setSpeed(0);
    } else if (lines == 2) {
      // only right sensor detected
      rightMotor.setSpeed(0);
    } else if (lines == 3) {
      Navigation.stopMotors();
      // Both lines detected or no lines detected.
      // Therefore, do nothing
    }
    if (leftMotor.getSpeed() == 0 && rightMotor.getSpeed() == 0) {
      // SET THE ODOMETER HERE
      return false;
    }

    if (endTime - startTime < LIGHT_SENSOR_PERIOD) {
      // Sleep the correct amount of time
      try {
        Thread.sleep(LIGHT_SENSOR_PERIOD - (endTime - startTime));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    return true;
  }



  /**
   * Method determining if a line is detected
   * 
   * @return : 0 if no line has been detected, 1 if left sensor detected a line, 2 if right sensor detected a line and 3
   *         if both sensors detected a line.
   */
  public int lineDetected() {

    // keep track of the execution time of the method to adjust the period
    startTime = System.currentTimeMillis();
    // no line detected initially
    int line = 0;
    // fetch sample from the light sensor
    leftColorSensor.fetchSample(leftSensorData, 0);
    rightColorSensor.fetchSample(rightSensorData, 0);
    float temporaryLeftColorValue = leftSensorData[0] * 100;
    float temporaryRightColorValue = rightSensorData[0] * 100;

    // set the current color value
    this.currentLeftColorValue = temporaryLeftColorValue;
    this.currentRightColorValue = temporaryRightColorValue;

    // Check if the difference between the previous reading and the current reading is bigger than the differential line
    // threshold
    float leftLineDifference = this.lastLeftColorValue - this.currentLeftColorValue;
    float rightLineDifference = this.lastRightColorValue - this.currentRightColorValue;


    if (leftLineDifference >= DIFFERENTIAL_LINE_THRESHOLD && !leftCurrentlyOnLine) {
      // System.out.println("DIFF: " + (this.lastColorValue - this.currentColorValue));
      leftCurrentlyOnLine = true;
      line += 1;
    } else if (-leftLineDifference >= DIFFERENTIAL_LINE_THRESHOLD) {
      leftCurrentlyOnLine = false;
    }
    if (rightLineDifference >= DIFFERENTIAL_LINE_THRESHOLD && !rightCurrentlyOnLine) {

      rightCurrentlyOnLine = true;
      line += 2;
    } else if (-rightLineDifference >= DIFFERENTIAL_LINE_THRESHOLD) {
      rightCurrentlyOnLine = false;
    }

    // update the last reading of the light sensor with the current one
    lastLeftColorValue = currentLeftColorValue;
    lastRightColorValue = currentRightColorValue;
    // pause the thread to respect the line detection period
    endTime = System.currentTimeMillis();
    return line;
  }

  /**
   * Method used to light localize when the robot is already approximately on the localize point
   */
  public static void lightLocalize(Point point) {
    if ((point.x > (currentLeftLimit + 1)) && (point.x < (currentRightLimit - 1))
        && (point.y > (currentBottomLimit + 1)) && (point.y < (currentTopLimit - 1))) {
      Navigation.turnTo(0, ROTATE_SPEED_FAST);
      LightLocalizer.twoLineDetection();
      Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
      odometer.setXYT(odometer.getX(), point.y * TILE_SIZE, 0);
      Navigation.turnTo(90, ROTATE_SPEED_FAST);
      LightLocalizer.twoLineDetection();
      odometer.setXYT((point.x * TILE_SIZE) + OFFSET_FROM_WHEELBASE, odometer.getY(), 90);
    } else {
      // close to top limit and right limit
      if (point.x == (currentRightLimit - 1) && point.y == (currentTopLimit - 1)) {
        Navigation.turnTo(180, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), point.y * TILE_SIZE, 0);
        Navigation.turnTo(270, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) - OFFSET_FROM_WHEELBASE, odometer.getY(), 90);
      }
      // either close to left limit or to bottom limit
      else if (point.x == (currentLeftLimit + 1) || point.y == (currentBottomLimit + 1)) {
        Navigation.turnTo(0, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), point.y * TILE_SIZE, 0);
        Navigation.turnTo(90, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) + OFFSET_FROM_WHEELBASE, odometer.getY(), 90);
      }
      // close to right limit
      else if (point.x == (currentRightLimit - 1)) {
        Navigation.turnTo(0, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), point.y * TILE_SIZE, 0);
        Navigation.turnTo(270, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) - OFFSET_FROM_WHEELBASE, odometer.getY(), 270);

      }
      // close to top limit
      else if (point.y == (currentTopLimit - 1)) {
        Navigation.turnTo(180, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), point.y * TILE_SIZE, 0);
        Navigation.turnTo(90, ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) + OFFSET_FROM_WHEELBASE, odometer.getY(), 90);
      }
    }

  }

  /**
   * Method used to light localize after the us localization, different light localization depending on which corner we
   * are localizing on
   */
  public static void initialLightLocalize(Point point, int corner) {

    switch (corner) {
      case 0:
        Navigation.turnTo(0, ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), (point.y * TILE_SIZE), 0);
        Navigation.turnTo(90, ROTATE_SPEED_FAST);
        twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) + OFFSET_FROM_WHEELBASE, odometer.getY(), 90);
        break;
      case 1:
        Navigation.turnTo(0, ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), (point.y * TILE_SIZE), 0);
        Navigation.turnTo(270, ROTATE_SPEED_FAST);
        twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) - OFFSET_FROM_WHEELBASE, odometer.getY(), 270);
        break;
      case 2:
        Navigation.turnTo(180, ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), (point.y * TILE_SIZE), 180);
        Navigation.turnTo(270, ROTATE_SPEED_FAST);
        twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) - OFFSET_FROM_WHEELBASE, odometer.getY(), 270);
        break;
      case 3:
        Navigation.turnTo(180, ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
        odometer.setXYT(odometer.getX(), (point.y * TILE_SIZE), 180);
        Navigation.turnTo(90, ROTATE_SPEED_FAST);
        twoLineDetection();
        odometer.setXYT((point.x * TILE_SIZE) + OFFSET_FROM_WHEELBASE, odometer.getY(), 90);
        break;
    }
  }


}
