package ca.mcgill.ecse211.project.Localization;

import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.game.GameResources;
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
  private float[] leftSensorData = new float[GameResources.leftColorSensor.sampleSize()]; // array of sensor readings
  private float[] rightSensorData = new float[GameResources.rightColorSensor.sampleSize()]; // array of sensor readings

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
    GameResources.leftColorSensor.setCurrentMode("Red");
    GameResources.rightColorSensor.setCurrentMode("Red");

    // initiate the current readings of the sensor
    GameResources.leftColorSensor.fetchSample(leftSensorData, 0);
    GameResources.rightColorSensor.fetchSample(rightSensorData, 0);

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
    GameResources.leftColorSensor.setCurrentMode("Red");
    GameResources.rightColorSensor.setCurrentMode("Red");
    boolean left = false, right = false;
    float[] leftSensorData = new float[3];
    float[] rightSensorData = new float[3];
    Navigation.travelForward(GameResources.FORWARD_SPEED_NORMAL);
    while (!(left && right)) {
      GameResources.leftColorSensor.fetchSample(leftSensorData, 0);
      GameResources.rightColorSensor.fetchSample(rightSensorData, 0);
      currentLeftValue = leftSensorData[0] * 100;
      currentRightValue = rightSensorData[0] * 100;

      if (-currentLeftValue + lastLeftValue >= GameResources.DIFFERENTIAL_LINE_THRESHOLD) {
        left = true;
        GameResources.leftMotor.setSpeed(0);
      }
      if (-currentRightValue + lastRightValue >= GameResources.DIFFERENTIAL_LINE_THRESHOLD) {
        right = true;
        GameResources.rightMotor.setSpeed(0);
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
      GameResources.leftMotor.setSpeed(0);
    } else if (lines == 2) {
      // only right sensor detected
      GameResources.rightMotor.setSpeed(0);
    } else if (lines == 3) {
      Navigation.stopMotors();
      // Both lines detected or no lines detected.
      // Therefore, do nothing
    }
    if (GameResources.leftMotor.getSpeed() == 0 && GameResources.rightMotor.getSpeed() == 0) {
      // SET THE ODOMETER HERE
      return false;
    }

    if (endTime - startTime < GameResources.LIGHT_SENSOR_PERIOD) {
      // Sleep the correct amount of time
      try {
        Thread.sleep(GameResources.LIGHT_SENSOR_PERIOD - (endTime - startTime));
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
    GameResources.leftColorSensor.fetchSample(leftSensorData, 0);
    GameResources.rightColorSensor.fetchSample(rightSensorData, 0);
    float temporaryLeftColorValue = leftSensorData[0] * 100;
    float temporaryRightColorValue = rightSensorData[0] * 100;

    // set the current color value
    this.currentLeftColorValue = temporaryLeftColorValue;
    this.currentRightColorValue = temporaryRightColorValue;

    // Check if the difference between the previous reading and the current reading is bigger than the differential line
    // threshold
    float leftLineDifference = this.lastLeftColorValue - this.currentLeftColorValue;
    float rightLineDifference = this.lastRightColorValue - this.currentRightColorValue;


    if (leftLineDifference >= GameResources.DIFFERENTIAL_LINE_THRESHOLD && !leftCurrentlyOnLine) {
      // System.out.println("DIFF: " + (this.lastColorValue - this.currentColorValue));
      leftCurrentlyOnLine = true;
      line += 1;
    } else if (-leftLineDifference >= GameResources.DIFFERENTIAL_LINE_THRESHOLD) {
      leftCurrentlyOnLine = false;
    }
    if (rightLineDifference >= GameResources.DIFFERENTIAL_LINE_THRESHOLD && !rightCurrentlyOnLine) {

      rightCurrentlyOnLine = true;
      line += 2;
    } else if (-rightLineDifference >= GameResources.DIFFERENTIAL_LINE_THRESHOLD) {
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
   * Method used for general light localization when the robot is placed already approximately on the localize point.
   * The method checks if the localization point is near a wall. If not, the general procedure is performed: travels on
   * the y axis until both its sensors detect a line, adjust the y value and the theta of its odometer, travels on the x
   * axis until both its sensors detect a line and adjust the x value and the theta of its odometer. If the point is
   * near a wall: special cases are applied
   * 
   * @param: point used to set the x and y values of the odometer during light localization
   */
  public static void lightLocalize(Point point) {
    // check if the point is near a wall
    if ((point.x != 1) && (point.x != (GameResources.FIELD_RIGHT - 1)) && (point.y != 1)
        && (point.y != (GameResources.FIELD_TOP - 1))) {
      // GENERAL PROCEDURE
      Navigation.turnTo(0, GameResources.ROTATE_SPEED_FAST);
      LightLocalizer.twoLineDetection();
      Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
      GameResources.odometer.setXYT(GameResources.odometer.getX(), point.y * GameResources.TILE_SIZE, 0);
      Navigation.turnTo(90, GameResources.ROTATE_SPEED_FAST);
      LightLocalizer.twoLineDetection();
      GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) + GameResources.OFFSET_FROM_WHEELBASE,
          GameResources.odometer.getY(), 90);
    }
    // The point is near a wall --> SPECIAL CASES
    else {
      // close to top wall and right wall
      if (point.x == (GameResources.FIELD_RIGHT - 1) && point.y == (GameResources.FIELD_RIGHT - 1)) {
        Navigation.turnTo(180, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), point.y * GameResources.TILE_SIZE, 180);
        Navigation.turnTo(270, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) - GameResources.OFFSET_FROM_WHEELBASE,
            GameResources.odometer.getY(), 270);
      }

      // only close to right wall
      else if (point.x == (GameResources.FIELD_RIGHT - 1)) {
        Navigation.turnTo(0, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), point.y * GameResources.TILE_SIZE, 0);
        Navigation.turnTo(270, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) - GameResources.OFFSET_FROM_WHEELBASE,
            GameResources.odometer.getY(), 270);

      }
      // only close to top wall
      else if (point.y == (GameResources.currentTopLimit - 1)) {
        Navigation.turnTo(180, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), point.y * GameResources.TILE_SIZE, 0);
        Navigation.turnTo(90, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) + GameResources.OFFSET_FROM_WHEELBASE,
            GameResources.odometer.getY(), 90);
      }
      // either close to left wall or to bottom wall --> general procedure
      else if (point.x == 1 || point.y == 1) {
        Navigation.turnTo(0, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), point.y * GameResources.TILE_SIZE, 0);
        Navigation.turnTo(90, GameResources.ROTATE_SPEED_FAST);
        LightLocalizer.twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) + GameResources.OFFSET_FROM_WHEELBASE,
            GameResources.odometer.getY(), 90);
      }

    }

  }

  /**
   * Method used to perform the initial light localization in the starting corner. 4 different cases of initial
   * localization are implemented depending on which corner number is inputted.
   * 
   * @param: point is the starting corner point used to set the odometer x and y values properly
   * 
   * @param: corner is the starting corner number
   */
  public static void initialLightLocalize(Point point, int corner) {

    switch (corner) {
      case 0:
        Navigation.turnTo(0, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), (point.y * GameResources.TILE_SIZE), 0);
        Navigation.turnTo(90, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) + GameResources.OFFSET_FROM_WHEELBASE, GameResources.odometer.getY(), 90);
        break;
      case 1:
        Navigation.turnTo(0, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), (point.y * GameResources.TILE_SIZE), 0);
        Navigation.turnTo(270, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) - GameResources.OFFSET_FROM_WHEELBASE, GameResources.odometer.getY(), 270);
        break;
      case 2:
        Navigation.turnTo(180, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), (point.y * GameResources.TILE_SIZE), 180);
        Navigation.turnTo(270, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) - GameResources.OFFSET_FROM_WHEELBASE, GameResources.odometer.getY(), 270);
        break;
      case 3:
        Navigation.turnTo(180, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
        GameResources.odometer.setXYT(GameResources.odometer.getX(), (point.y * GameResources.TILE_SIZE), 180);
        Navigation.turnTo(90, GameResources.ROTATE_SPEED_FAST);
        twoLineDetection();
        GameResources.odometer.setXYT((point.x * GameResources.TILE_SIZE) + GameResources.OFFSET_FROM_WHEELBASE, GameResources.odometer.getY(), 90);
        break;
    }
  }


}
