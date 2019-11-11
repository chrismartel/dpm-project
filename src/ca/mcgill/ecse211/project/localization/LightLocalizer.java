package ca.mcgill.ecse211.project.localization;

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
   * coordinates to reach with light localization
   */
  private Point coordinates;


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
   * Returns the LightLocalizer Object. Use this method to obtain an instance of LightLocalizer. 
   * Method used to make sure there is just one instance of LightLocalizer throughout the code
   * 
   * @return the LightLocalizer Object
   */
  public synchronized static LightLocalizer getLightLocalizer() {
    if (loc == null) {
      loc = new LightLocalizer();
    }

    return loc;
  }

  


  public boolean lightLocalize() {
    
    // Perform full rotation to record angle values at each black line
//    Navigation.rotate(Turn.COUNTER_CLOCK_WISE, ROTATE_SPEED_SLOW);
    
      int lines = lineDetected();
      if(lines == 1) {
        //only left sensor detected
//        leftValues = odometer.getXYT();
        leftMotor.setSpeed(0);
      }
      else if (lines == 2) {
        //only right sensor detected
        rightMotor.setSpeed(0);
//        rightValues = odometer.sgetXYT();
      }
      else if (lines == 3){
        Navigation.stopMotors();
        // Both lines detected or no lines detected. 
        // Therefore, do nothing
      }
      
      if(leftMotor.getSpeed() == 0 && rightMotor.getSpeed() == 0) {
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
   * @return : 0 if no line has been detected, 1 if left sensor detected a line, 2 if right sensor detected a line
   * and 3 if both sensors detected a line.
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
    }
    else if(-leftLineDifference >= DIFFERENTIAL_LINE_THRESHOLD) {
      leftCurrentlyOnLine = false;
    }
    if(rightLineDifference >= DIFFERENTIAL_LINE_THRESHOLD && !rightCurrentlyOnLine) {

      rightCurrentlyOnLine = true;
      line += 2;
    }
    else if(-rightLineDifference >= DIFFERENTIAL_LINE_THRESHOLD) {
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
   * getter and setter for the goal coordinates of the light localization [x, y]
   */
  public void setCoordinates(Point coordinates) {
    this.coordinates = coordinates;
  }

  public Point getCoordinates() {
    return coordinates;
  }
}
