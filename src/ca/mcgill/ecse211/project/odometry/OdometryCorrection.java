package ca.mcgill.ecse211.project.odometry;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.game.GameState;
import lejos.hardware.Sound;
import ca.mcgill.ecse211.project.game.GameController;

public class OdometryCorrection {


  /**
   * linked list to store the data from the window of samples
   */
  // private LinkedList<Float> llDataList;

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
   * coordinates to reach with light localization
   */
  private int[] coordinates;
  
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

  /**
   * Constructor of the light localization class
   */
  public OdometryCorrection() {
    // initialize the data list
    // llDataQueue = new LinkedList<Float>();
    // llDataList = new LinkedList<Float>();

    // set sensor to use red light alone
    leftColorSensor.setCurrentMode("Red");
    rightColorSensor.setCurrentMode("Red");

    //    leftSensorData = new float[leftColorSensor.sampleSize()];

    // initiate the current readings of the sensor
    leftColorSensor.fetchSample(leftSensorData, 0);
    rightColorSensor.fetchSample(rightSensorData, 0);

    // initialize the sensor readings
    currentLeftColorValue = (leftSensorData[0] * 100);
    lastLeftColorValue = currentLeftColorValue;

    currentRightColorValue = (rightSensorData[0] * 100);
    lastLeftColorValue = currentRightColorValue;

    //coordinates = new int[2];
    
  }
  
  public void correctValues() {
    if((leftValues != null)) {
      leftMotor.stop();
    }
    if((rightValues != null)) {
      rightMotor.stop();
    }
    if((leftValues != null) && (rightValues != null)) {
      
      //continue since the robot is aligned.
      
      // calculate correction.
      
      // apply correction.
      Sound.beep();
      
      leftValues = null;
      rightValues = null;
    }

  }
  
  
  public void correction() {
    while(gameState == GameState.Navigation) {
      int lines = lineDetected();
      if(lines == 1) {
        //only left sensor detected
        leftValues = odometer.getXYT();
        
      }
      else if (lines == 2) {
        //only right sensor detected
        rightValues = odometer.getXYT();
      }
      else {
        // Both lines detected or no lines detected. 
        // Therefore, do nothing
        
      }
      correctValues();
      
      if (endTime - startTime < LIGHT_SENSOR_PERIOD) {
        // Sleep the correct amount of time
        try {
          Thread.sleep(LIGHT_SENSOR_PERIOD - (endTime - startTime));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      
    }
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



  // when one line is detected, save odometer X, Y, Theta
  // when the other sensor detects the line, save Odometer X, Y, Theta.
  // Do math to compute the offset of the robot
  // Change Odometer reading depending on the offset.



}