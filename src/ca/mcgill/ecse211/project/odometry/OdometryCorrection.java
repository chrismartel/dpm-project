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
  
  public static void correctValues() {
    int difference = 5;
    double lowerBound = odometer.getTheta() - difference;
    double upperBound = odometer.getTheta() + difference;
    
    if((lowerBound >= (0 - difference) && upperBound <= (0 + (2* difference))) ||
        (lowerBound >= (360 - (2 * difference)) && upperBound <= (360 + difference))) {
      //The robot is going forward
      double currentYLine = odometer.getY() - OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentYLine / TILE_SIZE); 
      odometer.setY(lineCount * TILE_SIZE + OFFSET_FROM_WHEELBASE);
      odometer.setTheta(0);
      System.out.println("GOING FORWARD");
    }

    else if(lowerBound >= (90 - (2 * difference)) && upperBound <= (90 + (2 * difference)) ) {
      //going right
      double currentXLine = odometer.getX() - OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentXLine / TILE_SIZE); 
      Sound.beep();
      System.out.println("line count" + lineCount);
      odometer.setX(lineCount * TILE_SIZE + OFFSET_FROM_WHEELBASE);
      odometer.setTheta(90);
      System.out.println("GOING right");
    }
    else if(lowerBound >= (180 - (2 * difference) ) && upperBound <= (180 + (2 * difference)) ) {
      //going backward
      double currentYLine = odometer.getY() + OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentYLine / TILE_SIZE); 
      odometer.setY(lineCount * TILE_SIZE - OFFSET_FROM_WHEELBASE);
      odometer.setTheta(180);
      System.out.println("GOING backward");
    }
    else if(lowerBound >= (270 - (2 * difference) ) && upperBound <= (270 + (2 * difference)) ){
      //going left
      double currentXLine = odometer.getX() + OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentXLine / TILE_SIZE); 
      odometer.setX(lineCount * TILE_SIZE - OFFSET_FROM_WHEELBASE);
      odometer.setTheta(270);
      System.out.println("GOING left");
    }
    else {
      Sound.beep();
      System.out.println("gonig where??");
    }
  }
  
  
  public void correction() {
    
  }





  // when one line is detected, save odometer X, Y, Theta
  // when the other sensor detects the line, save Odometer X, Y, Theta.
  // Do math to compute the offset of the robot
  // Change Odometer reading depending on the offset.



}