package ca.mcgill.ecse211.project.localization;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.game.Navigation;
import ca.mcgill.ecse211.project.game.Navigation.Turn;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * This class implements the methods used to execute the light localization using 2 sensors at the back of the robot
 */
public class LightLocalizer {
  // TODO: light localization with 2 sensors at the back
  /**
   * last and current sensor color readings used for comparison in each line detection loop
   */
  private float currentColorValue;

  private float lastColorValue;

  // Variables and values to operate color sensor
  private float[] sensorData = new float[leftColorSensor.sampleSize()]; // array of sensor readings

  /**
   * coordinates to reach with light localization
   */
  private double[] coordinates;


  /**
   * Constructor of the light localization class
   */
  /*
  public LightLocalizer() {
    // initialize the data list
    // llDataQueue = new LinkedList<Float>();
    // llDataList = new LinkedList<Float>();

    // set sensor to use red light alone
    leftColorSensor.setCurrentMode("Red");
    sensorData = new float[leftColorSensor.sampleSize()];
    // initiate the current readings of the sensor
    leftColorSensor.fetchSample(sensorData, 0);
    // initialize the sensor readings
    currentColorValue = (sensorData[0] * 100);
    lastColorValue = currentColorValue;
    coordinates = new double[2];
  }*/

  /**
   * Method implementing the initial positioning of the robot before light localization
   */
  /*
  public void initialPositioning() {
    // Set motion towards point (1,1)
    Navigation.turnTo(45, ROTATE_SPEED_SLOW);
    // travel forward until a line is detected
    Navigation.travelForward(ROTATE_SPEED_SLOW);
    while (!this.lineDetected());
    // Stop as soon as first black line is detected by the light sensor
    Sound.beep();
    Navigation.stopMotors();
    // Make robot move back such that the center of rotation is somewhat close to the point (1,1)
    Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_SLOW);
  }
*/
  /**
   * Method performs the light localization routine. Robot rotates around center of rotation on point (1,1). The angle
   * values recorded when a black line is detected is used to correct the robots heading and position on the point
   * (1,1). The robot returns to a 0 degree stop at the end of the process.
   * 
   */
  /*
  public void lightLocalize() {

    // Perform full rotation to record angle values at each black line
    Navigation.rotate(Turn.COUNTER_CLOCK_WISE, ROTATE_SPEED_SLOW);

    double angles[] = new double[4];

    double tempTheta = 0;
    double deltaTheta;
    double thetaYMinus = 0;

    // Execute until 5 lines are detected
    for (int lineCounter = 0; lineCounter < 5; lineCounter++) {
      // wait until a line is detected
      while (!this.lineDetected());
      // minus y axis line
      if (lineCounter == 0) {
        thetaYMinus = odometer.getTheta();
//        System.out.println("thetaYMinus: " + thetaYMinus);
      }
      if (lineCounter == 0 || lineCounter == 1 || lineCounter == 2) {
        tempTheta = odometer.getTheta() - 180;
      }
      if (lineCounter == 3) {
        tempTheta = odometer.getTheta() + 180;
      }
      // adjust each angle so there are not bigger than 360 or smaller than -360
      tempTheta = tempTheta % 360;
      if (tempTheta < 0) {
        tempTheta = 360 + tempTheta;
      }
      // stores the angles recorded in the array for the first 4 lines
      if (lineCounter != 4) {
        angles[lineCounter] = tempTheta;

      } else {
        // stop motors after 5 lines are detected
        Navigation.stopMotors();
        Sound.beep();
      }
    }
    // Calculations to know current robot location and corrections to make accordingly
    // Compute the thetas
    double thetaY = Math.toRadians(angles[0] - angles[2]);
    double thetaX = Math.toRadians(angles[3] - angles[1]);
    double y = (OFFSET_FROM_WHEELBASE * Math.cos(thetaX / 2));
    double x = (OFFSET_FROM_WHEELBASE * Math.cos(thetaY / 2));
    // Compute the angle to add to the odometer
    deltaTheta = (270 + (Math.toDegrees(thetaY) / 2) - thetaYMinus);
    // Adjust the odometer angle
    odometer.setTheta(odometer.getTheta() + deltaTheta);
    // Adjust the odometer x and y
    odometer.setXYT(this.getCoordinates()[0] * TILE_SIZE + x, this.getCoordinates()[1] * TILE_SIZE + y, 0);
    Button.waitForAnyPress();
    Navigation.travelTo(coordinates[0], coordinates[1], FORWARD_SPEED_NORMAL);
    // turn to 0 degree
    Navigation.turnTo(0, ROTATE_SPEED_SLOW);
  }


  /**
   * Method determining if a line is detected
   * 
   * @return : true if a line is detected and false if no lines are detected
   */
  public boolean lineDetected() {
    long positioningStart;
    long positioningEnd;
    float temporaryColorValue;

    // keep track of the execution time of the method to adjust the period
    positioningStart = System.currentTimeMillis();
    // no line detected initially
    boolean line = false;
    // fetch sample from the light sensor
    leftColorSensor.fetchSample(sensorData, 0);
    temporaryColorValue = sensorData[0] * 100;

    // set the current color value
    this.currentColorValue = temporaryColorValue;

    // Check if the difference between the previous reading and the current reading is bigger than the differential line
    // threshold
    if ((this.lastColorValue - this.currentColorValue) >= DIFFERENTIAL_LINE_THRESHOLD) {
      // System.out.println("DIFF: " + (this.lastColorValue - this.currentColorValue));
      Sound.beep();
      // a line is detected
      line = true;
    }

    // update the last reading of the light sensor with the current one
    lastColorValue = currentColorValue;

    // pause the thread to respect the line detection period
    positioningEnd = System.currentTimeMillis();
    if (positioningEnd - positioningStart < LIGHT_SENSOR_PERIOD) {

      // Sleep the correct amount of time
      try {
        Thread.sleep(LIGHT_SENSOR_PERIOD - (positioningEnd - positioningStart));
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    return line;
  }

  /**
   * getter and setter for the goal coordinates of the light localization [x, y]
   */
  public void setCoordinates(double[] coordinates) {
    this.coordinates = coordinates;
  }

  public double[] getCoordinates() {
    return coordinates;
  }
}
