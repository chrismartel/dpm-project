package ca.mcgill.ecse211.project.Localization;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Navigation.Turn;
import lejos.hardware.Sound;

public class LightLocalizer {

  /**
   * last and current sensor color readings used for comparison in each line detection loop
   */
  private float currentColorValue;

  private float lastColorValue;

  // Variables and values to operate color sensor
  private float[] sensorData = new float[colorSensor.sampleSize()]; // array of sensor readings


  // Array holds angle values
  private double[] angles = new double[4];

  /**
   * Constructor of the light localization class
   */
  public LightLocalizer() {
    // set sensor to use red light alone
    colorSensor.setCurrentMode("Red");
    sensorData = new float[colorSensor.sampleSize()];
    // initiate the current readings of the sensor
    colorSensor.fetchSample(sensorData, 0);
    // initialize the sensor readings
    currentColorValue = (sensorData[0] * 100);
    lastColorValue = currentColorValue;
  }
  
  public void initialPositioning() {
    // Set motion towards point (1,1)
    navigation.turnTo(45);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    while (!this.lineDetected());
    // Stop as soon as first black line is detected by the light sensor
    Sound.beep();
    navigation.stopMotors();
    // Make robot move back such that the center of roation is somewhat close to the point (1,1)
    navigation.backUp(OFFSET_FROM_WHEELBASE);
  }

  /**
   * Method performs the light localization routine. Robot rotates 360 degrees around center of rotation on point (1,1).
   * The angle values recorded when a black line is detected is used to correct the robots heading and position on the
   * point (1,1). The robot returns to a 0 degree stop at the end of the process.
   * 
   */
  public void lightLocalize() {

    // Perform full rotation to record angle values at each black line
    navigation.rotate(Turn.COUNTER_CLOCK_WISE);

    double tempTheta = 0;
    double deltaTheta;
    double thetaYMinus = 0;


    for (int lineCounter = 0; lineCounter < 5; lineCounter++) {

      while (!this.lineDetected());
      Sound.beep();

      if (lineCounter == 0) {
        thetaYMinus = odometer.getTheta();
      }
      if (lineCounter == 0 || lineCounter == 1 || lineCounter == 2) {
        tempTheta = tempTheta - 180;
      }
      if (lineCounter == 3) {
        tempTheta = tempTheta + 180;
      }
      // adjust each angle so there are not bigger than 360 or smaller than -360
      tempTheta = tempTheta % 360;
      if (tempTheta < 0) {
        tempTheta = 360 + tempTheta;
      }

      if (lineCounter != 4) {
        angles[lineCounter] = tempTheta;
      }
    }
    navigation.stopMotors();

    // Calculations to know current robot location and corrections to make accordingly
    double thetaY = angles[0] - angles[2];
    double thetaX = angles[3] - angles[1];
    double x = (OFFSET_FROM_WHEELBASE * Math.cos(Math.toRadians(thetaY) / 2));
    double y = -(OFFSET_FROM_WHEELBASE * Math.cos(Math.toRadians(thetaX) / 2));

    // formula to compute the angle to add to the odometer
    deltaTheta = (270 + (thetaY / 2) - thetaYMinus) % 360;
    odometer.setTheta(odometer.getTheta() + deltaTheta);

    // turn towards the positive x axis
    // travel the distance x forward or backward depending on the sign of x
    navigation.turnTo(90);
    if (x < 0) {
      navigation.backUp(x);
    } else {
      navigation.travel(x);
    }

    // turn towards the positive y axis
    // travel the distance y forward or backward depending on the sign of y
    navigation.turnTo(0);
    if (y < 0) {
      navigation.backUp(y);
    } else {
      navigation.travel(y);
    }

    // Return to 0 degree y-axis
    navigation.turnTo(0);
  }

  /**
   * The method fetches data recorded by the color sensor in RedMode and compares the most recent value to verify if the
   * robot has traveled over a black line. Method makes use of a fixed threshold value which may not be reliable in
   * certain conditions, however it has been tested and conditioned to minimize false negatives.
   * 
   * @return true iff black line is detected
   */
  public boolean blackLineTrigger() {
    // tools to manage the sensor period
    long positioningStart;
    long positioningEnd;
    positioningStart = System.currentTimeMillis();

    colorSensor.fetchSample(sensorData, 0);
    currentColorValue = (int) (sensorData[0] * 100);
    positioningEnd = System.currentTimeMillis();

    // Manage the light sensor period
    if (positioningEnd - positioningStart < LIGHT_SENSOR_PERIOD) {
      try {
        Thread.sleep(LIGHT_SENSOR_PERIOD - (positioningEnd - positioningStart));
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

    }
    // When recorded color intensity is below threshold
    if (currentColorValue < LINE_THRESHOLD) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Method determining if a line is detected
   * 
   * @return : true if a line is detected and false if no lines are detected
   */
  public boolean lineDetected() {
    long positioningStart;
    long positioningEnd;
    positioningStart = System.currentTimeMillis();
    // no line detected initially
    boolean line = false;
    // fetch sample from the light sensor
    colorSensor.fetchSample(sensorData, 0);
    currentColorValue = sensorData[0] * 100;
    // Check if the difference between the previous reading and the current reading is bigger than the line threshold
    if ((this.lastColorValue - this.currentColorValue) >= DIFFERENTIAL_LINE_THRESHOLD) {
      Sound.beep();
      // a line is detected
      line = true;
    }
    // update the last reading of the light sensor
    lastColorValue = currentColorValue;
    // pause the thread to respect the line detection period
    positioningEnd = System.currentTimeMillis();
    if (positioningEnd - positioningStart < LIGHT_SENSOR_PERIOD) {
      try {
        Thread.sleep(LIGHT_SENSOR_PERIOD - (positioningEnd - positioningStart));
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

    }
    return line;
  }
}
