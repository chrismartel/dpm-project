package ca.mcgill.ecse211.project.Localization;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer {



  // Variables and values to operate color sensor
  private static SampleProvider color_sensor = colorSensor.getRedMode();
  private static float[] sensor_data = new float[color_sensor.sampleSize()]; // array of sensor readings
  private static int current_color_value = 1000;

  // Array holds angle values
  private static double[] angles = new double[4];

  /**
   * Method performs the light localization routine. Robot rotates 360 degrees around center of rotation on point (1,1).
   * The angle values recorded when a black line is detected is used to correct the robots heading and position on the
   * point (1,1). The robot returns to a 0 degree stop at the end of the process.
   * 
   * @author Abhimukth Chaudhuri, Aly Elgharabawy
   * 
   */
  public static void lightLocalize() {

    // Set motion towards point (1,1)
    navigation.turnTo(45);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.forward();

    while (true) {
      // Stop as soon as first black line is detected by the light sensor
      if (LightLocalizer.blackLineTrigger()) {
        Sound.beep();
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        break;
      }
    }

    // Make robot move back such that the center of roation is somewhat close to the point (1,1)
    navigation.backUp(OFFSET_FROM_WHEELBASE);

    // Perform full rotation to record angle values at each black line
    navigation.turn(-360);

    int lineCounter = 0;
    double tempTheta = 0;
    double deltaTheta;
    double thetaYMinus = 0;

    while (true) {
      if (LightLocalizer.blackLineTrigger()) {

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
        angles[lineCounter] = tempTheta;
        Sound.beep();
        lineCounter++;
      }
      // Break after 4 angle values have been recorded
      if (lineCounter == 4)
        break;
    }

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
  public static boolean blackLineTrigger() {
    // tools to manage the sensor period
    long positioningStart;
    long positioningEnd;
    positioningStart = System.currentTimeMillis();

    color_sensor.fetchSample(sensor_data, 0);
    current_color_value = (int) (sensor_data[0] * 100);
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
    if (current_color_value < LINE_THRESHOLD) {
      return true;
    } else {
      return false;
    }
  }
}
