package ca.mcgill.ecse211.project.Localization;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Navigation.Turn;


/**
 * This class contains methods for the two ultrasonic localization routines: the falling edge and rising edge
 * localization routines.
 *
 */
public class UltrasonicLocalizer {

  /**
   * angle at which back wall is detected
   */
  public double alpha;
  /**
   * angle at which left wall is detected
   */
  public double beta;


  /**
   * Method performs the falling edge localization. Robot always completes an clockwise rotation around its center of
   * rotation to record the value for alpha. Then it rotates in the anti-clockwise direction to record value for beta.
   * Using the values recorded, the robot will then appropriately orient itself accordingly along the 0 degree y-axis.
   */
  public void fallingEdge() {

    double angleAdjustment = 0;
    int[] distances;
    int currentDistance;
    int lastDistance;
    // clockwise rotation to record value for alpha
    navigation.rotate(Turn.CLOCK_WISE, ROTATE_SPEED_SLOW);
    while (true) {
      distances = ultrasonicPoller.getDistances();
      currentDistance = distances[0];
      lastDistance = distances[1];
      System.out.println(ultrasonicPoller.getDistance());
      //if (ultrasonicPoller.getDistance() < COMMON_D - FALLINGEDGE_K) {
    if (lastDistance>=(COMMON_D+FALLINGEDGE_K) && currentDistance<=(COMMON_D-FALLINGEDGE_K)) {
        this.alpha = odometer.getTheta();
        navigation.stopMotors();
        break;
      }
    }

    // anti-clockwise rotation to record beta value
    navigation.turn(-20, ROTATE_SPEED_SLOW);
    navigation.rotate(Turn.COUNTER_CLOCK_WISE,ROTATE_SPEED_SLOW);
    while (true) {
      distances = ultrasonicPoller.getDistances();
      currentDistance = distances[0];
      lastDistance = distances[1];
      //if (ultrasonicPoller.getDistance() < COMMON_D - FALLINGEDGE_K) {
       if (lastDistance>=(COMMON_D+FALLINGEDGE_K) && currentDistance<=(COMMON_D-FALLINGEDGE_K)) {
        this.beta = odometer.getTheta();
        navigation.stopMotors();
        break;
      }
    }
    // compute the angle heading considering the 2 angles obtained
    angleAdjustment = this.angleHeadingAdjustment();
    // Adjust the current theta of the odometer by adding the computed heading
    odometer.setTheta(odometer.getTheta() + angleAdjustment);
    navigation.turnTo(0);

  }

  /**
   * Method performs the rising edge localization. Robot always completes an clockwise rotation around its center of
   * rotation to record the value for alpha. Then it rotates in the anti-clockwise direction to record value for beta.
   * Using the values recorded, the robot will then appropriately orient itself accordingly along the 0 degree y-axis.
   */
  public void risingEdge() {

    double angleAdjustment = 0;

    // clockwise rotation to record alpha value
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(-1 * ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.backward();
    while (true) {
      if (ultrasonicPoller.getDistance() > COMMON_D + RISINGEDGE_K) {
        this.alpha = odometer.getXYT()[2];
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        break;
      }

    }

    // anti-clockwise rotation to record beta value
    leftMotor.setSpeed(-1 * ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(-70, true);
    rightMotor.rotate(70);
    leftMotor.backward();
    rightMotor.forward();
    while (true) {
      if (ultrasonicPoller.getDistance() > COMMON_D + RISINGEDGE_K) {
        this.beta = odometer.getXYT()[2];
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        break;

      }
    }

    // compute the angle heading considering the 2 angles obtained
    angleAdjustment = this.angleHeadingAdjustment();
    // Adjust the current theta of the odometer by adding the computed heading
    odometer.setTheta(odometer.getTheta() + angleAdjustment);
    navigation.turnTo(0);
  }

  /**
   * Computes the heading to add to the odometer current angle considering the two angles obtained by rising or falling
   * edge routines
   * 
   * @return : the heading to add to the odometer
   */
  private double angleHeadingAdjustment() {
    double deltaTheta = 0;
    // Formulas to compute the change in heading of the robot
    if (alpha < beta) {
      deltaTheta = (225 - (alpha + beta) / 2);
    } else if (alpha > beta) {
      deltaTheta = (45 - (alpha + beta) / 2);
    }
    deltaTheta = deltaTheta % 360;
    return deltaTheta;
  }

}
