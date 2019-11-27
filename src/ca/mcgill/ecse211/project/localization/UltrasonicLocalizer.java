package ca.mcgill.ecse211.project.Localization;

import ca.mcgill.ecse211.project.game.GameResources;
import ca.mcgill.ecse211.project.game.Navigation;
import ca.mcgill.ecse211.project.game.Navigation.Turn;


/**
 * This class contains the method for the ultrasonic localization falling edge routine using two ultrasonic sensors
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
  public void fallingEdge(int rotateSpeed) {

    double angleAdjustment = 0;
    int[] distances;
    int currentDistance;
    int lastDistance;
    // clockwise rotation to record value for alpha
    Navigation.rotate(Turn.CLOCK_WISE, rotateSpeed);
    while (true) {
      distances = GameResources.ultrasonicPoller.getFrontUsController().getDistances();
      currentDistance = distances[0];
      lastDistance = distances[1];
      
      if (lastDistance > (GameResources.FALLINGEDGE_D + GameResources.FALLINGEDGE_K) && currentDistance < (GameResources.FALLINGEDGE_D - GameResources.FALLINGEDGE_K)) {
//        System.out.println("differential: "+(lastDistance-currentDistance));
        this.alpha = GameResources.odometer.getTheta();
        Navigation.stopMotors();
        break;
      }
    }

    // anti-clockwise rotation to record beta value
    Navigation.turn(GameResources.FALLING_EDGE_ADJUSTMENT_ANGLE, GameResources.ROTATE_SPEED_FAST);
    Navigation.rotate(Turn.COUNTER_CLOCK_WISE, rotateSpeed);
    while (true) {
      distances = GameResources.ultrasonicPoller.getFrontUsController().getDistances();
      currentDistance = distances[0];
      lastDistance = distances[1];
      // if (ultrasonicPoller.getDistance() < COMMON_D - FALLINGEDGE_K) {
      if (lastDistance > (GameResources.FALLINGEDGE_D + GameResources.FALLINGEDGE_K) && currentDistance < (GameResources.FALLINGEDGE_D - GameResources.FALLINGEDGE_K)) {
        this.beta = GameResources.odometer.getTheta();
        Navigation.stopMotors();
        break;
      }
    }
    // compute the angle heading considering the 2 angles obtained
    angleAdjustment = this.angleHeadingAdjustment();
    // Adjust the current theta of the odometer by adding the computed heading
    GameResources.odometer.update(0, 0, angleAdjustment);
    Navigation.turnTo(0, rotateSpeed);
    // set theta depending on the starting corner
    switch(GameResources.CORNER_NUMBER) {
      // lower left corner
      case 0:
        GameResources.odometer.setTheta(0);
        break;
        // lower right corner
      case 1:
        GameResources.odometer.setTheta(270);
        break;
      case 2:
        // top right corner
        GameResources.odometer.setTheta(180);
        break;
      case 3:
        // top left corner
        GameResources.odometer.setTheta(90);
        break;
    }
    

  }


  /**
   * Computes the heading to add to the odometer current angle considering the two angles obtained by falling
   * edge routine
   * 
   * @return : the heading to add to the odometer
   */
  public double angleHeadingAdjustment() {
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
