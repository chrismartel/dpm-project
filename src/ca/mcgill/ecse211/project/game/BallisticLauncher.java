package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;

/**
 * This class implements the behaviour of the ballistic launcher of the robot
 */
public class BallisticLauncher {

  
  /**
   * The distance to launch to
   */
  private double distance;


  /**
   * Method implementing the behaviour of the ballistic motors during launch
   * 
   * @param : the distance we want the ball to travel in cm
   */
  public void launch(double distance) {
    // sleeps 5 seconds before launch
    try {
      Thread.sleep(LAUNCH_SLEEP);
    } catch (InterruptedException e) {
    }
    // set the launching speed to a constant (only for lab 5)
    int  motorLaunchingSpeed = (int)(distance * LAUNCH_COEFFICIENT);
    // set the speeds and accelerations of the launching motors
    leftBallisticMotor.setSpeed(motorLaunchingSpeed);
    rightBallisticMotor.setSpeed(motorLaunchingSpeed);
    leftBallisticMotor.setAcceleration(LAUNCH_ACCELERATION);
    rightBallisticMotor.setAcceleration(LAUNCH_ACCELERATION);
    // launch the ball by rotating a particular amount of degrees
    leftBallisticMotor.rotate(-LAUNCHING_ANGLE, true);
    rightBallisticMotor.rotate(-LAUNCHING_ANGLE, false);
    // sleeps after launch
    try {
      Thread.sleep(RELOAD_SLEEP);
    } catch (InterruptedException e) {
    }
  }

  /**
   * Method implementing the behaviour of the ballistic motors during reload
   */
  public void reload() {
    // set the speed and acceleration of the motors for reload
    leftBallisticMotor.setSpeed(ROTATE_SPEED_SLOW);
    rightBallisticMotor.setSpeed(ROTATE_SPEED_SLOW);
    leftBallisticMotor.setAcceleration(RELOAD_ACCELERATION);
    rightBallisticMotor.setAcceleration(RELOAD_ACCELERATION);
    // reload the launcher by rotating back the motors
    leftBallisticMotor.rotate(RELOAD_ANGLE, true);
    rightBallisticMotor.rotate(RELOAD_ANGLE, false);

  }
  
  /**
   * Method implementing multiple launches and reload depending on how many balls the robot is carrying
   */
  public void multipleLaunch(double distance) {
    for(int i = 0 ; i< NUMBER_OF_BALLS; i++) {
      this.launch(distance);
      this.reload();
    }
  }

  /**
   * Setter for the launching distance
   */
  public void setDistance(double distance) {
    this.distance = distance;
  }

}
