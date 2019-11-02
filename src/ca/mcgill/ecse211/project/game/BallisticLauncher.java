package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;

/**
 * This class implements the behaviour of the ballistic launcher of the robot
 */
public class BallisticLauncher {
  /**
   * The motors launching speed
   */
  private int motorLaunchingSpeed;
  /**
   * Instance of the ballistic launcher returned as singleton
   */
  private static BallisticLauncher bl;

  /**
   * Method to get the coordinates of the launching point of the robot
   * 
   * @return : the array of coordinates of the launching point
   */
  public double[] launchLocation(double x, double y) {
    y = (y - 4.5);
    double[] location = new double[2];
    location[0] = x;
    location[1] = y;
    return location;
  }

  /**
   * Method implementing the behaviour of the ballistic motors during launch
   * 
   * @param : the distance we want the ball to travel
   */
  public void launch(double distance) {
    // sleeps 5 seconds before launch
    try {
      Thread.sleep(LAUNCH_SLEEP);
    } catch (InterruptedException e) {
    }
    // set the launching speed to a constant (only for lab 5)
    motorLaunchingSpeed = (int)(distance * LAUNCH_COEFFICIENT);
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
    leftBallisticMotor.rotate(LAUNCHING_ANGLE, true);
    rightBallisticMotor.rotate(LAUNCHING_ANGLE, false);

  }

  /**
   * Returns the BallisticLauncher object. Use this method to obtain an instance of Launcher. Method used to make sure
   * there is just one instance of BallisticLauncher throughout the code
   * 
   * @return the BallisticLauncher Object
   */
  public synchronized static BallisticLauncher getBallisticLauncher() {
    if (bl == null) {
      bl = new BallisticLauncher();
    }
    return bl;
  }
}
