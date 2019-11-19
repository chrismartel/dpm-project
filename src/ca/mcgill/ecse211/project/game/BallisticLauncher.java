package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.game.GameResources.*;

/**
 * This class implements the behaviour of the ballistic launcher of the robot
 */
public class BallisticLauncher {

  /**
   * Method implementing the behaviour of the ballistic motors during launch
   * 
   * @param : the distance we want the ball to travel in cm
   */
  public void launch(double distance) {
    // sleeps 5 seconds before launch
    try {
      Thread.sleep(GameResources.LAUNCH_SLEEP);
    } catch (InterruptedException e) {
    }
    // set the launching speed to a constant (only for lab 5)
    int  motorLaunchingSpeed = (int) this.computeMotorSpeed(distance);
    // set the speeds and accelerations of the launching motors
    GameResources.leftBallisticMotor.setSpeed(motorLaunchingSpeed);
    GameResources.rightBallisticMotor.setSpeed(motorLaunchingSpeed);
    GameResources.leftBallisticMotor.setAcceleration(GameResources.LAUNCH_ACCELERATION);
    GameResources.rightBallisticMotor.setAcceleration(GameResources.LAUNCH_ACCELERATION);
    // launch the ball by rotating a particular amount of degrees
    GameResources.leftBallisticMotor.rotate(-GameResources.LAUNCHING_ANGLE, true);
    GameResources.rightBallisticMotor.rotate(-GameResources.LAUNCHING_ANGLE, false);
    // sleeps after launch
    try {
      Thread.sleep(GameResources.RELOAD_SLEEP);
    } catch (InterruptedException e) {
    }
  }
  
  /**
   * Method used to test the ballistic launcher at different motor speeds in order to determine the launch coefficient
   * 
   * @param : the motor speed to apply to the launching motors
   */
  public void launchTest(int motorSpeed) {
    // sleeps 5 seconds before launch
    try {
      Thread.sleep(GameResources.LAUNCH_SLEEP);
    } catch (InterruptedException e) {
    }
    // set the speeds and accelerations of the launching motors
    GameResources.leftBallisticMotor.setSpeed(motorSpeed);
    GameResources.rightBallisticMotor.setSpeed(motorSpeed);
    GameResources.leftBallisticMotor.setAcceleration(GameResources.LAUNCH_ACCELERATION);
    GameResources.rightBallisticMotor.setAcceleration(GameResources.LAUNCH_ACCELERATION);
    // launch the ball by rotating a particular amount of degrees
    GameResources.leftBallisticMotor.rotate(-GameResources.LAUNCHING_ANGLE, true);
    GameResources.rightBallisticMotor.rotate(-GameResources.LAUNCHING_ANGLE, false);
    // sleeps after launch
    try {
      Thread.sleep(GameResources.RELOAD_SLEEP);
    } catch (InterruptedException e) {
    }
  }

  /**
   * Method implementing the behaviour of the ballistic motors during reload
   */
  public void reload() {
    // set the speed and acceleration of the motors for reload
    GameResources.leftBallisticMotor.setSpeed(GameResources.RELOAD_SPEED);
    GameResources.rightBallisticMotor.setSpeed(GameResources.RELOAD_SPEED);
    GameResources.leftBallisticMotor.setAcceleration(GameResources.RELOAD_ACCELERATION);
    GameResources.rightBallisticMotor.setAcceleration(GameResources.RELOAD_ACCELERATION);
    // reload the launcher by rotating back the motors
    GameResources.leftBallisticMotor.rotate(GameResources.RELOAD_ANGLE, true);
    GameResources.rightBallisticMotor.rotate(GameResources.RELOAD_ANGLE, false);

  }
  
  /**
   * Method implementing multiple launches and reload depending on how many balls the robot is carrying
   */
  public void multipleLaunch(double distance) {
    for(int i = 0 ; i< GameResources.NUMBER_OF_BALLS; i++) {
      this.launch(distance);
      this.reload();
    }
  }
  
  public double computeMotorSpeed(double distance) {
    double motorSpeed = GameResources.LAUNCH_COEFFICIENT*distance+ GameResources.LAUNCH_IV;
    return motorSpeed;
  }



}
