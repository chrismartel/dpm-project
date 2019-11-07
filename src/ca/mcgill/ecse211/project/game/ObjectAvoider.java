package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;

public class ObjectAvoider {

  private int objectDistance;


  /**
   * Method describing the wall following process using P-Controller
   */
  public void wallFollower() {
    // TODO: implement conditions that checks the current limits and generates unreal obstacles at those limits
    // initial positioning
    Navigation.turn(90, ROTATE_SPEED_NORMAL);
    double wallDistance;
    double leftSpeed;
    double rightSpeed;
    double startTime;
    double endTime;
    Navigation.travelForward(FORWARD_SPEED_SLOW);
    while (!orientationCheck()) {
      startTime = System.currentTimeMillis();
      wallDistance = ultrasonicPoller.getLeftUsController().getDistance();
      wallDistance = this.calculateLateralDistance(wallDistance);
      double error = wallDistance - BAND_CENTER;
      // both motors go forward, robot is at correct distance from obstacle
      if ((-1) * BAND_WIDTH <= error && error <= BAND_WIDTH) {
        leftMotor.forward();
        rightMotor.forward();
        leftMotor.setSpeed(FORWARD_SPEED_SLOW);
        rightMotor.setSpeed(FORWARD_SPEED_SLOW);
      } else {
        // robot is too far from wall
        if (error > 0) {
          // imminent collision
          if (error > MAXIMAL_ERROR) {
            leftMotor.forward();
            rightMotor.backward();
            leftSpeed = FORWARD_SPEED_SLOW;
            rightSpeed = FORWARD_SPEED_SLOW;
          }
          // normal adjustment
          else {
            leftMotor.forward();
            rightMotor.forward();
            leftSpeed = (int) (FORWARD_SPEED_SLOW - this.calculateGain(error));
            rightSpeed = (int) (FORWARD_SPEED_SLOW + this.calculateGain(error));
          }
        }
        // robot is too close to wall
        else {
          if (error < MINIMAL_ERROR) {
            leftMotor.backward();
            rightMotor.forward();
            leftSpeed = FORWARD_SPEED_SLOW;
            rightSpeed = FORWARD_SPEED_SLOW;
          }
          // normal adjustment
          else {
            leftMotor.forward();
            rightMotor.forward();
            leftSpeed = (int) (FORWARD_SPEED_SLOW + this.calculateGain(error));
            rightSpeed = (int) (FORWARD_SPEED_SLOW - this.calculateGain(error));
          }
        }
        // set bounds on the speed during avoidance
        if (leftSpeed <= MIN_AVOID_SPEED) {
          leftSpeed = MIN_AVOID_SPEED;
        } else if (leftSpeed >= MAX_AVOID_SPEED) {
          leftSpeed = MAX_AVOID_SPEED;
        }
        if (rightSpeed <= MIN_AVOID_SPEED) {
          rightSpeed = MIN_AVOID_SPEED;
        } else if (rightSpeed >= MAX_AVOID_SPEED) {
          rightSpeed = MAX_AVOID_SPEED;
        }
        leftMotor.setSpeed((int) leftSpeed);
        rightMotor.setSpeed((int) rightSpeed);
        endTime = System.currentTimeMillis();
        if (endTime - startTime < OBJECT_AVOIDANCE_PERIOD) {
          try {
            Thread.sleep((long) (OBJECT_AVOIDANCE_PERIOD - (endTime - startTime)));
          } catch (InterruptedException e) {
            // there is nothing to be done
          }
        }
      }
    }
    Navigation.stopMotors();
 

  }

  /**
   * Method indicating if the orientation of the robot is correct or not during the wall following process
   * 
   * @return true if the orientation of the robot
   */
  public boolean orientationCheck() {
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    double goalX = navigationCoordinates.x * TILE_SIZE;
    double goalY = navigationCoordinates.y * TILE_SIZE;
    double dX = goalX - currentX;
    double dY = goalY - currentY;
    double turnToAngle = Math.abs(Math.toDegrees(Math.atan2(dX, dY)));
    if (turnToAngle <= ORIENTATION_CHECK_ERROR) {
      return true;
    }
    return false;
  }

  // Method to calculate the gain of the motors proportionally with the error processed
  private double calculateGain(double error) {
    double absError = Math.abs(error);
    double gain = absError * GAIN_CONSTANT;
    return gain;
  }

  double calculateLateralDistance(double distance) {// method used to approximate the lateral distance of the robot from
                                                    // the wall
    // lateral distance = distance *cos(45)
    // 4 centimeters between sensor and wheels
    double lateralDistance = distance * Math.acos(Math.PI / 4);
    return lateralDistance;
  }

  public int getObjectDistance() {
    return objectDistance;
  }

  public void setObjectDistance(int objectDistance) {
    this.objectDistance = objectDistance;
  }


  
}
