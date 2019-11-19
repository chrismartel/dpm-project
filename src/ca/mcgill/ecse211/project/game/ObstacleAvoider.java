package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.game.Navigation.Turn;

public class ObstacleAvoider {

  private int obstacleDistance;


  /**
   * Method describing the wall following process using P-Controller
   */
  public void wallFollower(int speed) {
    // initial positioning
    Navigation.turn(90, GameResources.ROTATE_SPEED_FAST);
    double wallDistance;
    double leftSpeed;
    double rightSpeed;
    double startTime;
    double endTime;
    int convexCornerCounter = 0;
    Navigation.travelForward(speed);
    while (!orientationCheck()) {
      startTime = System.currentTimeMillis();
      // get distance from obstacle
      wallDistance = GameResources.ultrasonicPoller.getLeftUsController().getDistance();
      wallDistance = this.calculateLateralDistance(wallDistance);

      // CONVEX CORNER HANDLING
      if (wallDistance >= GameResources.CONVEX_CORNER_CONSTANT) {
        convexCornerCounter++;
        // convex corner is faced
        if (convexCornerCounter > 5) {
          convexCornerCounter = 0;
          this.convexCornerProcedure();
          // after passing the convex corner, get new distance and restart time counter
          wallDistance = GameResources.ultrasonicPoller.getLeftUsController().getDistance();
          wallDistance = this.calculateLateralDistance(wallDistance);

        }
      }
      // GENERAL WALL FOLLOWING PROCEDURE
      // The distance seen is small
      else {
        double error = wallDistance - GameResources.BAND_CENTER;
        // both motors go forward, robot is at correct distance from obstacle
        if (Math.abs(error) <= GameResources.BAND_WIDTH) {
          GameResources.leftMotor.forward();
          GameResources.rightMotor.forward();
          GameResources.leftMotor.setSpeed(speed);
          GameResources.rightMotor.setSpeed(speed);
        }
        // correction has to be made
        else {
          // robot is too far from wall
          if (error > 0) {
            // imminent collision
            if (error > GameResources.MAXIMAL_ERROR) {
              GameResources.leftMotor.backward();
              GameResources.rightMotor.forward();
              leftSpeed = speed;
              rightSpeed = speed;
            }
            // normal adjustment
            else {
              GameResources.leftMotor.forward();
              GameResources.rightMotor.forward();
              leftSpeed = (int) (speed - this.calculateGain(error));
              rightSpeed = (int) (speed + this.calculateGain(error));
            }
          }
          // robot is too close to wall
          else {
            if (error < GameResources.MINIMAL_ERROR) {
              GameResources.leftMotor.forward();
              GameResources.rightMotor.backward();
              leftSpeed = speed;
              rightSpeed = speed;
            }
            // normal adjustment
            else {
              GameResources.leftMotor.forward();
              GameResources.rightMotor.forward();
              leftSpeed = (int) (speed + this.calculateGain(error));
              rightSpeed = (int) (speed - this.calculateGain(error));
            }
          }
          // set bounds on the speed during avoidance
          if (leftSpeed <= GameResources.MIN_AVOID_SPEED) {
            leftSpeed = GameResources.MIN_AVOID_SPEED;
          } else if (leftSpeed >= GameResources.MAX_AVOID_SPEED) {
            leftSpeed = GameResources.MAX_AVOID_SPEED;
          }
          if (rightSpeed <= GameResources.MIN_AVOID_SPEED) {
            rightSpeed = GameResources.MIN_AVOID_SPEED;
          } else if (rightSpeed >= GameResources.MAX_AVOID_SPEED) {
            rightSpeed = GameResources.MAX_AVOID_SPEED;
          }
          GameResources.leftMotor.setSpeed((int) leftSpeed);
          GameResources.rightMotor.setSpeed((int) rightSpeed);
          endTime = System.currentTimeMillis();
          if (endTime - startTime < GameResources.OBSTACLE_AVOIDANCE_PERIOD) {
            try {
              Thread.sleep((long) (GameResources.OBSTACLE_AVOIDANCE_PERIOD - (endTime - startTime)));
            } catch (InterruptedException e) {
              // there is nothing to be done
            }
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
  public static boolean orientationCheck() {
    double currentX = GameResources.odometer.getX();
    double currentY = GameResources.odometer.getY();
    double goalX = GameResources.getNavigationCoordinates().x * GameResources.TILE_SIZE;
    double goalY = GameResources.getNavigationCoordinates().y * GameResources.TILE_SIZE;
    double dX = goalX - currentX;
    double dY = goalY - currentY;
    double turnToAngle = Math.toDegrees(Math.atan2(dX, dY));
    double rotation = turnToAngle -GameResources.odometer.getTheta();

    // Adjust the rotation of the robot to make sure it turns the minimal angle possible
    if (rotation > 180) {
      rotation -= 360;
    } else if (rotation < -180) {
      rotation += 360;
    } else if (Math.abs(rotation) == 180) {
      rotation = 180;
    }

    if (Math.abs(rotation) <= GameResources.ORIENTATION_CHECK_ERROR) {
      return true;
    }
    return false;
  }

  /**
   * Method used to calculate the gain to apply to the motor speed in function of the error between the wall distance
   * and the band center
   * 
   * @param : the error between the wall distance and the band center
   * @return : the gain to apply to the motor speeds
   */
  private double calculateGain(double error) {
    double absError = Math.abs(error);
    double gain = absError * GameResources.GAIN_CONSTANT;
    return gain;
  }

  
  /**
   * Method used to calculate the lateral distance between the robot and the wall during wall following
   * 
   * @param : the distance seen by the left ultrasonic sensor
   * @return : the lateral distance between the robot and the wall
   */
 private double calculateLateralDistance(double distance) {// method used to approximate the lateral distance of the robot from
                                                    // the wall
    // lateral distance = distance *cos(45)
    // 4 centimeters between sensor and wheels
    double lateralDistance = distance * Math.acos(Math.PI / 4);
    return lateralDistance;
  }

 /**
  * Method used to perform the convex corner procedure during wall following to avoid crashing into the walls
  */
 public void convexCornerProcedure() {
   // convex corner procedure
   Navigation.travel(GameResources.CONVEX_CORNER_ADJUSTMENT_DISTANCE / 2, GameResources.FORWARD_SPEED_FAST);
   double currentTheta = GameResources.odometer.getTheta();
   double goalTheta = currentTheta - 90;
   if (goalTheta < 0) {
     goalTheta = goalTheta + 360;
   }
   // 90 degrees turn towards the left
   Navigation.rotate(Turn.COUNTER_CLOCK_WISE, GameResources.ROTATE_SPEED_SLOW);
   while (!orientationCheck()) {
     if (Math.abs(GameResources.odometer.getTheta() - goalTheta) <= 1) {
       Navigation.travel(GameResources.CONVEX_CORNER_ADJUSTMENT_DISTANCE, GameResources.FORWARD_SPEED_FAST);
       break;
     }
   }
   return;

   
 }
  public int getObstacleDistance() {
    return obstacleDistance;
  }



}
