package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.Resources;
import ca.mcgill.ecse211.project.game.Navigation.Turn;

public class ObstacleAvoider {

  private int obstacleDistance;


  /**
   * STRATEGY 1 : WALL FOLLOWER Robot avoids obstacle by wall following around them, wall following stops when the robot
   * points to its goal coordinate point
   */

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
      System.out.println("DISTANCE WALL FOLOWER1:" + wallDistance);

      System.out.println("DISTANCE WALL FOLOWER2:" + wallDistance);

      /*
       * // CONVEX CORNER HANDLING if (wallDistance >= GameResources.CONVEX_CORNER_CONSTANT) {
       * System.out.println("CONVEX CORNER");
       * 
       * convexCornerCounter++; // convex corner is faced if (convexCornerCounter > 5) { convexCornerCounter = 0;
       * this.convexCornerProcedure(); // after passing the convex corner, get new distance and restart time counter
       * wallDistance = GameResources.ultrasonicPoller.getLeftUsController().getDistance(); wallDistance =
       * this.calculateLateralDistance(wallDistance);
       * 
       * } }
       */

      // GENERAL WALL FOLLOWING PROCEDURE
      // The distance seen is small
      // else {
      double error = wallDistance - GameResources.BAND_CENTER;
      System.out.println("ERROR: " + error);
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

      }
      // }
      endTime = System.currentTimeMillis();
      if (endTime - startTime < GameResources.OBSTACLE_AVOIDANCE_PERIOD) {
        try {
          Thread.sleep((long) (GameResources.OBSTACLE_AVOIDANCE_PERIOD - (endTime - startTime)));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }



    Navigation.stopMotors();


  }

  /**
   * Method indicating if the orientation of the robot is correct or not during the wall following process
   * 
   * @return true if the orientation of the robot is facing its navigation goal coordinates and if there no object in
   *         front of the robot, false if the orientation is not facing the goal coordinates or if there is an object in
   *         front of the robot
   */
  public static boolean orientationCheck() {
    double currentX = GameResources.odometer.getX();
    double currentY = GameResources.odometer.getY();
    double goalX = GameResources.getNavigationCoordinates().x * GameResources.TILE_SIZE;
    double goalY = GameResources.getNavigationCoordinates().y * GameResources.TILE_SIZE;
    double dX = goalX - currentX;
    double dY = goalY - currentY;
    double turnToAngle = Math.toDegrees(Math.atan2(dX, dY));
    double rotation = turnToAngle - GameResources.odometer.getTheta();

    // Adjust the rotation of the robot to make sure it turns the minimal angle possible
    if (rotation > 180) {
      rotation -= 360;
    } else if (rotation < -180) {
      rotation += 360;
    } else if (Math.abs(rotation) == 180) {
      rotation = 180;
    }
    // if the orientation is with 1 degree from the expected navigation orientation and that there are no objects in
    // front of the robot --> return true
    if (Math.abs(rotation) <= GameResources.ORIENTATION_CHECK_ERROR && GameResources.ultrasonicPoller
        .getFrontUsController().getDistance() > 1.5 * GameResources.OBSTACLE_DETECTION_DISTANCE) {
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
   * Method used to check if the robot is able to avoid on the right without crashing into a wall or falling in the
   * river using its left sensor to wall follow or not
   * 
   * @return : true if the robot can avoid right, false if the robot can't avoid right
   */

  // public boolean avoidRight() {
  //
  //
  //
  // }



  /**
   * STRATEGY 2 : PATH FINDER Robot can navigate in 2 different paths in square navigation, either x axis first or y
   * axis first. If an obstacle is detected, the robot is shifted of a tile and navigates using the other path.
   */


  /**
   * Method used to shift the robot from 1 and a half tile when it detects an object at its front. If the robot is
   * heading towards 0 degrees or 180 degrees, the method checks if the robot is in the left part or the right part of
   * the island to determine if it has to avoid left or right. If the robot is heading towards 90 degrees or 270
   * degrees, it checks if it is in the top part or the bottom part of the island to determine if it has to avoid top or
   * bottom.
   */
  public void shiftRobot() {
    double theta = GameResources.odometer.getTheta();
    //
    double islandWidth = Resources.island.ur.x - Resources.island.ll.x;
    double islandHeight = Resources.island.ur.y - Resources.island.ll.y;
    double islandMiddleX = Resources.island.ur.x - (islandWidth / 2);
    double islandMiddleY = Resources.island.ur.y - (islandHeight / 2);
    System.out.println("island width: " + islandWidth);
    System.out.println("island height: " + islandHeight);
    System.out.println("island middle x: " + islandMiddleX);
    System.out.println("island middle y: " + islandMiddleY);

    // coordinates of the robots position
    double x = GameResources.odometer.getX() / GameResources.TILE_SIZE;
    double y = GameResources.odometer.getY() / GameResources.TILE_SIZE;

    // heading approximately towards 0 degrees
    if ((theta >= 355 && theta <= 360) || (theta >= 0 && theta <= 5)) {
      System.out.println("HEADING UP");

      // robot is in the right part of the island--> avoid left
      if (x >= islandMiddleX) {
        // GameNavigation.squareNavigation(x - GameResources.SHIFT_DISTANCE, y, true,false);
        Navigation.travelTo(x - GameResources.SHIFT_DISTANCE, y, GameResources.FORWARD_SPEED_NORMAL);
      }
      // robot is in the left part of the island --> avoid right
      else {
        // GameNavigation.squareNavigation(x + GameResources.SHIFT_DISTANCE, y, true,false);
        Navigation.travelTo(x + GameResources.SHIFT_DISTANCE, y, GameResources.FORWARD_SPEED_NORMAL);

      }
    }
    // heading approximately towards 90 degrees
    else if (theta >= 85 && theta <= 95) {
      System.out.println("HEADING RIGHT");
      // robot is in the top part of the island --> avoid bottom
      if (y >= islandMiddleY) {
        // GameNavigation.squareNavigation(x, y - GameResources.SHIFT_DISTANCE, false,false);
        Navigation.travelTo(x, y - GameResources.SHIFT_DISTANCE, GameResources.FORWARD_SPEED_NORMAL);

      }
      // robot is in the bottom part of the island --> avoid top
      else {
        // GameNavigation.squareNavigation(x, y + GameResources.SHIFT_DISTANCE, false,false);
        Navigation.travelTo(x, y + GameResources.SHIFT_DISTANCE, GameResources.FORWARD_SPEED_NORMAL);

      }
    }
    // heading approximately towards 180 degrees
    else if (theta >= 175 && theta <= 185) {
      System.out.println("HEADING DOWN");

      if (x >= islandMiddleX) {

        // GameNavigation.squareNavigation(x - GameResources.SHIFT_DISTANCE, y, true,false);
        Navigation.travelTo(x - GameResources.SHIFT_DISTANCE, y, GameResources.FORWARD_SPEED_NORMAL);

      }
      // robot is in the left part of the island --> avoid right
      else {
        // GameNavigation.squareNavigation(x + GameResources.SHIFT_DISTANCE, y, true,false);
        Navigation.travelTo(x + GameResources.SHIFT_DISTANCE, y, GameResources.FORWARD_SPEED_NORMAL);

      }
    }
    // heading approximately towards 270 degrees
    else if (theta >= 265 && theta <= 275) {
      System.out.println("HEADING LEFT");

      // robot is in the top part of the island --> avoid bottom
      if (y >= islandMiddleY) {
        // GameNavigation.squareNavigation(x, y - GameResources.SHIFT_DISTANCE, false,false);
        Navigation.travelTo(x, y - GameResources.SHIFT_DISTANCE, GameResources.FORWARD_SPEED_NORMAL);

      }
      // robot is in the bottom part of the island --> avoid top
      else {
        // GameNavigation.squareNavigation(x, y + GameResources.SHIFT_DISTANCE, false,false);
        Navigation.travelTo(x, y + GameResources.SHIFT_DISTANCE, GameResources.FORWARD_SPEED_NORMAL);

      }
    }


  }
}
