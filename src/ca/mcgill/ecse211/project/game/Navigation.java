package ca.mcgill.ecse211.project.game;



import ca.mcgill.ecse211.project.odometry.OdometryCorrection;

/**
 * The navigation class is used to define all the low level navigation movements of the robot.
 */
public class Navigation {

  /**
   * last coordinates recorded from the odometer
   */
  private static double lastX;
  private static double lastY;


  private static Navigation nav; // Returned as singleton

  public static enum Turn {
    CLOCK_WISE, COUNTER_CLOCK_WISE
  }


  /**
   * Returns the Navigation Object. Use this method to obtain an instance of Navigation. Method used to make sure there
   * is just one instance of Navigation throughout the code
   * 
   * @return the Navigation Object
   */
  public synchronized static Navigation getNavigation() {
    if (nav == null) {
      nav = new Navigation();
    }

    return nav;
  }

  /**
   * This method causes the robot to travel to the current way point location
   * 
   * @param x : x coordinate of the current way point
   * @param y : y coordinate of the current way point
   */
  public static void travelTo(double x, double y, int speed) {
    // Convert the coordinates to centimeters
    if (!GameResources.isObstacleDetected()) {
      x = x * GameResources.TILE_SIZE;
      y = y * GameResources.TILE_SIZE;
      GameResources.leftMotor.setSpeed(speed);
      GameResources.rightMotor.setSpeed(speed);
      // Poll the odometer for info about current the position
      double currentX = GameResources.odometer.getX();
      double currentY = GameResources.odometer.getY();

      // compute the distances to travel in X and Y
      double dX = x - currentX;
      double dY = y - currentY;

      // the robot turns by the angle between its current orientation and the orientation towards the current way point
      turnTo(Math.toDegrees(Math.atan2(dX, dY)), speed);

      // the robot travels to the way point
      travel(dX, dY, speed);
    }
  }


  public static void turnTo(double x, double y, int speed) {
    // Convert the coordinates to centimeters
    x = x * GameResources.TILE_SIZE;
    y = y * GameResources.TILE_SIZE;

    // Poll the odometer for info about current the position
    double currentX = GameResources.odometer.getX();
    double currentY = GameResources.odometer.getY();

    // compute the distances to travel in X and Y
    double dX = x - currentX;
    double dY = y - currentY;

    turnTo(Math.toDegrees(Math.atan2(dX, dY)), speed);
  }

  /**
   * This method causes the robot to turn to a specific orientation using the minimum angle possible
   * 
   * @param theta :
   */
  public static void turnTo(double theta, int speed) {
    double currentTheta = GameResources.odometer.getTheta();
    // compute the difference between the current orientation and the desired orientation
    double rotation = theta - currentTheta;

    // Adjust the rotation of the robot to make sure it turns the minimal angle possible
    if (rotation > 180) {
      rotation -= 360;
    } else if (rotation < -180) {
      rotation += 360;
    } else if (Math.abs(rotation) == 180) {
      rotation = 180;
    }

    // rotate to the calculated angle
    turn(rotation, speed);
  }


  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance : the distance to cover
   * @return the degrees of rotation to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * GameResources.WHEEL_RADIUS));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle : the angle to obtain
   * @return the degrees of rotation to reach this angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * GameResources.TRACK * angle / 360.0);
  }


  /**
   * Travels a distance from x and y information
   * 
   * @param x : the x distance to travel
   * @param y : the y distance to travel
   */
  public static void travel(double x, double y, int speed) {
    travel(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)), speed);
  }

  /**
   * Method causing the robot to travel forward a until the odometer reaches the travel distance
   * 
   * @param travelDistance : the magnitude of the distance to travel in cm
   */
  public static void travel(double travelDistance, int speed) {
    double dX, dY;

    // poll the odometer to get the current X and Y coordinates
    lastX = GameResources.odometer.getX();
    lastY = GameResources.odometer.getY();

    // robot travels forward
    travelForward(speed);

    // navigates as long as the state is not in avoidance
    while (!GameResources.isObstacleDetected()) {
      dX = Math.abs(GameResources.odometer.getX() - lastX);
      dY = Math.abs(GameResources.odometer.getY() - lastY);
      // Reached goal coordinates condition
      if (Math.pow(dX, 2) + Math.pow(dY, 2) >= Math.pow(travelDistance, 2)) {
        // navigation is considered completed when the distance is reached while being in navigation state
        if (GameResources.getGameState() == GameState.Navigation) {
          GameResources.setNavigationCompleted(true);
        }
        // exit the travel method if the destination is reached
        break;
      }
      if (GameResources.isEnableCorrection()) {
        if (!GameResources.lightLocalizer.lightLocalize()) {
          OdometryCorrection.correctValues();
          // System.out.println("x: "+odometer.getX());
          // System.out.println("y: "+odometer.getY());
          Navigation.travelForward(speed);
        }
      }
    }
    stopMotors();

  }

  /**
   * Method causing the robot to travel backward a until the odometer reaches the travel distance
   * 
   * @param travelDistance : the magnitude of the distance to travel
   */
  public static void backUp(double travelDistance, int speed) {
    double dX, dY;

    // poll the odometer to get the current X and Y coordinates
    lastX = GameResources.odometer.getX();
    lastY = GameResources.odometer.getY();

    // robot travels forward
    travelBackward(speed);

    while (true) {
      dX = Math.abs(GameResources.odometer.getX() - lastX);
      dY = Math.abs(GameResources.odometer.getY() - lastY);
      // Reached goal coordinates condition
      if (Math.pow(dX, 2) + Math.pow(dY, 2) >= Math.pow(travelDistance, 2)) {

        // robot stops if its destination is reached
        stopMotors();

        // exit the travel method if the destination is reached
        break;
      }

    }
  }



  /**
   * set both motors forward
   */
  public static void travelForward(int speed) {

    GameResources.leftMotor.setSpeed(speed);
    GameResources.rightMotor.setSpeed(speed);
    GameResources.rightMotor.forward();
    GameResources.leftMotor.forward();

  }

  /**
   * Set both motors backward
   */
  public static void travelBackward(int speed) {
    GameResources.leftMotor.setSpeed(speed);
    GameResources.rightMotor.setSpeed(speed);
    GameResources.rightMotor.backward();
    GameResources.leftMotor.backward();
  }

  /**
   * Turn by a certain amount of degrees
   * 
   * @param : angle to turn to. If theta < 0, the robot turns counter clock wise if theta >0, the robot turns clock wise
   */
  public static void turn(double theta, int speed) {
    GameResources.leftMotor.setSpeed(speed);
    GameResources.rightMotor.setSpeed(speed);
    GameResources.leftMotor.rotate(+convertAngle(theta), true);// doesn't wait for the motor to complete the rotation
    GameResources.rightMotor.rotate(-convertAngle(theta), false);
  }


  /**
   * Rotate continuously in one direction
   * 
   * @param direction Direction which the robot needs to turn (CLOCK_WISE, COUNTER_CLOCK_WISE)
   * 
   */
  public static void rotate(Turn direction, int speed) {
    GameResources.leftMotor.setSpeed(speed);
    GameResources.rightMotor.setSpeed(speed);
    switch (direction) {
      case CLOCK_WISE:
        GameResources.rightMotor.backward();
        GameResources.leftMotor.forward();
        break;
      case COUNTER_CLOCK_WISE:
        GameResources.rightMotor.forward();
        GameResources.leftMotor.backward();
        break;
    }
  }



  /**
   * stop both motors at once
   */
  public static void stopMotors() {
    GameResources.rightMotor.setSpeed(0);// does not wait for the motor to actually stop
    GameResources.leftMotor.setSpeed(0);
  }



}
