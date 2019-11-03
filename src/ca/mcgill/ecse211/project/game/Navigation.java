package ca.mcgill.ecse211.project.game;



import static ca.mcgill.ecse211.project.game.Resources.*;

/**
 * The navigation class is used to define all the low level navigation movements of the robot.
 */
public class Navigation {

  /**
   * last coordinates recorded from the odometer
   */
  private static double lastX;
  private static double lastY;

  /**
   * boolean representing if the robot is traveling to a way point it is set to false ONLY when a way point is reached
   */
  private static boolean navigating = false;

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
  public static void travelTo(double x, double y) {

    // Convert the coordinates to centimeters
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;

    // Poll the odometer for info about current the position
    double currentX = odometer.getX();
    double currentY = odometer.getY();

    // compute the distances to travel in X and Y
    double dX = x - currentX;
    double dY = y - currentY;

    // the robot turns by the angle between its current orientation and the orientation towards the current way point
    turnTo(Math.toDegrees(Math.atan2(dX, dY)));

    // the robot travels to the way point
    travel(dX, dY);

  }


  public static void turnTo(double x, double y) {
    // Convert the coordinates to centimeters
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;

    // Poll the odometer for info about current the position
    double currentX = odometer.getX();
    double currentY = odometer.getY();

    // compute the distances to travel in X and Y
    double dX = x - currentX;
    double dY = y - currentY;

    turnTo(Math.toDegrees(Math.atan2(dX, dY)));
  }

  /**
   * This method causes the robot to turn to a specific orientation using the minimum angle possible
   * 
   * @param theta :
   */
  public static void turnTo(double theta) {
    double currentTheta = odometer.getTheta();
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
    turn(rotation, ROTATE_SPEED);
  }


  /**
   * checks the navigating state
   * 
   * @return navigating : the navigation state of the robot
   */
  public boolean isNavigating() {
    return navigating;
  }

  /**
   * Sets the navigating state
   * 
   * @param isNavigating : true if the robot is navigating towards a way point, false if the robot has reached a way
   *        point
   */
  public void setNavigating(boolean isNavigating) {
    navigating = isNavigating;
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance : the distance to cover
   * @return the degrees of rotation to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RADIUS));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   * 
   * @param angle : the angle to obtain
   * @return the degrees of rotation to reach this angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }


  /**
   * Travels a distance from x and y information
   * 
   * @param x : the x distance to travel
   * @param y : the y distance to travel
   */
  public static void travel(double x, double y) {
    travel(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
  }

  /**
   * Method causing the robot to travel forward a until the odometer reaches the travel distance
   * 
   * @param travelDistance : the magnitude of the distance to travel
   */
  public static void travel(double travelDistance) {
    double dX, dY;

    // poll the odometer to get the current X and Y coordinates
    lastX = odometer.getX();
    lastY = odometer.getY();

    // robot travels forward
    travelForward();

    while (true) {
      dX = Math.abs(odometer.getX() - lastX);
      dY = Math.abs(odometer.getY() - lastY);
      // TODO: if game state is in tunnel--> check if the wall is too close and adjust
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
   * Method causing the robot to travel backward a until the odometer reaches the travel distance
   * 
   * @param travelDistance : the magnitude of the distance to travel
   */
  public  static void backUp(double travelDistance) {
    double dX, dY;

    // poll the odometer to get the current X and Y coordinates
    lastX = odometer.getX();
    lastY = odometer.getY();

    // robot travels forward
    travelBackward();

    while (true) {
      dX = Math.abs(odometer.getX() - lastX);
      dY = Math.abs(odometer.getY() - lastY);

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
  public static void travelForward() {

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    rightMotor.forward();
    leftMotor.forward();

  }

  /**
   * Set both motors backward
   */
  public static void travelBackward() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    rightMotor.backward();
    leftMotor.backward();
  }

  /**
   * Turn by a certain amount of degrees
   * 
   * @param : angle to turn to. If theta < 0, the robot turns counter clock wise if theta >0, the robot turns clock wise
   */
  public static  void turn(double theta, int speed) {
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    leftMotor.rotate(+convertAngle(theta), true);// doesn't wait for the motor to complete the rotation
    rightMotor.rotate(-convertAngle(theta), false);
  }


  /**
   * Rotate continuously in one direction
   * 
   * @param direction Direction which the robot needs to turn (CLOCK_WISE, COUNTER_CLOCK_WISE)
   * 
   */
  public static void rotate(Turn direction, int speed) {
    navigating = true;
    leftMotor.setSpeed(speed);
    rightMotor.setSpeed(speed);
    switch (direction) {
      case CLOCK_WISE:
        rightMotor.backward();
        leftMotor.forward();
        break;
      case COUNTER_CLOCK_WISE:
        rightMotor.forward();
        leftMotor.backward();
        break;
    }
    navigating = false;
  }



  /**
   * stop both motors at once
   */
  public static void stopMotors() {
    rightMotor.stop(true);// does not wait for the motor to actually stop
    leftMotor.stop(false);
  }



}
