package ca.mcgill.ecse211.project.odometry;

import ca.mcgill.ecse211.project.game.GameResources;

public class OdometryCorrection {

  /**
   * Booleans indicating if either sensor is currently detecting a line.
   */

  boolean leftCurrentlyOnLine = false;
  boolean rightCurrentlyOnLine = false;

  /**
   * Start and end times to accurately time our correction.
   */
  long startTime;
  long endTime;

  double[] leftValues = null;
  double[] rightValues = null;


  public static void correctValues() {
    int difference = 5;
    double lowerBound = GameResources.odometer.getTheta() - difference;
    double upperBound = GameResources.odometer.getTheta() + difference;

    if ((lowerBound >= (0 - difference) && upperBound <= (0 + (2 * difference)))
        || (lowerBound >= (360 - (2 * difference)) && upperBound <= (360 + difference))) {
      // The robot is going forward
      double currentYLine = GameResources.odometer.getY() - GameResources.OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentYLine / GameResources.TILE_SIZE);
      GameResources.odometer.setY(lineCount * GameResources.TILE_SIZE + GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(0);
      // System.out.println("GOING FORWARD");
    }

    else if (lowerBound >= (90 - (2 * difference)) && upperBound <= (90 + (2 * difference))) {
      // going right
      double currentXLine = GameResources.odometer.getX() - GameResources.OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentXLine / GameResources.TILE_SIZE);
      // System.out.println("line count" + lineCount);
      GameResources.odometer.setX(lineCount * GameResources.TILE_SIZE + GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(90);
      // System.out.println("GOING right");
    } else if (lowerBound >= (180 - (2 * difference)) && upperBound <= (180 + (2 * difference))) {
      // going backward
      double currentYLine = GameResources.odometer.getY() + GameResources.OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentYLine / GameResources.TILE_SIZE);
      GameResources.odometer.setY(lineCount * GameResources.TILE_SIZE - GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(180);
      // System.out.println("GOING backward");
    } else if (lowerBound >= (270 - (2 * difference)) && upperBound <= (270 + (2 * difference))) {
      // going left
      double currentXLine = GameResources.odometer.getX() + GameResources.OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentXLine / GameResources.TILE_SIZE);
      GameResources.odometer.setX(lineCount * GameResources.TILE_SIZE - GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(270);
      // System.out.println("GOING left");
    }
    // System.out.println("ODOMETRY CORRECTION"+ odometer.getX()/TILE_SIZE+ ", "+ odometer.getY()/TILE_SIZE+ ", ");
  }



  // when one line is detected, save odometer X, Y, Theta
  // when the other sensor detects the line, save Odometer X, Y, Theta.
  // Do math to compute the offset of the robot
  // Change Odometer reading depending on the offset.



}
