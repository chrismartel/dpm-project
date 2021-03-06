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



  /**
   * Method that will correct either the X or Y value of the odometer as well as the theta value. Uses the current theta
   * value and the accepted theta ranged from game resources. Uses the nearest line to commpute which value it should
   * correct to. Afterwards, corrects either X or Y and the Theta.
   */
  public static void correctValues() {
    // This is to ensure that even if the robot is not exactly straight it will still detect its general heading.
    double lowerBound = GameResources.odometer.getTheta() - GameResources.THETA_RANGE;
    double upperBound = GameResources.odometer.getTheta() + GameResources.THETA_RANGE;



    if ((lowerBound >= (0 - GameResources.THETA_RANGE) && upperBound <= (0 + (2 * GameResources.THETA_RANGE)))
        || (lowerBound >= (360 - (2 * GameResources.THETA_RANGE)) && upperBound <= (360 + GameResources.THETA_RANGE))) {
      // The robot is going forward

      //This if statement is in case the robot detects the crack in-between two boards.
      if ((((GameResources.odometer.getY() - GameResources.OFFSET_FROM_WHEELBASE) / GameResources.TILE_SIZE <= 4.6
          && (GameResources.odometer.getY() - GameResources.OFFSET_FROM_WHEELBASE) / GameResources.TILE_SIZE >= 4.4))) {
        GameResources.odometer.setTheta(0);
        return;

      }
      // The current Y position of the light sensor
      double currentYLine = GameResources.odometer.getY() - GameResources.OFFSET_FROM_WHEELBASE;
      int lineCount = (int) Math.round(currentYLine / GameResources.TILE_SIZE);
      GameResources.odometer.setY(lineCount * GameResources.TILE_SIZE + GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(0);
    }

    else if (lowerBound >= (90 - (2 * GameResources.THETA_RANGE))
        && upperBound <= (90 + (2 * GameResources.THETA_RANGE))) {
      // going right
      // The current X position of the light sensor
      double currentXLine = GameResources.odometer.getX() - GameResources.OFFSET_FROM_WHEELBASE;
      // The current line count given the X position
      int lineCount = (int) Math.round(currentXLine / GameResources.TILE_SIZE);
      GameResources.odometer.setX(lineCount * GameResources.TILE_SIZE + GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(90);

    } else if (lowerBound >= (180 - (2 * GameResources.THETA_RANGE))
        && upperBound <= (180 + (2 * GameResources.THETA_RANGE))) {
      
      //This if statement is in case the robot detects the crack in-between two boards.
      if ((((GameResources.odometer.getY() + GameResources.OFFSET_FROM_WHEELBASE) / GameResources.TILE_SIZE <= 4.6
          && (GameResources.odometer.getY() + GameResources.OFFSET_FROM_WHEELBASE) / GameResources.TILE_SIZE >= 4.4))) {
        GameResources.odometer.setTheta(180);
        return;

      }
      // The current Y position of the light sensor
      double currentYLine = GameResources.odometer.getY() + GameResources.OFFSET_FROM_WHEELBASE;
      // The current line count given the Y position
      int lineCount = (int) Math.round(currentYLine / GameResources.TILE_SIZE);
      GameResources.odometer.setY(lineCount * GameResources.TILE_SIZE - GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(180);
    } else if (lowerBound >= (270 - (2 * GameResources.THETA_RANGE))
        && upperBound <= (270 + (2 * GameResources.THETA_RANGE))) {
      // going left
      // The current X position of the light sensor
      double currentXLine = GameResources.odometer.getX() + GameResources.OFFSET_FROM_WHEELBASE;
      // The current line count given the X position
      int lineCount = (int) Math.round(currentXLine / GameResources.TILE_SIZE);
      GameResources.odometer.setX(lineCount * GameResources.TILE_SIZE - GameResources.OFFSET_FROM_WHEELBASE);
      GameResources.odometer.setTheta(270);
    }
  }

}
