package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.game.GameResources.*;
import java.util.LinkedList;
import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.game.GameResources.REGION;
import lejos.hardware.Button;
import lejos.hardware.Sound;


/**
 * This class implements all the methods related to map generation, navigation to different locations on the field,
 * tunnel computations or distance computations
 * 
 */
public class GameNavigation {
  /**
   * coordinates of the tunnel entrance and exit
   */
  private Point tunnelEntrance;
  private Point tunnelExit;

  /**
   * coordinates of the launch point
   */
  private Point launchPoint;

  /**
   * array of launch points on the island
   */
  private LinkedList<Point> launchPoints = new LinkedList<Point>();



  /**
   * Orientation the robot needs to have to traverse the tunnel
   */
  private double tunnelTraversalOrientation;

  public GameNavigation() {
    tunnelEntrance = new Point(0, 0);
    tunnelExit = new Point(0, 0);
    launchPoint = new Point(0, 0);
  }



  /**
   * Method used to travel on the field by following the black lines directions, it enables the correction it the
   * travel() method in the Navigation class. It allows heading correction to be peform each time 2 lines are detected.
   * 
   * @param : x is the goal coordinate on the x axis
   * @param : y is the goal coordinate on the y axis
   */
  public void squareNavigation(double x, double y) {
    enableCorrection = true;
    Navigation.travelTo(x, (odometer.getY() / TILE_SIZE), FORWARD_SPEED_NORMAL);
    Navigation.travelTo((odometer.getX() / TILE_SIZE), y, FORWARD_SPEED_NORMAL);
    enableCorrection = false;
  }


  /**
   * Method implementing the navigation to the tunnel entrance and the turn towards the tunnel traversal orientation
   */
  public void navigateToTunnel() {
    this.squareNavigation(tunnelEntrance.x, tunnelEntrance.y);
    Navigation.turnTo(tunnelTraversalOrientation, ROTATE_SPEED_SLOW);
  }

  /**
   * Method implementing the navigation to the launch and the turn towards the bin
   */
  public void navigateToLaunchPoint() {
    // navigate to launch point
    this.squareNavigation(launchPoint.x, launchPoint.y);
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    double binX = bin.x * TILE_SIZE;
    double binY = bin.y * TILE_SIZE;
    double dX = binX - currentX;
    double dY = binY - currentY;
    // turn towards launch point
    Navigation.turnTo(Math.toDegrees(Math.atan2(dX, dY)), ROTATE_SPEED_SLOW);
  }

  /**
   * Method implementing the navigation through a tunnel. At first, the robot travels to the exit of the tunnel, then it
   * adjust its heading according to the next black line and backs up. Also, after the tunnel traversal, the map
   * parameters are updated.
   */
  public void navigateThroughTunnel() {
    Navigation.travelTo(tunnelExit.x, tunnelExit.y, FORWARD_SPEED_FAST);
    LightLocalizer.twoLineDetection();
    Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_FAST);
    this.updateParameters();
  }

  /**
   * Method returning the region of a specific tile
   * 
   * @param x : the x coordinate of the center of the tile
   * @param y : the y coordinate of the center of the tile
   * @return the region type of the tile
   */
  public REGION regionCalculation(double x, double y) {

    if (x <= red.ur.x && x >= red.ll.x && y <= red.ur.y && y >= red.ll.y) {
      return REGION.RED;
    }

    else if (x <= green.ur.x && x >= green.ll.x && y <= green.ur.y && y >= green.ll.y) {
      return REGION.GREEN;
    }

    else if (x <= island.ur.x && x >= island.ll.x && y <= island.ur.y && y >= island.ll.y) {
      return REGION.ISLAND;
    }

    else if (x <= tnr.ur.x && x >= tnr.ll.x && y <= tnr.ur.y && y >= tnr.ll.y) {
      return REGION.TUNNEL_RED;
    }

    else if (x <= tng.ur.x && x >= tng.ll.x && y <= tng.ur.y && y >= tng.ll.y) {
      return REGION.TUNNEL_GREEN;
    }

    return REGION.WATER;
  }

  /**
   * Method used to find the closest coordinate point from the actual position of the odometer. The method return the
   * closest point from the current position of the robot which is not on a border of the current region the robot is
   * on.
   * 
   * @return the closest coordinate point from the current odometer position which is not on a region border
   */
  public Point closestPoint() {
    double x = odometer.getX();
    double y = odometer.getY();
    int x1 = (int) (x / TILE_SIZE);
    int y1 = (int) (y / TILE_SIZE);
    int x2 = (int) x1 + 1;
    int y2 = (int) y1 + 1;
    // array of closest points
    LinkedList<Point> closestPoints = new LinkedList<Point>();
    Point p1 = new Point(x1, y1);
    Point p2 = new Point(x1, y2);
    Point p3 = new Point(x2, y1);
    Point p4 = new Point(x2, y2);

    // check if the 4 points generated are localizable
    // if they are, add them to the array
    if (GameNavigation.localizablePoint(p1))
      closestPoints.add(p1);
    if (GameNavigation.localizablePoint(p2))
      closestPoints.add(p2);
    if (GameNavigation.localizablePoint(p3))
      closestPoints.add(p3);
    if (GameNavigation.localizablePoint(p4))
      closestPoints.add(p4);

    // compute distances of 4 points from actual position
    x = odometer.getX() / TILE_SIZE;
    y = odometer.getY() / TILE_SIZE;

    Point minimalPoint = closestPoints.get(0);
    double minimalDistance = this.calculateDistance(x, y, minimalPoint.x, minimalPoint.y);

    for (Point point : closestPoints) {
      double distance = this.calculateDistance(x, y, point.x, point.y);
      if (distance < minimalDistance) {
        minimalPoint = point;
        minimalDistance = distance;
      }
    }
    return minimalPoint;
  }

  /**
   * Method used to calculate the distance between 2 points
   * 
   * @param x1: x coordinate of the first point
   * @param y1: y coordinate of the first point
   * @param x2: x coordinate of the second point
   * @param y2: y coordinate of the second point
   * @return the distance between 2 points in centimeters
   */
  public double calculateDistance(double x1, double y1, double x2, double y2) {
    // convert into centimeters
    x1 = x1 * TILE_SIZE;
    y1 = y1 * TILE_SIZE;
    x2 = x2 * TILE_SIZE;
    y2 = y2 * TILE_SIZE;
    double distance = Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    return distance;
  }

  /**
   * Method used to find the tunnel entrance by evaluating the region of the points around the tunnel. Once the method
   * has found the entrance, it sets the tunnel entrance, exit and length data
   */
  public void updateTunnelData() {

    REGION targetRegion;
    REGION tunnelBottom = this.regionCalculation((Tunnel.ll.x + 0.5), (Tunnel.ll.y - 0.5));
    REGION tunnelTop = this.regionCalculation((Tunnel.ur.x - 0.5), (Tunnel.ur.y + 0.5));
    REGION tunnelLeft = this.regionCalculation((Tunnel.ll.x - 0.5), (Tunnel.ll.y + 0.5));
    REGION tunnelRight = this.regionCalculation((Tunnel.ur.x + 0.5), (Tunnel.ur.y - 0.5));

    // determine the target region, the region of the tunnel entrance
    if (currentRegion == REGION.RED) {
      targetRegion = REGION.RED;
    } else if (currentRegion == REGION.GREEN) {
      targetRegion = REGION.GREEN;
    } else {
      targetRegion = REGION.ISLAND;
    }
    // determine where is the tunnel entrance
    if (tunnelBottom == targetRegion) {
      this.tunnelEntrance.x = (Tunnel.ll.x + 0.5);
      this.tunnelEntrance.y = (Tunnel.ll.y - 1);
      this.tunnelExit.x = (Tunnel.ur.x - 0.5);
      this.tunnelExit.y = (Tunnel.ur.y + 1);
      this.tunnelTraversalOrientation = 0;
      // this.tunnelLength = (TUNNEL_UR.y - TUNNEL_LL.y) * TILE_SIZE;
    } else if (tunnelTop == targetRegion) {
      this.tunnelEntrance.x = (Tunnel.ur.x - 0.5);
      this.tunnelEntrance.y = (Tunnel.ur.y + 1);
      this.tunnelExit.x = (Tunnel.ll.x + 0.5);
      this.tunnelExit.y = (Tunnel.ll.y - 1);
      this.tunnelTraversalOrientation = 180;
      // this.tunnelLength = (TUNNEL_UR.y - TUNNEL_LL.y) * TILE_SIZE;
    } else if (tunnelLeft == targetRegion) {
      this.tunnelEntrance.x = (Tunnel.ll.x - 1);
      this.tunnelEntrance.y = (Tunnel.ll.y + 0.5);
      this.tunnelExit.x = (Tunnel.ur.x + 1);
      this.tunnelExit.y = (Tunnel.ur.y - 0.5);
      this.tunnelTraversalOrientation = 90;
      // this.tunnelLength = (TUNNEL_UR.x - TUNNEL_LL.x) * TILE_SIZE;
    } else if (tunnelRight == targetRegion) {
      this.tunnelEntrance.x = (Tunnel.ur.x + 1);
      this.tunnelEntrance.y = (Tunnel.ur.y - 0.5);
      this.tunnelExit.x = (Tunnel.ll.x - 1);
      this.tunnelExit.y = (Tunnel.ll.y + 0.5);
      this.tunnelTraversalOrientation = 270;
      // this.tunnelLength = (TUNNEL_UR.x - TUNNEL_LL.x) * TILE_SIZE;
    }
  }

  /**
   * Method used to set the tunnel depending on the team color
   */
  public void setTunnel() {
    if (color == COLOR.RED) {
      Tunnel.ll.x = tnr.ll.x;
      Tunnel.ll.y = tnr.ll.y;
      Tunnel.ur.x = tnr.ur.x;
      Tunnel.ur.y = tnr.ur.y;
    } else {
      System.out.println("GREEN TUNNEL SET");

      Tunnel.ll.x = tng.ll.x;
      Tunnel.ll.y = tng.ll.y;
      Tunnel.ur.x = tng.ur.x;
      Tunnel.ur.y = tng.ur.y;
    }
  }

  /**
   * Method used to set the limits of the current region
   */
  public void setLimits() {
    // set the limits depending on the current region
    switch (currentRegion) {
      case GREEN:
        currentLeftLimit = green.ll.x;
        currentRightLimit = green.ur.x;
        currentTopLimit = green.ur.y;
        currentBottomLimit = green.ll.y;
        break;
      case RED:
        currentLeftLimit = red.ll.x;
        currentRightLimit = red.ur.x;
        currentTopLimit = red.ur.y;
        currentBottomLimit = red.ll.y;
        break;
      case ISLAND:
        currentLeftLimit = island.ll.x;
        currentRightLimit = island.ur.x;
        currentTopLimit = island.ur.y;
        currentBottomLimit = island.ll.y;
        break;
      default:
        break;

    }
  }

  /**
   * Method used to set the color
   */
  public void setColor() {
    if (redTeam == TEAM_NUMBER) {
      color = COLOR.RED;
    } else if (greenTeam == TEAM_NUMBER) {
      color = COLOR.GREEN;
      System.out.println("GREEN TEAM SET");
    }

  }

  /**
   * Method used to set the staring region depending on the team color
   */
  public void setStartingRegion() {
    if (color == COLOR.GREEN) {
      currentRegion = REGION.GREEN;
      System.out.println("STARTING REGION SET TO GREEN");

    } else {
      currentRegion = REGION.RED;
    }
  }

  /**
   * Method used to update the region after a tunnel traversal depending on the current region
   */
  public void updateRegion() {
    if (currentRegion == REGION.GREEN || currentRegion == REGION.RED) {
      currentRegion = REGION.ISLAND;
    } else if (currentRegion == REGION.ISLAND) {
      if (color == COLOR.GREEN) {
        currentRegion = REGION.GREEN;
      } else {
        currentRegion = REGION.RED;
      }
    }

  }

  /**
   * Method setting the coordinates of the starting corner depending on the team color and on the corner of the team
   * color
   */
  public void setCorner() {
    if (color == COLOR.GREEN) {
      switch (greenCorner) {
        // lower left corner
        case 0:
          CORNER_NUMBER = 0;
          STARTING_POINT = new Point(1, 1);
          break;
        // lower right corner
        case 1:
          System.out.println("GREEN CORNER SET TO 1");

          CORNER_NUMBER = 1;
          STARTING_POINT = new Point(FIELD_RIGHT - 1, 1);
          System.out.println("STARTING POINT: " + STARTING_POINT.x + ", " + STARTING_POINT.y);

          break;
        // top right corner
        case 2:
          System.out.println("GREEN CORNER SET TO 2");

          CORNER_NUMBER = 2;
          STARTING_POINT = new Point(FIELD_RIGHT - 1, FIELD_TOP - 1);
          System.out.println("STARTING POINT: " + STARTING_POINT.x + ", " + STARTING_POINT.y);

          break;
        // top left corner
        case 3:
          System.out.println("GREEN CORNER SET TO 3");

          CORNER_NUMBER = 3;
          STARTING_POINT = new Point(1, FIELD_TOP - 1);
          System.out.println("STARTING POINT: " + STARTING_POINT.x + ", " + STARTING_POINT.y);

          break;
      }
    } else {
      switch (redCorner) {
        // lower left corner
        case 0:
          CORNER_NUMBER = 0;
          STARTING_POINT = new Point(1, 1);
          break;
        // lower right corner
        case 1:
          CORNER_NUMBER = 1;
          STARTING_POINT = new Point(FIELD_RIGHT - 1, 1);
          break;
        // top right corner
        case 2:
          CORNER_NUMBER = 2;
          STARTING_POINT = new Point(FIELD_RIGHT - 1, FIELD_TOP - 1);
          break;
        // top left corner
        case 3:
          CORNER_NUMBER = 3;
          STARTING_POINT = new Point(1, FIELD_TOP - 1);
          break;
      }

    }
  }

  /**
   * Method used to compute the distance of the robot from the bin
   * 
   * @param x: x coordinate of the point
   * @param y: y coordinate of the point
   * 
   * @return: the distance between the robot and the bin in centimeters
   */
  public double distanceFromBin(double x, double y) {
    double distance = this.calculateDistance(x, y, bin.x, bin.y);
    return distance;
  }

  /**
   * Method used to set the initial map parameters
   */
  public void setGameParameters() {
    this.setColor();
    this.setStartingRegion();
    this.setCorner();
    this.setLimits();
    this.setTunnel();
    this.updateTunnelData();
    this.generateLaunchPoints();
  }

  /**
   * Method used to update the map parameters after a tunnel traversal
   */
  public void updateParameters() {
    // update the current region of the robot
    this.updateRegion();
    // update the tunnel data
    this.updateTunnelData();
    // set the new zone limits
    this.setLimits();
  }


  /**
   * Method used to calculate the closest possible launch point from the robot and to set the current launch point of
   * the game navigation instance
   */
  public void calculateClosestLaunchPoint() {
    double x = odometer.getX() / TILE_SIZE;
    double y = odometer.getY() / TILE_SIZE;
    Point minimal_point = launchPoints.get(0);
    double minimal_distance = this.calculateDistance(x, y, minimal_point.x, minimal_point.y);
    for (Point point : launchPoints) {
      double distance = this.calculateDistance(x, y, point.x, point.y);
      if (distance <= minimal_distance) {
        minimal_distance = distance;
        minimal_point = point;
      }
    }
    this.launchPoint = minimal_point;
  }


  /**
   * Method used to populate the linked list of possible launch points on the island of the game navigation instance. A
   * possible launch point is a point that is not on the borders of the current region, that is within the maximal
   * distance from the bin and that is not in the restricted linked list of points.
   */
  public void generateLaunchPoints() {
    // set limits of launching zone
    int left = (int) island.ll.x + 1;
    int right = (int) island.ur.x - 1;
    int bottom = (int) island.ll.y + 1;
    int top = (int) island.ur.y - 1;
    // initialize or clear the list
    launchPoints = new LinkedList<Point>();
    // for all inside x values of the island
    for (int i = left; i <= right; i++) {
      // for all inside y values of the island
      for (int j = bottom; j <= top; j++) {
        Point point = new Point(i, j);
        double distance = this.distanceFromBin(i, j);
        // check if the distance from the bin is within the maximal distance
        if (distance <= MAXIMAL_LAUNCH_DISTANCE) {
          boolean restricted = false;
          // check if the point is restricted
          for (Point restrictedPoint : restrictedPoints) {
            // if the point is restricted
            if ((restrictedPoint.x == point.x) && (restrictedPoint.y == point.y)) {
              restricted = true;
            }
          }
          // add the point to the list only if it is not restricted
          if (!restricted) {
            launchPoints.add(point);
          }
        }
      }
    }
  }

  /**
   * Method that checks if a point is on the border of the current zone. This method is used to filter out the
   * localization points. A localizable point can't be on a zone limit and can't be in the restricted points array.
   * 
   * @return: true if the point is not on any border and is localizable, false if the point is on a border and is not
   *          localizable
   */
  public static boolean localizablePoint(Point point) {
    // check if the point is on one of the current zone limits
    if ((point.x != currentLeftLimit) && (point.x != currentRightLimit) && (point.y != currentBottomLimit)
        && (point.y != currentTopLimit)) {
      for (Point restrictedPoint : restrictedPoints) {
        // verification that the input point is not in the restricted point array
        if ((point.x == restrictedPoint.x) && (point.y == restrictedPoint.y)) {
          return false;
        }
      }
      return true;
    }
    return false;
  }


  /**
   * Getter Method for the tunnel entrance
   * 
   * @return: the point corresponding to the tunnel entrance
   */
  public Point getTunnelEntrance() {
    return tunnelEntrance;
  }

  /**
   * Getter Method for the tunnel exit
   * 
   * @return: the point corresponding to the tunnel exit
   */
  public Point getTunnelExit() {
    return tunnelExit;
  }



  /**
   * Getter Method for the tunnel traversal orientation
   * 
   * @return: the orientation needed to traverse the tunnel
   */
  public double getTunnelTraversalOrientation() {
    return tunnelTraversalOrientation;
  }



}
