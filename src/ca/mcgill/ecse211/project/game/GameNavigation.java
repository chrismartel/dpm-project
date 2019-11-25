package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.Resources;
import ca.mcgill.ecse211.project.game.GameResources.*;
import lejos.hardware.Sound;
import java.util.LinkedList;
import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;


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
  private int tunnel = 0;

  /**
   * coordinates of the launch point
   */
  private Point launchPoint;

  /**
   * array of launch points on the island
   */
  private LinkedList<Point> launchPoints = new LinkedList<Point>();



  /**
   * Orientation the robot needs to have to traverse the tunnel the first time
   */
  private double tunnelEntranceTraversalOrientation;

  /**
   * Orientation the robot needs to have to traverse the tunnel when coming back
   */
  private double tunnelExitTraversalOrientation;

  public GameNavigation() {
    tunnelEntrance = new Point(0, 0);
    tunnelExit = new Point(0, 0);
    launchPoint = new Point(0, 0);
  }



  /**
   * Method used to travel on the field by following the black lines directions, it enables the correction it the
   * travel() method in the Navigation class. It allows heading correction to be perform each time 2 lines are detected.
   * Each time this method is called, the localized static boolean is set to false. This method has 2 options: either it
   * travels on the x axis first or either it travels on the y axis first.
   * 
   * @param : x is the goal coordinate on the x axis
   * @param : y is the goal coordinate on the y axis
   * @param : xFirst, if true, the method travels on the x axis first, if false, the method travels on the y axis first.
   * @param : correction, if true, navigate with correction, if false, navigate without correction
   */
  public static void squareNavigation(double x, double y, boolean xFirst, boolean correction) {
    GameResources.setLocalized(false);
    GameResources.setEnableCorrection(correction);
    GameResources.setNavigationCoordinates(new Point(x, y));

    if (xFirst) {
      Navigation.travelTo(x, (GameResources.odometer.getY() / GameResources.TILE_SIZE),
          GameResources.FORWARD_SPEED_NORMAL);
      Navigation.travelTo((GameResources.odometer.getX() / GameResources.TILE_SIZE), y,
          GameResources.FORWARD_SPEED_NORMAL);
    } else {
      Navigation.travelTo((GameResources.odometer.getX() / GameResources.TILE_SIZE), y,
          GameResources.FORWARD_SPEED_NORMAL);
      Navigation.travelTo(x, (GameResources.odometer.getY() / GameResources.TILE_SIZE),
          GameResources.FORWARD_SPEED_NORMAL);
    }
    GameResources.setObstacleDetected(false);
    GameResources.setEnableCorrection(false);
  }


  /**
   * Method implementing the navigation to the tunnel entrance and the turn towards the tunnel traversal orientation
   * 
   * @param : xFirst indicates if the navigation travels on the x or the y axis first
   */
  public void navigateToTunnelEntrance(boolean xFirst) {
    squareNavigation(tunnelEntrance.x, tunnelEntrance.y, xFirst,true);
    //Navigation.turnTo(tunnelEntranceTraversalOrientation, GameResources.ROTATE_SPEED_FAST);
  }

  /**
   * Method implementing the navigation to the tunnel entrance and the turn towards the tunnel traversal orientation
   * 
   * @param : xFirst indicates if the navigation travels on the x or the y axis first
   */
  public void navigateToTunnelExit(boolean xFirst) {
    squareNavigation(tunnelExit.x, tunnelExit.y, xFirst,true);
    Navigation.turnTo(tunnelExitTraversalOrientation, GameResources.ROTATE_SPEED_FAST);
  }

  /**
   * Method implementing the navigation to the launch and the turn towards the bin
   * 
   * @param : xFirst indicates if the navigation travels on the x or the y axis first
   */
  public void navigateToLaunchPoint(boolean xFirst) {
    boolean localized = false;
    if(this.calculateDistance(launchPoint.x, launchPoint.y, 
        GameResources.odometer.getX()/GameResources.TILE_SIZE, 
        GameResources.odometer.getY()/GameResources.TILE_SIZE)<GameResources.localizationDistance) {
      localized=true;
    }
    // navigate to launch point
    squareNavigation(launchPoint.x, launchPoint.y, xFirst,true);
    GameResources.setLocalized(localized);
  }

  /**
   * Method implementing the turn towards the launch point
   */
  public void turnToTarget() {
    double currentX = GameResources.odometer.getX();
    double currentY = GameResources.odometer.getY();
    double binX = GameResources.bin.x * GameResources.TILE_SIZE;
    double binY = GameResources.bin.y * GameResources.TILE_SIZE;
    double dX = binX - currentX;
    double dY = binY - currentY;
    // turn towards launch point
    Navigation.turnTo(Math.toDegrees(Math.atan2(dX, dY)), GameResources.ROTATE_SPEED_SLOW);
    System.out.println("atan: "+(Math.toDegrees(Math.atan2(dX, dY))));
    double distance = this.distanceFromBin(launchPoint.x, launchPoint.y);
    System.out.println("distance: "+distance);

    // additional turn so that the ballistic launcher points to the bin
    double adjustmentAngle = Math.toDegrees((Math.asin((GameResources.BALLISTIC_X_OFFSET_FROM_CENTER / distance))));
    Navigation.turn(GameResources.BALLISTIC_ADJUSTMENT_ANGLE- 7*adjustmentAngle, GameResources.ROTATE_SPEED_SLOW);

  }

  /**
   * Method implementing the navigation through a tunnel. At first, the robot travels to the exit of the tunnel, then it
   * adjust its heading according to the next black line and backs up. Also, after the tunnel traversal, the map
   * parameters are updated.
   */
  public void navigateThroughTunnel() {
    // first tunnel traversal
    if (tunnel == 0) {
      Navigation.travelTo(tunnelExit.x, tunnelExit.y, GameResources.FORWARD_SPEED_FAST);
      tunnel++;
    }
    // second tunnel traversal
    else if (tunnel == 1) {
      Navigation.travelTo(tunnelEntrance.x, tunnelEntrance.y, GameResources.FORWARD_SPEED_FAST);
    }
    LightLocalizer.twoLineDetection();
    Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_FAST);
    // update new zone parameters
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

    if (x <= Resources.red.ur.x && x >= Resources.red.ll.x && y <= Resources.red.ur.y && y >= Resources.red.ll.y) {
      return REGION.RED;
    }

    else if (x <= Resources.green.ur.x && x >= Resources.green.ll.x && y <= Resources.green.ur.y
        && y >= Resources.green.ll.y) {
      return REGION.GREEN;
    }

    else if (x <= Resources.island.ur.x && x >= Resources.island.ll.x && y <= Resources.island.ur.y
        && y >= Resources.island.ll.y) {
      return REGION.ISLAND;
    }

    else if (x <= Resources.tnr.ur.x && x >= Resources.tnr.ll.x && y <= Resources.tnr.ur.y && y >= Resources.tnr.ll.y) {
      return REGION.TUNNEL_RED;
    }

    else if (x <= Resources.tng.ur.x && x >= Resources.tng.ll.x && y <= Resources.tng.ur.y && y >= Resources.tng.ll.y) {
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
    double x = GameResources.odometer.getX();
    double y = GameResources.odometer.getY();
    int x1 = (int) (x / GameResources.TILE_SIZE);
    int y1 = (int) (y / GameResources.TILE_SIZE);
    int x2 = (int) x1 + 1;
    int y2 = (int) y1 + 1;
    int x3 = (int) x1 - 1;
    int y3 = (int) y1 - 1;
    // array of closest points
    LinkedList<Point> closestPoints = new LinkedList<Point>();
    Point p1 = new Point(x1, y1);
    Point p2 = new Point(x1, y2);
    Point p3 = new Point(x1, y3);

    Point p4 = new Point(x2, y1);
    Point p5 = new Point(x2, y2);
    Point p6 = new Point(x2, y3);

    Point p7 = new Point(x3, y1);
    Point p8 = new Point(x3, y2);
    Point p9 = new Point(x3, y3);



    // check if the 4 points generated are localizable
    // if they are, add them to the array
    if (GameNavigation.localizablePoint(p1)) {
      closestPoints.add(p1);
    }
    if (GameNavigation.localizablePoint(p2)) {
      closestPoints.add(p2);
    }
    if (GameNavigation.localizablePoint(p3)) {
      closestPoints.add(p3);
    }
    if (GameNavigation.localizablePoint(p4)) {
      closestPoints.add(p4);
    }
    if (GameNavigation.localizablePoint(p5)) {
      closestPoints.add(p5);
    }
    if (GameNavigation.localizablePoint(p6)) {
      closestPoints.add(p6);
    }
    if (GameNavigation.localizablePoint(p7)) {
      closestPoints.add(p7);
    }
    if (GameNavigation.localizablePoint(p8)) {
      closestPoints.add(p8);
    }
    if (GameNavigation.localizablePoint(p9)) {
      closestPoints.add(p9);
    }

    // compute distances of 4 points from actual position
    x = GameResources.odometer.getX() / GameResources.TILE_SIZE;
    y = GameResources.odometer.getY() / GameResources.TILE_SIZE;

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
    x1 = x1 * GameResources.TILE_SIZE;
    y1 = y1 * GameResources.TILE_SIZE;
    x2 = x2 * GameResources.TILE_SIZE;
    y2 = y2 * GameResources.TILE_SIZE;
    double distance = Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    return distance;
  }

  /**
   * Method used to find the tunnel entrance by evaluating the region of the points around the tunnel. Once the method
   * has found the entrance, it sets the tunnel entrance, exit and length data
   */
  public void updateTunnelData() {

    REGION targetRegion;
    REGION tunnelBottom = this.regionCalculation((GameResources.Tunnel.ll.x + 0.5), (GameResources.Tunnel.ll.y - 0.5));
    REGION tunnelTop = this.regionCalculation((GameResources.Tunnel.ur.x - 0.5), (GameResources.Tunnel.ur.y + 0.5));
    REGION tunnelLeft = this.regionCalculation((GameResources.Tunnel.ll.x - 0.5), (GameResources.Tunnel.ll.y + 0.5));
    REGION tunnelRight = this.regionCalculation((GameResources.Tunnel.ur.x + 0.5), (GameResources.Tunnel.ur.y - 0.5));

    // determine the target region, the region of the tunnel entrance
    if (GameResources.getCurrentRegion() == REGION.RED) {
      targetRegion = REGION.RED;
    } else  {
      targetRegion = REGION.GREEN;
    }
    // determine where is the tunnel entrance
    if (tunnelBottom == targetRegion) {
      this.setTunnelEntrance(new Point(GameResources.Tunnel.ll.x + 0.5, GameResources.Tunnel.ll.y - 1));
      this.setTunnelExit(new Point(GameResources.Tunnel.ur.x - 0.5, GameResources.Tunnel.ur.y + 1));
      this.setTunnelEntranceTraversalOrientation(0);
      this.setTunnelExitTraversalOrientation(180);
    } else if (tunnelTop == targetRegion) {
      this.setTunnelEntrance(new Point(GameResources.Tunnel.ur.x - 0.5, GameResources.Tunnel.ur.y + 1));
      this.setTunnelExit(new Point(GameResources.Tunnel.ll.x + 0.5, GameResources.Tunnel.ll.y - 1));
      this.setTunnelEntranceTraversalOrientation(180);
      this.setTunnelExitTraversalOrientation(0);

    } else if (tunnelLeft == targetRegion) {
      this.setTunnelEntrance(new Point(GameResources.Tunnel.ll.x - 1, GameResources.Tunnel.ll.y + 0.5));
      this.setTunnelExit(new Point(GameResources.Tunnel.ur.x + 1, GameResources.Tunnel.ur.y - 0.5));
      this.setTunnelEntranceTraversalOrientation(90);
      this.setTunnelExitTraversalOrientation(270);

    } else if (tunnelRight == targetRegion) {
      System.out.println("ENTRANCE IS RIGHT");
      this.setTunnelEntrance(new Point(GameResources.Tunnel.ur.x + 1, GameResources.Tunnel.ur.y - 0.5));
      this.setTunnelExit(new Point(GameResources.Tunnel.ll.x - 1, GameResources.Tunnel.ll.y + 0.5));
      this.setTunnelEntranceTraversalOrientation(270);
      this.setTunnelExitTraversalOrientation(90);

      System.out.println("TUNNEL ENTRANCE: " + tunnelEntrance.x + ", " + tunnelEntrance.y);

    }
  }

  /**
   * Method used to set the tunnel depending on the team color
   */
  public void setTunnel() {
    if (GameResources.getColor() == COLOR.RED) {
      System.out.println("RED TUNNEL SET");

      GameResources.Tunnel.ll.x = Resources.tnr.ll.x;
      GameResources.Tunnel.ll.y = Resources.tnr.ll.y;
      GameResources.Tunnel.ur.x = Resources.tnr.ur.x;
      GameResources.Tunnel.ur.y = Resources.tnr.ur.y;
    } else {
      System.out.println("GREEN TUNNEL SET");

      GameResources.Tunnel.ll.x = Resources.tng.ll.x;
      GameResources.Tunnel.ll.y = Resources.tng.ll.y;
      GameResources.Tunnel.ur.x = Resources.tng.ur.x;
      GameResources.Tunnel.ur.y = Resources.tng.ur.y;
    }
  }

  /**
   * Method used to set the limits of the current region
   */
  public void setLimits() {
    // set the limits depending on the current region
    switch (GameResources.getCurrentRegion()) {
      case GREEN:
        System.out.println("GREEN LIMITS SET");

        GameResources.currentLeftLimit = Resources.green.ll.x;
        GameResources.currentRightLimit = Resources.green.ur.x;
        GameResources.currentTopLimit = Resources.green.ur.y;
        GameResources.currentBottomLimit = Resources.green.ll.y;
        break;
      case RED:
        System.out.println("RED LIMITS SET");
        GameResources.currentLeftLimit = Resources.red.ll.x;
        GameResources.currentRightLimit = Resources.red.ur.x;
        GameResources.currentTopLimit = Resources.red.ur.y;
        GameResources.currentBottomLimit = Resources.red.ll.y;
        break;
      case ISLAND:
        System.out.println("ISLAND LIMITS SET");

        GameResources.currentLeftLimit = Resources.island.ll.x;
        GameResources.currentRightLimit = Resources.island.ur.x;
        GameResources.currentTopLimit = Resources.island.ur.y;
        GameResources.currentBottomLimit = Resources.island.ll.y;
        break;
      default:
        break;

    }
  }

  /**
   * Method used to set the color
   */
  public void setColor() {
    if (Resources.redTeam == Resources.TEAM_NUMBER) {
      System.out.println("RED COLOR SET");

      GameResources.setColor(COLOR.RED);
    } else if (Resources.greenTeam == Resources.TEAM_NUMBER) {
      GameResources.setColor(COLOR.GREEN);
      System.out.println("GREEN COLOR SET");
    }

  }

  /**
   * Method used to set the staring region depending on the team color
   */
  public void setStartingRegion() {
    if (GameResources.getColor() == COLOR.GREEN) {
      GameResources.setCurrentRegion(REGION.GREEN);
      System.out.println("STARTING REGION SET TO GREEN");

    } else {
      GameResources.setCurrentRegion(REGION.RED);
      System.out.println("STARTING REGION SET TO RED");

    }
  }

  /**
   * Method used to set the bin depending on the team color
   */
  public void setBin() {
    if (GameResources.getColor() == COLOR.GREEN) {
      GameResources.setBin(Resources.greenBin);
      System.out.println("STARTING BIN SET TO GREEN");

    } else {
      GameResources.setCurrentRegion(REGION.RED);
      GameResources.setBin(Resources.redBin);
      System.out.println("STARTING BIN SET TO RED");


    }
  }

  /**
   * Method used to update the region after a tunnel traversal depending on the current region
   */
  public void updateRegion() {
    if (GameResources.getCurrentRegion() == REGION.GREEN || GameResources.getCurrentRegion() == REGION.RED) {
      GameResources.setCurrentRegion(REGION.ISLAND);
      System.out.println("REGION SET TO ISLAND" + GameResources.currentRegion);
    } else if (GameResources.getCurrentRegion() == REGION.ISLAND) {
      if (GameResources.getColor() == COLOR.GREEN) {
        GameResources.setCurrentRegion(REGION.GREEN);
      } else {
        GameResources.setCurrentRegion(REGION.RED);
      }
    }

  }

  /**
   * Method setting the coordinates of the starting corner depending on the team color and on the corner of the team
   * color
   */
  public void setCorner() {
    if (GameResources.getColor() == COLOR.GREEN) {
      switch (Resources.greenCorner) {
        // lower left corner
        case 0:
          GameResources.CORNER_NUMBER = 0;
          GameResources.STARTING_POINT = new Point(1, 1);
          System.out.println("GREEN CORNER SET TO 0");

          break;
        // lower right corner
        case 1:
          System.out.println("GREEN CORNER SET TO 1");

          GameResources.CORNER_NUMBER = 1;
          GameResources.STARTING_POINT = new Point(GameResources.FIELD_RIGHT - 1, 1);
          System.out
              .println("STARTING POINT: " + GameResources.STARTING_POINT.x + ", " + GameResources.STARTING_POINT.y);

          break;
        // top right corner
        case 2:
          System.out.println("GREEN CORNER SET TO 2");

          GameResources.CORNER_NUMBER = 2;
          GameResources.STARTING_POINT = new Point(GameResources.FIELD_RIGHT - 1, GameResources.FIELD_TOP - 1);
          System.out
              .println("STARTING POINT: " + GameResources.STARTING_POINT.x + ", " + GameResources.STARTING_POINT.y);

          break;
        // top left corner
        case 3:
          System.out.println("GREEN CORNER SET TO 3");

          GameResources.CORNER_NUMBER = 3;
          GameResources.STARTING_POINT = new Point(1, GameResources.FIELD_TOP - 1);
          System.out
              .println("STARTING POINT: " + GameResources.STARTING_POINT.x + ", " + GameResources.STARTING_POINT.y);

          break;
      }
    } else {
      switch (Resources.redCorner) {
        // lower left corner
        case 0:
          GameResources.CORNER_NUMBER = 0;
          GameResources.STARTING_POINT = new Point(1, 1);
          System.out.println("RED CORNER SET TO 0");

          break;
        // lower right corner
        case 1:
          GameResources.CORNER_NUMBER = 1;
          GameResources.STARTING_POINT = new Point(GameResources.FIELD_RIGHT - 1, 1);
          System.out.println("RED CORNER SET TO 1");

          break;
        // top right corner
        case 2:
          GameResources.CORNER_NUMBER = 2;
          GameResources.STARTING_POINT = new Point(GameResources.FIELD_RIGHT - 1, GameResources.FIELD_TOP - 1);
          System.out.println("RED CORNER SET TO 2");

          break;
        // top left corner
        case 3:
          GameResources.CORNER_NUMBER = 3;
          GameResources.STARTING_POINT = new Point(1, GameResources.FIELD_TOP - 1);
          System.out.println("RED CORNER SET TO 3");

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
    double distance = this.calculateDistance(x, y, GameResources.bin.x, GameResources.bin.y);
    return distance;
  }

  /**
   * Method used to set the initial map parameters
   */
  public void setGameParameters() {
    this.setColor();
    this.setStartingRegion();
    this.setCorner();
    this.setBin();
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
    // set the new zone limits
    this.setLimits();
  }


  /**
   * Method used to calculate the closest possible launch point from the robot and to set the current launch point of
   * the game navigation instance
   * 
   * @return: The closest possible launch point from the robot positon
   */
  public Point calculateClosestLaunchPoint() {
    double x = GameResources.odometer.getX() / GameResources.TILE_SIZE;
    double y = GameResources.odometer.getY() / GameResources.TILE_SIZE;
    Point minimal_point = launchPoints.get(0);
    double minimal_distance = this.calculateDistance(x, y, minimal_point.x, minimal_point.y);
    for (Point point : launchPoints) {
      double distance = this.calculateDistance(x, y, point.x, point.y);
      if (distance <= minimal_distance) {
        minimal_distance = distance;
        minimal_point = point;
      }
    }
    // set the launch point of the class instance and return it
    this.launchPoint = minimal_point;
    // set new navigation coordinates
    GameResources.setNavigationCoordinates(minimal_point);
    return minimal_point;
  }

 
  /**
   * Method used to populate the linked list of possible launch points on the island of the game navigation instance. A
   * possible launch point is a point that is not on the borders of the current region, that is within the maximal
   * distance from the bin and that is not in the restricted linked list of points.
   */
  public void generateLaunchPoints() {
    // set limits of launching zone
    int left = (int) Resources.island.ll.x + 1;
    int right = (int) Resources.island.ur.x - 1;
    int bottom = (int) Resources.island.ll.y + 1;
    int top = (int) Resources.island.ur.y - 1;
    // initialize or clear the list
    launchPoints = new LinkedList<Point>();
    // for all inside x values of the island
    for (int i = left; i <= right; i++) {
      // for all inside y values of the island
      for (int j = bottom; j <= top; j++) {
        Point point = new Point(i, j);
        double distance = this.distanceFromBin(i, j);
        // check if the distance from the bin is within the maximal distance
        if (distance <= GameResources.MAXIMAL_LAUNCH_DISTANCE && distance >= GameResources.MINIMAL_LAUNCH_DISTANCE) {
          boolean restricted = false;
          // check if the point is restricted
          for (Point restrictedPoint : GameResources.restrictedPoints) {
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
    if ((point.x != GameResources.currentLeftLimit) && (point.x != GameResources.currentRightLimit)
        && (point.y != GameResources.currentBottomLimit) && (point.y != GameResources.currentTopLimit)) {
      for (Point restrictedPoint : GameResources.restrictedPoints) {
        // verification that the input point is not in the restricted point array
        if ((point.x == restrictedPoint.x) && (point.y == restrictedPoint.y)) {
          return false;
        }
      }
      // point is not in restricted array and not on a border--> localizable point
      return true;
    }
    return false;
  }

  /**
   * Method creating restricted points. It is called after an obstacle detection. Depending on the orientation of the
   * robot, (0 degrees, 90 degrees, 180 degrees or 270 degrees), the method estimates the coordinate position of the
   * center of the obstacle. The method will then add the 9 points surrounding the obstacle in the restricted points
   * array. If one of those restricted points was the current launching point, the method returns true and indicates
   * that the launching point must be changed
   * 
   * @return: true if the current launching point was restricted, false if the current launching point is not restricted
   */
  public boolean createRestrictedPoints() {
    double x = GameResources.odometer.getX();
    double y = GameResources.odometer.getY();
    double theta = GameResources.odometer.getTheta();

    // heading approximatly towards 0 degrees
    if ((theta >= 355 && theta <= 360) || (theta >= 0 && theta <= 5)) {
      y = y + GameResources.OBSTACLE_DETECTION_DISTANCE + GameResources.OBSTACLE_WIDTH;
    }
    // heading approximatly towards 90 degrees
    else if (theta >= 85 && theta <= 95) {
      x = x + GameResources.OBSTACLE_DETECTION_DISTANCE + GameResources.OBSTACLE_WIDTH;
    }
    // heading approximately towards 180 degrees
    else if (theta >= 175 && theta <= 185) {
      y = y - GameResources.OBSTACLE_DETECTION_DISTANCE - GameResources.OBSTACLE_WIDTH;
    }
    // heading approximately towards 270 degrees
    else if (theta >= 265 && theta <= 275) {
      y = x - GameResources.OBSTACLE_DETECTION_DISTANCE - GameResources.OBSTACLE_WIDTH;
    }
    // x and y represent approximatly the coordinates of the center of the obstacle
    int x1 = (int) (x / GameResources.TILE_SIZE);
    int y1 = (int) (y / GameResources.TILE_SIZE);
    int x2 = x1 + 1;
    int y2 = y1 + 1;
    int x3 = x1 - 1;
    int y3 = y1 - 1;
    // create 9 points around the approximate center of the obstacle
    LinkedList<Point> obstaclePoints = new LinkedList<Point>();
    Point p1 = new Point(x1, y1);
    obstaclePoints.add(p1);
    Point p2 = new Point(x1, y2);
    obstaclePoints.add(p2);
    Point p3 = new Point(x1, y3);
    obstaclePoints.add(p3);
    Point p4 = new Point(x2, y1);
    obstaclePoints.add(p4);
    Point p5 = new Point(x2, y2);
    obstaclePoints.add(p5);
    Point p6 = new Point(x2, y3);
    obstaclePoints.add(p6);
    Point p7 = new Point(x3, y1);
    obstaclePoints.add(p7);
    Point p8 = new Point(x3, y2);
    obstaclePoints.add(p8);
    Point p9 = new Point(x3, y3);
    obstaclePoints.add(p9);
    boolean newLaunchPoint = false;

    // add the 9 points surrounding the obstacle in the restricted points array
    for (Point point : obstaclePoints) {
      // add each point in the restricted points list
      GameResources.restrictedPoints.add(point);
      // if one of the new restricted point was the current launch point, we need a new launch point
      if (this.getLaunchPoint().x == point.x && this.getLaunchPoint().y == point.y) {
        newLaunchPoint = true;
      }
    }
    return newLaunchPoint;
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
   * Getter Method for the tunnel entrance traversal orientation
   * 
   * @return: the orientation needed to traverse the tunnel from the entrance
   */
  public double getTunnelEntranceTraversalOrientation() {
    return tunnelEntranceTraversalOrientation;
  }

  /**
   * Getter Method for the tunnel exit traversal orientation
   * 
   * @return: the orientation needed to traverse the tunnel from the exit
   */
  public double getTunnelExitTraversalOrientation() {
    return tunnelExitTraversalOrientation;
  }

  /**
   * Getter Method for the launch point
   * 
   * @return: the launching coordinate point
   */
  public Point getLaunchPoint() {
    return launchPoint;
  }

  /**
   * Setter Method of the tunnel entrance
   * 
   * @param: the coordinate point of the tunnel entrance
   */
  public void setTunnelEntrance(Point tunnelEntrance) {
    this.tunnelEntrance = tunnelEntrance;
  }

  /**
   * Setter Method of the tunnel exit
   * 
   * @param: the coordinate point of the tunnel exit
   */
  public void setTunnelExit(Point tunnelExit) {
    this.tunnelExit = tunnelExit;
  }

  /**
   * Setter Method for the tunnel entrance traversal orientation
   * 
   * @param: the orientation needed to traverse the tunnel from the exit
   */
  public void setTunnelEntranceTraversalOrientation(double tunnelEntranceTraversalOrientation) {
    this.tunnelEntranceTraversalOrientation = tunnelEntranceTraversalOrientation;
  }

  /**
   * Setter Method for the tunnel exit traversal orientation
   * 
   * @param: the orientation needed to traverse the tunnel from the exit
   */
  public void setTunnelExitTraversalOrientation(double tunnelExitTraversalOrientation) {
    this.tunnelExitTraversalOrientation = tunnelExitTraversalOrientation;
  }
  
  public void setLaunchPoint(Point launchPoint) {
    this.launchPoint = launchPoint;
  }

}
