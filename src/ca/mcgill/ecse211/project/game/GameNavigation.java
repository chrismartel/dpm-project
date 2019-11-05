package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;


public class GameNavigation {

  public enum REGION {
    RED, WATER, TUNNEL_RED, TUNNEL_GREEN, GREEN, ISLAND
  }

  /**
   * coordinates of the tunnel entrance and exit
   */
  private double[] tunnelEntrance;
  private double[] tunnelExit;
  /**
   * length of the tunnel
   */
  private double tunnelLength;

  /**
   * coordinates of the launch point
   */
  private double[] launchPoint;
  /**
   * coordinates of the target point
   */
  private double[] targetPoint;

  /**
   * Orientation the robot needs to have to traverse the tunnel
   */
  private double tunnelTraversalOrientation;

  public GameNavigation() {
    tunnelEntrance = new double[2];
    tunnelExit = new double[2];
    launchPoint = new double[2];
    targetPoint = new double[2];
    targetPoint[0] = BIN[0];
    targetPoint[1] = BIN[1];
  }

  /**
   * Method used to navigate to the tunnel entrance
   */
  public void navigateToTunnel() {
    System.out.println("navigate to tunnel");
    System.out.println("entrance: "+tunnelEntrance[0] + tunnelEntrance[1]);

    Navigation.travelTo(tunnelEntrance[0], tunnelEntrance[1], FORWARD_SPEED_NORMAL);
    Navigation.turnTo(tunnelTraversalOrientation, ROTATE_SPEED_SLOW);
  }

  public void navigateThroughTunnel() {
    // TODO: method using ultrasonic sensors to travel through tunnel
    // if traversal completed --> set tunnelCompleted boolean to true
  }

  /**
   * Method returning the region of a specific tile
   * 
   * @param x : the x coordinate of the center of the tile
   * @param y : the y coordinate of the center of the tile
   * @return the region type of the tile
   */
  public REGION regionCalculation(double x, double y) {

    if (x <= RED_UR[0] && x >= RED_LL[0] && y <= RED_UR[1] && y >= RED_LL[1]) {
      return REGION.RED;
    }

    else if (x <= GREEN_UR[0] && x >= GREEN_LL[0] && y <= GREEN_UR[1] && y >= GREEN_LL[1]) {
      return REGION.GREEN;
    }

    else if (x <= ISLAND_UR[0] && x >= ISLAND_LL[0] && y <= ISLAND_UR[1] && y >= ISLAND_LL[1]) {
      return REGION.ISLAND;
    }

    else if (x <= TNR_UR[0] && x >= TNR_LL[0] && y <= TNR_UR[1] && y >= TNR_LL[1]) {
      return REGION.TUNNEL_RED;
    }

    else if (x <= TNG_UR[0] && x >= TNG_LL[0] && y <= TNG_UR[1] && y >= TNG_LL[1]) {
      return REGION.TUNNEL_GREEN;
    }

    return REGION.WATER;
  }

  /**
   * Method used to find the closest coordinate point from the actual position of the odometer
   * 
   * @return an array representing the closest coordinate point from the current odometer position
   */
  public int[] closestPoint() {
    double x = odometer.getX();
    double y = odometer.getY();
    int x1 = (int) (x % TILE_SIZE);
    int y1 = (int) (y % TILE_SIZE);
    int x2 = (int) x1 + 1;
    int y2 = (int) y1 + 1;
    int[] p1 = {x1, y1};
    int[] p2 = {x1, y2};
    int[] p3 = {x2, y1};
    int[] p4 = {x2, y2};
    // compute distances of 4 points from actual position
    double d1 = this.calculateDistance(x, y, x1, y1);
    double d2 = this.calculateDistance(x, y, x1, y2);
    double d3 = this.calculateDistance(x, y, x2, y1);
    double d4 = this.calculateDistance(x, y, x2, y2);
    // find the minimal distance
    double d = Math.min(d1, d2);
    d = Math.min(d, d3);
    d = Math.min(d, d4);
    // return the closest point
    if (d == d1) {
      return p1;
    } else if (d == d2) {
      return p2;

    } else if (d == d3) {
      return p3;

    } else {
      return p4;
    }
  }

  /**
   * Method used to calculate the distance between 2 points
   * 
   * @param x1: x coordinate of the first point
   * @param y1: y coordinate of the first point
   * @param x2: x coordinate of the second point
   * @param y2: y coordinate of the second point
   * @return the distance between 2 points
   */
  public double calculateDistance(double x1, double y1, double x2, double y2) {
    double distance = Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    return distance;
  }

  /**
   * Method used to find the tunnel entrance by evaluating the region of the points around the tunnel. Once the method
   * has found the entrance, it sets the tunnel entrance, exit and length data
   */
  public void updateTunnelData() {

    REGION targetRegion;
    REGION tunnelBottom = this.regionCalculation((TUNNEL_LL[0] + 0.5), (TUNNEL_LL[1] - 0.5));
    REGION tunnelTop = this.regionCalculation((TUNNEL_UR[0] - 0.5), (TUNNEL_UR[1] + 0.5));
    REGION tunnelLeft = this.regionCalculation((TUNNEL_LL[0] - 0.5), (TUNNEL_LL[1] + 0.5));
    REGION tunnelRight = this.regionCalculation((TUNNEL_UR[0] + 0.5), (TUNNEL_UR[1] - 0.5));

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
      this.tunnelEntrance[0] = (TUNNEL_LL[0] + 0.5);
      this.tunnelEntrance[1] = (TUNNEL_LL[1] - 0.5);
      this.tunnelExit[0] = (TUNNEL_UR[0] - 0.5);
      this.tunnelExit[1] = (TUNNEL_UR[1] + 0.5);
      this.tunnelTraversalOrientation = 0;
      // this.tunnelLength = (TUNNEL_UR[1] - TUNNEL_LL[1]) * TILE_SIZE;
    } else if (tunnelTop == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] + 0.5);
      this.tunnelExit[0] = (TUNNEL_LL[0] + 0.5);
      this.tunnelExit[1] = (TUNNEL_LL[1] - 0.5);
      this.tunnelTraversalOrientation = 180;
      // this.tunnelLength = (TUNNEL_UR[1] - TUNNEL_LL[1]) * TILE_SIZE;
    } else if (tunnelLeft == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_LL[1] + 0.5);
      this.tunnelExit[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelExit[1] = (TUNNEL_UR[1] - 0.5);
      this.tunnelTraversalOrientation = 90;
      // this.tunnelLength = (TUNNEL_UR[0] - TUNNEL_LL[0]) * TILE_SIZE;
    } else if (tunnelRight == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] - 0.5);
      this.tunnelExit[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelExit[1] = (TUNNEL_LL[1] + 0.5);
      this.tunnelTraversalOrientation = 270;
      // this.tunnelLength = (TUNNEL_UR[0] - TUNNEL_LL[0]) * TILE_SIZE;
    }
  }

  /**
   * Method used to set the tunnel depending on the team color
   */
  public void setTunnel() {
    if (color == COLOR.RED) {
      TUNNEL_LL[0] = TNR_LL[0];
      TUNNEL_LL[1] = TNR_LL[1];
      TUNNEL_UR[0] = TNR_UR[0];
      TUNNEL_UR[1] = TNR_UR[1];
    } else {
      TUNNEL_LL[0] = TNG_LL[0];
      TUNNEL_LL[1] = TNG_LL[1];
      TUNNEL_UR[0] = TNG_UR[0];
      TUNNEL_UR[1] = TNG_UR[1];
    }
  }
  /**
   * Method used to set the limits of the current region
   */
  public void setLimits() {
    // set the limits depending on the current region
    switch(currentRegion) {
      case GREEN:
        currentLeftLimit = GREEN_LL[0];
        currentRightLimit = GREEN_UR[0];
        currentTopLimit = GREEN_UR[1];
        currentBottomLimit = GREEN_LL[1];
        break;
      case RED:
        currentLeftLimit = RED_LL[0];
        currentRightLimit = RED_UR[0];
        currentTopLimit = RED_UR[1];
        currentBottomLimit = RED_LL[1];
        break;
      case ISLAND:
        currentLeftLimit = ISLAND_LL[0];
        currentRightLimit = ISLAND_UR[0];
        currentTopLimit = ISLAND_UR[1];
        currentBottomLimit = ISLAND_LL[1];
        break;
      default:
        break;
      
    }
  }
  
  /**
   * Method used to set the staring region depending on the team color
   */
  public void setStartingRegion() {
    if(color == COLOR.GREEN) {
      currentRegion = REGION.GREEN;
    }
    else {
      currentRegion = REGION.RED;
    }
  }
  /**
   * Method setting the coordinates of the starting corner
   */
  public void setCorner() {
    // TODO: determine which is the corner 0
    if(color == COLOR.GREEN) {
      switch(GREEN_CORNER) {
        case 0:
          STARTING_CORNER[0] = 1;
          STARTING_CORNER[1] = 1;
          break;
        case 1:
          STARTING_CORNER[0] = 1;
          STARTING_CORNER[1] = 14;
          break;
        case 2:
          STARTING_CORNER[0] = 8;
          STARTING_CORNER[1] = 14;
          break;
        case 3:
          STARTING_CORNER[0] = 8;
          STARTING_CORNER[1] = 1;
          break;
      }
    }
    else {
      switch(RED_CORNER) {
        case 0:
          STARTING_CORNER[0] = 1;
          STARTING_CORNER[1] = 1;
          break;
        case 1:
          STARTING_CORNER[0] = 1;
          STARTING_CORNER[1] = 14;
          break;
        case 2:
          STARTING_CORNER[0] = 8;
          STARTING_CORNER[1] = 14;
          break;
        case 3:
          STARTING_CORNER[0] = 8;
          STARTING_CORNER[1] = 1;
          break;
      }
      
    }
  }
  public void calculateLaunchPoints() {
    // TODO: method to find 3 possible launch points to consider that there might be obstacles
  }


  public double[] getTunnelEntrance() {
    return tunnelEntrance;
  }

  public double[] getTunnelExit() {
    return tunnelExit;
  }

  public double getTunnelTraversalOrientation() {
    return tunnelTraversalOrientation;
  }

  public double getTunnelLength() {
    return tunnelLength;
  }

}
