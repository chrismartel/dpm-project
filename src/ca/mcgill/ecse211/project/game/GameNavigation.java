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
  private void navigateToTunnel() {
    Navigation.travelTo(tunnelEntrance[0], tunnelEntrance[1], FORWARD_SPEED_NORMAL);
    Navigation.turnTo(tunnelTraversalOrientation, ROTATE_SPEED_SLOW);
  }

  private void navigateThroughTunnel() {
    Navigation.travelTo(tunnelExit[0], tunnelExit[1], FORWARD_SPEED_SLOW);
  }

  /**
   * Method returning the region of a specific tile
   * 
   * @param x : the x coordinate of the center of the tile
   * @param y : the y coordinate of the center of the tile
   * @return the region type of the tile
   */
  private REGION regionCalculation(double x, double y) {

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
  private int[] closestPoint() {
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
  private double calculateDistance(double x1, double y1, double x2, double y2) {
    double distance = Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    return distance;
  }

  /**
   * Method finding and setting the coordinates of the tunnel entrance
   */
  private void updateTunnelData() {

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
      //this.tunnelLength = (TUNNEL_UR[1] - TUNNEL_LL[1]) * TILE_SIZE;
    } else if (tunnelTop == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] + 0.5);
      this.tunnelExit[0] = (TUNNEL_LL[0] + 0.5);
      this.tunnelExit[1] = (TUNNEL_LL[1] - 0.5);
      this.tunnelTraversalOrientation = 180;
      //this.tunnelLength = (TUNNEL_UR[1] - TUNNEL_LL[1]) * TILE_SIZE;
    } else if (tunnelLeft == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_LL[1] + 0.5);
      this.tunnelExit[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelExit[1] = (TUNNEL_UR[1] - 0.5);
      this.tunnelTraversalOrientation = 90;
      //this.tunnelLength = (TUNNEL_UR[0] - TUNNEL_LL[0]) * TILE_SIZE;
    } else if (tunnelRight == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] - 0.5);
      this.tunnelExit[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelExit[1] = (TUNNEL_LL[1] + 0.5);
      this.tunnelTraversalOrientation = 270;
      //this.tunnelLength = (TUNNEL_UR[0] - TUNNEL_LL[0]) * TILE_SIZE;
    }
  }

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
