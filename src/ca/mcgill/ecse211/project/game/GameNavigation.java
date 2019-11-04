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
   * Method finding and setting the coordinates of the tunnel entrance
   */
  private void updateTunnelData() {

    REGION targetRegion;
    REGION tunnelBottom = this.regionCalculation((TUNNEL_LL[0] + 0.5), (TUNNEL_LL[1] - 0.5));
    REGION tunnelTop = this.regionCalculation((TUNNEL_UR[0] - 0.5), (TUNNEL_UR[1] + 0.5));
    REGION tunnelLeft = this.regionCalculation((TUNNEL_LL[0] - 0.5), (TUNNEL_LL[1] + 0.5));
    REGION tunnelRight = this.regionCalculation((TUNNEL_UR[0] + 0.5), (TUNNEL_UR[1] - 0.5));

    // determine the target region, the region of the tunel entrance
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
    } else if (tunnelTop == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] + 0.5);
      this.tunnelExit[0] = (TUNNEL_LL[0] + 0.5);
      this.tunnelExit[1] = (TUNNEL_LL[1] - 0.5);
      this.tunnelTraversalOrientation = 180;
    } else if (tunnelLeft == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_LL[1] + 0.5);
      this.tunnelExit[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelExit[1] = (TUNNEL_UR[1] - 0.5);
      this.tunnelTraversalOrientation = 90;
    } else if (tunnelRight == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] - 0.5);
      this.tunnelExit[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelExit[1] = (TUNNEL_LL[1] + 0.5);
      this.tunnelTraversalOrientation = 270;

    }
  }
  public void chooseTunnel() {
    if(color == COLOR.RED) {
      TUNNEL_LL[0] = TNR_LL[0];
      TUNNEL_LL[1] = TNR_LL[1];
      TUNNEL_UR[0] = TNR_UR[0];
      TUNNEL_UR[1] = TNR_UR[1];
    }
    else {
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

}
