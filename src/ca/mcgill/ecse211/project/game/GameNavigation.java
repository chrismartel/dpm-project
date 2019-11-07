package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import static ca.mcgill.ecse211.project.Resources.*;

import lejos.hardware.Button;



public class GameNavigation {



  /**
   * coordinates of the tunnel entrance and exit
   */
  private Point tunnelEntrance;
  private Point tunnelExit;
  /**
   * length of the tunnel
   */
  private double tunnelLength;

  /**
   * coordinates of the launch point
   */
  private Point launchPoint;

  /**
   * Orientation the robot needs to have to traverse the tunnel
   */
  private double tunnelTraversalOrientation;

  public GameNavigation() {
    tunnelEntrance = new Point(0,0);
    tunnelExit = new Point(0,0);
    launchPoint = new Point(0,0);
  }
  
  public void squareNavigation(double x, double y) {
    enableCorrection = true;
    Navigation.travelTo(x, (odometer.getY()/TILE_SIZE), FORWARD_SPEED_NORMAL);
    Navigation.travelTo((odometer.getX()/TILE_SIZE), y, FORWARD_SPEED_NORMAL);
    enableCorrection = false;
  }

  /**
   * Method used to navigate to the tunnel entrance
   */
  public void navigateToTunnel() {
    this.squareNavigation(tunnelEntrance.x, tunnelEntrance.y);
    Navigation.turnTo(tunnelTraversalOrientation, ROTATE_SPEED_SLOW);
  }

  /**
   * Method used to navigate through the tunnel entrance
   */
  public void navigateThroughTunnel() {
    Navigation.travelTo(tunnelExit.x, tunnelExit.y, FORWARD_SPEED_NORMAL);
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
   * Method used to find the closest coordinate point from the actual position of the odometer
   * 
   * @return an array representing the closest coordinate point from the current odometer position
   */
  public Point closestPoint() {
    double x = odometer.getX();
    double y = odometer.getY();
    int x1 = (int) (x / TILE_SIZE);
    int y1 = (int) (y / TILE_SIZE);
    int x2 = (int) x1 + 1;
    int y2 = (int) y1 + 1;
    Point p1 = new Point(x1,y1);
    Point p2 = new Point(x1,y2);
    Point p3 = new Point(x2,y1);
    Point p4 = new Point(x2,y2);
    // compute distances of 4 points from actual position
    x = odometer.getX()/TILE_SIZE;
    y = odometer.getY()/TILE_SIZE;
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
    REGION tunnelBottom = this.regionCalculation((Tunnel.ll.x + 0.5), (Tunnel.ll.y - 0.5));
    REGION tunnelTop = this.regionCalculation((Tunnel.ur.x - 0.5), (Tunnel.ur.y+ 0.5));
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
    if(redTeam == TEAM_NUMBER) {
      color = COLOR.RED;
    }
    else if(greenTeam == TEAM_NUMBER){
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
      System.out.println("STARTING REGION SET");

    } else {
      currentRegion = REGION.RED;
    }
  }

  /**
   * Method setting the coordinates of the starting corner
   */
  public void setCorner() {
    // TODO: determine which is the corner 0
    if (color == COLOR.GREEN) {
      switch (greenCorner) {
        case 0:
          STARTING_CORNER.x = 0;
          STARTING_CORNER.y = 0;
          CORNER_NUMBER = 0;
          break;
        case 1:
          STARTING_CORNER.x = 15;
          STARTING_CORNER.y = 0;
          CORNER_NUMBER = 1;
          System.out.println("CORNER 1 SET");

          break;
        case 2:
          STARTING_CORNER.x = 15;
          STARTING_CORNER.y = 9;
          CORNER_NUMBER = 2;
          break;
        case 3:
          STARTING_CORNER.x = 0;
          STARTING_CORNER.y = 9;
          CORNER_NUMBER = 3;
          break;
      }
    } else {
      switch (redCorner) {
        case 0:
          STARTING_CORNER.x = 1;
          STARTING_CORNER.y = 1;
          CORNER_NUMBER = 0;
          break;
        case 1:
          STARTING_CORNER.x = 1;
          STARTING_CORNER.y = 14;
          CORNER_NUMBER = 1;
          break;
        case 2:
          STARTING_CORNER.x = 8;
          STARTING_CORNER.y = 14;
          CORNER_NUMBER = 2;
          break;
        case 3:
          STARTING_CORNER.x = 8;
          STARTING_CORNER.y = 1;
          CORNER_NUMBER = 3;
          break;
      }

    }
  }

  public void calculateLaunchPoints() {
    // TODO: method to find 3 possible launch points to consider that there might be obstacles
  }


  public Point getTunnelEntrance() {
    return tunnelEntrance;
  }

  public Point getTunnelExit() {
    return tunnelExit;
  }

  public double getTunnelTraversalOrientation() {
    return tunnelTraversalOrientation;
  }

  public double getTunnelLength() {
    return tunnelLength;
  }

}
