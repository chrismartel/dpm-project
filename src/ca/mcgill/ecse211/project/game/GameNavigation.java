package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.game.GameResources.*;

import java.util.LinkedList;
import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.game.GameResources.REGION;
import lejos.hardware.Button;
import lejos.hardware.Sound;



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
   * length of the tunnel
   */
  private double tunnelLength;

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
   * Method used to light localize when the robot is already approximately on the localize point
   */
  public void lightLocalize2(Point point) {
    Navigation.turnTo(0, ROTATE_SPEED_NORMAL);
    twoLineDetection();
    Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_NORMAL);
    odometer.setXYT(odometer.getX(),point.y*TILE_SIZE , 0);
    Navigation.turnTo(90, ROTATE_SPEED_NORMAL);
    twoLineDetection();
    odometer.setXYT(point.x*TILE_SIZE+OFFSET_FROM_WHEELBASE,odometer.getY(),90);
  }
  
  /**
   * Method used to light localize to the closest point from the robot
   */
  public void lightLocalize(Point closestPoint) {

    // THE GAME STATE MUST BE IN LIGHTLOCALIZATION.
    int speed = 150;
    double x = closestPoint.x;
    double y = closestPoint.y;
    double midX;
    double midY;
    boolean turnRight = false;
    double firstTheta;
    double secondTheta;
    double newY;
    double newX;
    // LIGHT LOCALIZE FOR THE Y VALUE FIRST.
    if (x > (odometer.getX() / TILE_SIZE)) {
      midX = x - 0.5;
      turnRight = true;
      secondTheta = 90;
    } else {
      midX = x + 0.5;
      secondTheta = 270;
    }
    if (y > (odometer.getY() / TILE_SIZE)) {
      midY = y - 0.5;
      firstTheta = 0;
      newY = y * TILE_SIZE + OFFSET_FROM_WHEELBASE;
      if(turnRight) {
        newX = x * TILE_SIZE + OFFSET_FROM_WHEELBASE;
      }
      else {
        newX = x * TILE_SIZE - OFFSET_FROM_WHEELBASE;
      }

    } else {
      midY = y + 0.5;
      firstTheta = 180;
      secondTheta = Math.abs(secondTheta - 360);
      newY = x * TILE_SIZE - OFFSET_FROM_WHEELBASE;
      if(turnRight) {
        newX = x * TILE_SIZE - OFFSET_FROM_WHEELBASE;
//        Navigation.turn(-180, speed);
      }
      else {
        newX = x * TILE_SIZE + OFFSET_FROM_WHEELBASE;
//        Navigation.turn(180, speed);
      }

    }
    
//    gameState = GameState.Navigation;

//    gameState = GameState.LightLocalization;
    Navigation.turnTo(midX, y, speed);
    Navigation.travelTo(midX, y, speed);
    odometer.setY(newY);
   // System.out.println("corrected y: "+odometer.getY());

    // THE THETA NEEDS TO BE DYNAMICALLY SET
    odometer.setTheta(firstTheta);
    //System.out.println("angle: "+odometer.getTheta());
    
    Navigation.backUp((TILE_SIZE / 3), speed);

    // This forced turn is to avoid detecting lines when it is turning.
    if (turnRight) {
      Navigation.turn(90, speed);
    } else {
      Navigation.turn(-90, speed);
    }
    Navigation.travelTo(x , (odometer.getY()/TILE_SIZE), speed);
    odometer.setX(newX);
    odometer.setTheta(secondTheta);
  //  System.out.println("corrected x: "+odometer.getX());
    //System.out.println("angle: "+odometer.getTheta());



  }
  
  /**
   * Method in which the robot travels forward until one of its 2 light sensors detects a line. When a sensor detects a line, 
   * the corresponding motor stops, and the other motor turns until its light sensor also detects a line. When 2 lines are
   * detected, the robot stops.
   */
  public void twoLineDetection() {
    float lastLeftValue = -1000;
    float currentLeftValue = 0;
    float lastRightValue = -1000;
    float currentRightValue = 0;
    leftColorSensor.setCurrentMode("Red");
    rightColorSensor.setCurrentMode("Red");
    boolean left =false, right = false;
    float[] leftSensorData = new float[3];
    float[] rightSensorData = new float[3];
    Navigation.travelForward(FORWARD_SPEED_NORMAL);
    while(!(left && right)) {
      leftColorSensor.fetchSample(leftSensorData, 0);
      rightColorSensor.fetchSample(rightSensorData, 0);
      currentLeftValue = leftSensorData[0]*100;
      currentRightValue = rightSensorData[0]*100;

      if(-currentLeftValue + lastLeftValue>=DIFFERENTIAL_LINE_THRESHOLD)
      {
        left = true;
        leftMotor.setSpeed(0);
      }
      if(-currentRightValue + lastRightValue>=DIFFERENTIAL_LINE_THRESHOLD)
      {
        right = true;
        rightMotor.setSpeed(0);
      }
      lastLeftValue = currentLeftValue;
      lastRightValue = currentRightValue;
      try {
        Thread.sleep(75);
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }

    }
  }

  /**
   * Method used to travel on the field by following the black lines directions, it enables the correction it the travel() method 
   * in the Navigation class. It allows heading correction to be peform each time 2 lines are detected.
   */
  public void squareNavigation(double x, double y) {
    enableCorrection = true;
    Navigation.travelTo(x, (odometer.getY() / TILE_SIZE), FORWARD_SPEED_NORMAL);
    Navigation.travelTo((odometer.getX() / TILE_SIZE), y, FORWARD_SPEED_NORMAL);
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
   * Method used to navigate to the launch and to turn towards the bin
   */
  public void navigateToLaunchPoint() {
    // navigate to launch point
    this.squareNavigation(launchPoint.x,launchPoint.y);
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
    Point p1 = new Point(x1, y1);
    Point p2 = new Point(x1, y2);
    Point p3 = new Point(x2, y1);
    Point p4 = new Point(x2, y2);
    // compute distances of 4 points from actual position
    x = odometer.getX() / TILE_SIZE;
    y = odometer.getY() / TILE_SIZE;
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
      System.out.println("STARTING REGION SET");

    } else {
      currentRegion = REGION.RED;
    }
  }

  /**
   * Method used to update the region after a tunnel traversal
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
   * Method setting the coordinates of the starting corner depending on the corner of the team color
   */
  public void setCorner() {
    // TODO: determine which is the corner 0
    if (color == COLOR.GREEN) {
      switch (greenCorner) {
        case 0:
          STARTING_POINT.x = 1;
          STARTING_POINT.y = 1;
          CORNER_NUMBER = 0;
          break;
        case 1:
          STARTING_POINT.x = 14;
          STARTING_POINT.y = 1;
          CORNER_NUMBER = 1;
          break;
        case 2:
          STARTING_POINT.x = 14;
          STARTING_POINT.y = 8;
          CORNER_NUMBER = 2;
          break;
        case 3:
          STARTING_POINT.x = 1;
          STARTING_POINT.y = 8;
          CORNER_NUMBER = 3;
          break;
      }
    } else {
      switch (redCorner) {
        case 0:
          STARTING_POINT.x = 1;
          STARTING_POINT.y = 1;
          CORNER_NUMBER = 0;
          break;
        case 1:
          STARTING_POINT.x = 1;
          STARTING_POINT.y = 14;
          CORNER_NUMBER = 1;
          break;
        case 2:
          STARTING_POINT.x = 8;
          STARTING_POINT.y = 14;
          CORNER_NUMBER = 2;
          break;
        case 3:
          STARTING_POINT.x = 8;
          STARTING_POINT.y = 1;
          CORNER_NUMBER = 3;
          break;
      }

    }
  }

  /**
   * Method used to compute the distance of the robot from the bin
   * 
   * @param x: x position in centimeters
   * @param y: y position in centimeters
   * 
   * @return: the distance between the robot and the bin
   */
  public double distanceFromBin(double x, double y) {
    double distance = this.calculateDistance(x, y, bin.x*TILE_SIZE, bin.y*TILE_SIZE);
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
   * Method used to calculate the closest possible launch point from the robot
   */
  public void calculateClosestLaunchPoint() {
    double x = odometer.getX();
    double y = odometer.getY();
    Point minimal_point = launchPoints.get(0);
    double minimal_distance =
        this.calculateDistance(x, y, (minimal_point.x) * TILE_SIZE, (minimal_point.y) * TILE_SIZE);
    for (Point point : launchPoints) {
      double distance = this.calculateDistance(x, y, (point.x) * TILE_SIZE, (point.y) * TILE_SIZE);
      if (distance <= minimal_distance) {
        minimal_distance = distance;
        minimal_point = point;
      }
    }
    this.launchPoint = minimal_point;
  }


  /**
   * Method used to populate the array list of possible launch points depending on the island coordinates
   */
  public void generateLaunchPoints() {
    double left = island.ll.x;
    double right = island.ur.x;
    double bottom = island.ll.y;
    double top = island.ur.y;
    // initialize or clear the list
    launchPoints = new LinkedList<Point>();
    // for all inside x values of the island
    for (int i = (int) (left + 1); i < right; i++) {
      // for all inside y values of the island
      for (int j = (int) (bottom + 1); j < top; j++) {
        Point point = new Point(i, j);
        double distance = this.distanceFromBin(i, j);
        // if the distance is smaller than the maximal launching distance, add the point into the array of launching points
        if (distance <= MAXIMAL_LAUNCH_DISTANCE) {
          boolean restricted = false;
          for(Point restrictedLaunchPoint : restrictedLaunchPoints) {
            // if the point is restricted
            if((restrictedLaunchPoint.x == point.x) && (restrictedLaunchPoint.y == point.y)) {
              restricted = true;
            }
          }
          // add the point to the list only if it is not restricted
          if(!restricted) {
            launchPoints.add(point);
          }
        }
      }
    }
  }


  /**
   * Getter Method for the tunnel entrance
   */
  public Point getTunnelEntrance() {
    return tunnelEntrance;
  }

  /**
   * Getter Method for the tunnel exit
   */
  public Point getTunnelExit() {
    return tunnelExit;
  }
  
  public void odometerInitialSet() {
    switch(CORNER_NUMBER) {
      case 0:
        odometer.setXYT(0.4*TILE_SIZE, 0.6*TILE_SIZE, odometer.getTheta());
        break;
      case 1:
        odometer.setXYT(14.4*TILE_SIZE, 0.6*TILE_SIZE, odometer.getTheta());

        break;
      case 2:
        odometer.setXYT(14.4*TILE_SIZE, 8.4*TILE_SIZE, odometer.getTheta());

        break;
      case 3:
        odometer.setXYT(0.6*TILE_SIZE, 8.4*TILE_SIZE, odometer.getTheta());

        break;
      
    }
  }

  /**
   * Getter Method for the tunnel traversal orientation
   */
  public double getTunnelTraversalOrientation() {
    return tunnelTraversalOrientation;
  }

  /**
   * Getter Method for the tunnel length
   */
  public double getTunnelLength() {
    return tunnelLength;
  }



}
