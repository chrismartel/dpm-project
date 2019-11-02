package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;


public class GameNavigation {

  public enum REGION {
    RED, WATER, TUNNEL_RED, TUNNEL_GREEN, GREEN, ISLAND
  }

  private double [] tunnelEntrance;
  
  private void navigateToTunnel() {
    this.navigationWithCorrection(tunnelEntrance[0], tunnelEntrance[1]);
  }
  
  
  /**
   * Method used to navigate to a specific point using correction after traveling a specfic number of tiles
   */
  private void navigationWithCorrection(double x, double y) {
    
    
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
  private void setTunnelEntrance() {
    
    REGION targetRegion;
    REGION tunnelBottom = this.regionCalculation((TUNNEL_LL[0] + 0.5), (TUNNEL_LL[1] - 0.5));
    REGION tunnelTop = this.regionCalculation((TUNNEL_UR[0] - 0.5), (TUNNEL_UR[1] + 0.5));
    REGION tunnelLeft = this.regionCalculation((TUNNEL_LL[0] - 0.5), (TUNNEL_LL[1] + 0.5));
    REGION tunnelRight = this.regionCalculation((TUNNEL_UR[0] + 0.5), (TUNNEL_UR[1] - 0.5));

    // determine the target region
    if (currentColor == COLOR.RED && currentRegion == REGION.RED) {
      targetRegion = REGION.RED;
    } else if(currentColor == COLOR.GREEN && currentRegion == REGION.GREEN){
      targetRegion = REGION.GREEN;
    }
    else {
      targetRegion = REGION.ISLAND;
    }
    // determine where is the tunnel entrance
    if (tunnelBottom == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_LL[0] + 0.5);
      this.tunnelEntrance[1] = (TUNNEL_LL[1] - 0.5);
    } else if (tunnelTop == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] + 0.5);
    } else if (tunnelLeft == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_LL[0] - 0.5);
      this.tunnelEntrance[1] = (TUNNEL_LL[1] + 0.5);
    } else if (tunnelRight == targetRegion) {
      this.tunnelEntrance[0] = (TUNNEL_UR[0] + 0.5);
      this.tunnelEntrance[1] = (TUNNEL_UR[1] - 0.5);
    }
  }

public double[] getTunnelEntrance() {
  return tunnelEntrance;
}


}
