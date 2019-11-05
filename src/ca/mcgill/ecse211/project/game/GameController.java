package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.game.GameNavigation.REGION;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;

public class GameController {

  public enum NAVIGATION_DESTINATION {
    TUNNEL1_ENTRANCE, TUNNEL2_ENTRANCE, LAUNCH_POINT, END_POINT,
  }

  public static void main(String[] args) {
    // for tests, supposing we are green team
    // MAP 1
    TNG_LL[0] = 2;
    TNG_LL[1] = 3;
    TNG_UR[0] = 3;
    TNG_UR[1] = 4;
    
    TNR_LL[0] = 0;
    TNR_LL[1] = 0;
    TNR_UR[0] = 0;
    TNR_UR[1] = 0;
    
    RED_LL[0]=0;
    RED_LL[1]=0;
    RED_UR[0]=0;
    RED_UR[1]=0;
    
    GREEN_LL[0]=0;
    GREEN_LL[1]=0;
    GREEN_UR[0]=4;
    GREEN_UR[1]=3;
    
    ISLAND_LL[0]=0;
    ISLAND_LL[1]=4;
    ISLAND_UR[0]=5;
    ISLAND_UR[1]=10;
    
    BIN[0] = -4;
    BIN[1]= 2;
    GREEN_CORNER = 0;

    
    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    LightLocalizer lightLocalizer = new LightLocalizer();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObjectAvoider objectAvoider = new ObjectAvoider();

    // set initial state
    gameState = GameState.Initialization;

    // Execute until the game reaches the Done state
    while (gameState != GameState.Done) {

      // initialization process
      if (gameState == GameState.Initialization) {
        // initiliaze threads
        Thread odometerThread = new Thread(odometer);
        Thread usPollerTread = new Thread(ultrasonicPoller);
        // start threads
        odometerThread.start();
        usPollerTread.start();

        //TEST navigation to tunnel
        color = COLOR.GREEN;
        gameNavigation.setStartingRegion();
        gameNavigation.setCorner();
        gameNavigation.setTunnel();
        gameNavigation.updateTunnelData();
        odometer.setXYT(STARTING_CORNER[0]*TILE_SIZE, STARTING_CORNER[1]*TILE_SIZE, 0);
        gameNavigation.navigateToTunnel();
        gameState = GameState.Done;
        
        // TODO: get info from wifi class and generate map
        // set currentRegion and color depending on wifi
        // set tunnel
        //gameNavigation.setTunnel();
        //gameNavigation.setLimits();
        //gameState = GameState.UltrasonicLocalization;
      }

      // ultrasonic localization process
      else if (gameState == GameState.UltrasonicLocalization) {
        // TODO: us localization using falling edge
        ultrasonicLocalizer.fallingEdge();
        // when us localization is done --> transition to light localization
        gameState = GameState.LightLocalization;
      }

      // light localization process
      else if (gameState == GameState.LightLocalization) {
        lightLocalizer.setCoordinates(STARTING_CORNER);
        // TODO: light localization using 2 sensors at the back
        // when light localization is done --> transition to navigation
        gameState = GameState.Navigation;
        // first navigation destination --> first tunnel entrance
        navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;
      }

      // navigation process
      else if (gameState == GameState.Navigation) {
        // navigation is considered uncompleted initially
        navigationCompleted = false;
        // check the current navigation type
        switch (navigationDestination) {
          case TUNNEL1_ENTRANCE:
            // set the goal coordinates to the tunnel entrance
            goalCoordinates = gameNavigation.getTunnelEntrance();
            // travel to the tunnel entrance
            gameNavigation.navigateToTunnel();
            // if navigation is completed --> transition to tunnel state + update the navigation point to the launch
            // point
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
            }
            break;
          case TUNNEL2_ENTRANCE:
            // update the tunnel entrance and exit
            gameNavigation.setTunnel();
            // set the goal coordinates to the tunnel entrance
            goalCoordinates = gameNavigation.getTunnelEntrance();
            // travel to the tunnel entrance
            gameNavigation.navigateToTunnel();
            // when navigation is completed --> transition to tunnel state + update the destination point to the end
            // point
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationDestination = NAVIGATION_DESTINATION.END_POINT;
            }
            break;
          case LAUNCH_POINT:
            // TODO: compute possible launch points --> waiting on hardware for this part
            // TODO: set goal coordinates
            // TODO: navigate to launch point
            // if navigation to launch point is completed --> transition to launching state + update the new destination
            if (navigationCompleted == true) {
              gameState = GameState.Launch;
              navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
            }
            break;
          case END_POINT:
            goalCoordinates = (double[])STARTING_CORNER;
            Navigation.travelTo(goalCoordinates[0], goalCoordinates[1], FORWARD_SPEED_NORMAL);
            if (navigationCompleted == true) {
              gameState = GameState.Done;
            }
            break;
        }
      }

      // tunnel traversal process
      else if (gameState == GameState.Tunnel) {
        tunnelCompleted = false;
        gameNavigation.navigateThroughTunnel();
        // when navigation through tunnel is completed
        if (tunnelCompleted == true) {
          // update the current region of the robot
          if (currentRegion == REGION.GREEN || currentRegion == REGION.RED) {
            currentRegion = REGION.ISLAND;
          } else if (currentRegion == REGION.ISLAND) {
            if (color == COLOR.GREEN) {
              currentRegion = REGION.GREEN;
            } else {
              currentRegion = REGION.RED;
            }
          }
          // reset the zone limits when the traversal is completed
          gameNavigation.setLimits();
          // after a tunnel traversal --> transition to navigation state
          gameState = GameState.Navigation;
        }

      }

      // object avoidance process
      else if (gameState == GameState.Avoidance) {
        // object avoidance procedure using wall follower with P-Controller
        objectAvoider.wallFollower();
        // object avoidance is over --> transition to navigation
        gameState = GameState.Navigation;
      }

      // launch process
      else if (gameState == GameState.Launch) {
        // TODO: launch and reload --> waiting after hardware for this part
        // when launch is completed --> transition to navigation
        gameState = GameState.Navigation;
      }

    }
    LCD.drawString("DONE", 1, 1);


  }



}
