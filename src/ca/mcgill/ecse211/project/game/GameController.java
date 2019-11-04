package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.game.GameNavigation.REGION;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;

public class GameController {

  public enum NAVIGATION_DESTINATION {
    TUNNEL1_ENTRANCE, TUNNEL1_EXIT, TUNNEL2_ENTRANCE, TUNNEL2_EXIT, LAUNCH_POINT, END_POINT,
  }

  public static void main(String[] args) {

    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();;
    LightLocalizer lightLocalizer = new LightLocalizer();;
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
        // TODO: get info from wifi class and generate map
        // set currentRegion and color depending on wifi
        /*
         * if() { color = COLOR.RED; } else { color = COLOR.GREEN;
         * 
         * }
         */
        // set tunnel
        gameNavigation.setTunnel();
        gameState = GameState.UltrasonicLocalization;
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
            // if navigation is completed --> transition to tunnel state and update the next navigation destination
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_EXIT;
            }
            break;
          case TUNNEL2_ENTRANCE:
            goalCoordinates = gameNavigation.getTunnelEntrance();
            Navigation.travelTo(goalCoordinates[0], goalCoordinates[1], FORWARD_SPEED_NORMAL);
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_EXIT;
            }
            break;
          case LAUNCH_POINT:
            // TODO: navigate to launch point
            // TODO: set goal coordinates
            if (navigationCompleted == true) {
              gameState = GameState.Launch;
              navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
            }
            break;
          case END_POINT:
            goalCoordinates = STARTING_CORNER;
            Navigation.travelTo(goalCoordinates[0], goalCoordinates[1], FORWARD_SPEED_NORMAL);
            if (navigationCompleted == true) {
              gameState = GameState.Done;
            }
            break;
        }
      }

      // tunnel traversal process
      else if (gameState == GameState.Tunnel) {
        // TODO: navigate through tunnel
        tunnelCompleted = false;
        gameNavigation.navigateThroughTunnel();
        // if navigation through tunnel is completed
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
          // set the navigation destination
          // if exits tunnel 1 --> navigation destination to launch point
          if(navigationDestination == NAVIGATION_DESTINATION.TUNNEL1_EXIT) {
            navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
          }
          // if exits tunnel 2 --> navigation destination to end point
          else if(navigationDestination == NAVIGATION_DESTINATION.TUNNEL2_EXIT) {
            navigationDestination = NAVIGATION_DESTINATION.END_POINT;
          }
          // after a tunnel traversal --> transition to navigation state
          gameState = GameState.Navigation;
        }

      }

      // object avoidance process
      else if (gameState == GameState.Avoidance) {
        // TODO: object avoidance process
        // set goal coordinates of the object avoider



      }

      // launch process
      else if (gameState == GameState.Launch) {
        // TODO: launch and reload
        navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
      }

    }



  }



}
