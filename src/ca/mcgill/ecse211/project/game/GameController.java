package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.game.GameNavigation.REGION;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;

public class GameController {

  public enum NAVIGATION_DESTINATION {
    TUNNEL1_ENTRANCE, TUNNEL2_ENTRANCE, LAUNCH_POINT, END_POINT,
  }

  public static void main(String[] args) {


    // MAP 1
    // suppose we are green team
    TNG_LL[0] = 4;
    TNG_LL[1] = 1;
    TNG_UR[0] = 6;
    TNG_UR[1] = 2;

    TNR_LL[0] = 0;
    TNR_LL[1] = 0;
    TNR_UR[0] = 0;
    TNR_UR[1] = 0;

    RED_LL[0] = 0;
    RED_LL[1] = 0;
    RED_UR[0] = 0;
    RED_UR[1] = 0;

    GREEN_LL[0] = 0;
    GREEN_LL[1] = 0;
    GREEN_UR[0] = 4;
    GREEN_UR[1] = 4;

    ISLAND_LL[0] = 6;
    ISLAND_LL[1] = 0;
    ISLAND_UR[0] = 9;
    ISLAND_UR[1] = 4;

    BIN[0] = -4;
    BIN[1] = 2;
    // bottom left corner
    GREEN_CORNER = 0;


    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    LightLocalizer lightLocalizer = new LightLocalizer();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObjectAvoider objectAvoider = new ObjectAvoider();

    // set initial state
    gameState = GameState.Initialization;
    LCD.drawString("PRESS TO START", 1, 1);
    int buttonChoice = Button.waitForAnyPress();
    if (buttonChoice == Button.ID_ENTER) {


      // Execute until the game reaches the Done state
      while (gameState != GameState.Done) {

        // initialization process
        if (gameState == GameState.Initialization) {
          /*
          // initiate and start threads
          Thread odometerThread = new Thread(odometer);
          Thread usPollerTread = new Thread(ultrasonicPoller);
          odometerThread.start();
          usPollerTread.start();
          // generate map
          gameNavigation.setStartingRegion();
          gameNavigation.setCorner();
          gameNavigation.setLimits();
          gameNavigation.setTunnel();
          gameNavigation.updateTunnelData();
          goalCoordinates = gameNavigation.getTunnelEntrance();
          gameState = GameState.UltrasonicLocalization;
          */

          // ****** TEST 1 ******* //
          // DONE
          /*robot sets the tunnel data for its zone and navigate to the entrance, then traverses the tunnel
           * Test both tunnel orientations
           */
          /*
          Thread odometerThread = new Thread(odometer);
          Thread usPollerTread = new Thread(ultrasonicPoller);
          odometerThread.start();
          usPollerTread.start();
          color = COLOR.GREEN;
          navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;
          gameNavigation.setStartingRegion();
          gameNavigation.setCorner();
          gameNavigation.setLimits();
          gameNavigation.setTunnel();
          gameNavigation.updateTunnelData();
          goalCoordinates = gameNavigation.getTunnelEntrance();
          odometer.setXYT(STARTING_CORNER[0] * TILE_SIZE, STARTING_CORNER[1] * TILE_SIZE, 0);
          gameNavigation.navigateToTunnel();
          gameNavigation.navigateThroughTunnel();
          gameState = GameState.Done;*/
          
          // ***** END OF TEST 1 ****** //

          // ****** TEST 2 ******* //
          // DONE
          //Test the method returning the closest point
          /*
          Thread odometerThread = new Thread(odometer);
          Thread usPollerTread = new Thread(ultrasonicPoller);
          odometerThread.start();
          usPollerTread.start();
          //odometer.setXYT(1.7* TILE_SIZE, 3.5 * TILE_SIZE, 0);
          //odometer.setXYT(3* TILE_SIZE, 3 * TILE_SIZE, 0);
          //odometer.setXYT(5.4* TILE_SIZE, 2.3 * TILE_SIZE, 0);
          //odometer.setXYT(8.4* TILE_SIZE, 7 * TILE_SIZE, 0);

          int[] point = gameNavigation.closestPoint();
          System.out.println("closest point: "+point[0] + ", " + point[1]);
          gameState = GameState.Done;
          */
          // ***** END OF TEST 2 ****** //
        }

        /*
         * Ultrasonic Localization
         */
        else if (gameState == GameState.UltrasonicLocalization) {
          // TODO: us localization using falling edge
          ultrasonicLocalizer.fallingEdge();
          // when us localization is done --> transition to light localization
          gameState = GameState.LightLocalization;
        }

        /*
         * Light Localization
         */
        else if (gameState == GameState.LightLocalization) {
          lightLocalizer.setCoordinates(STARTING_CORNER);
          // TODO: light localization using 2 sensors at the back
          // when light localization is done --> transition to navigation
          gameState = GameState.Navigation;
          // first navigation destination --> first tunnel entrance
          navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;
        }

        /*
         * Navigation
         */
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
              // if navigation to launch point is completed --> transition to launching state + update the new
              // destination
              if (navigationCompleted == true) {
                gameState = GameState.Launch;
                navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
              }
              break;
            case END_POINT:
              goalCoordinates = (double[]) STARTING_CORNER;
              Navigation.travelTo(goalCoordinates[0], goalCoordinates[1], FORWARD_SPEED_NORMAL);
              if (navigationCompleted == true) {
                gameState = GameState.Done;
              }
              break;
          }
        }

        /*
         * Tunnel Traversal
         */
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

        /*
         * Object Avoidance
         */
        else if (gameState == GameState.Avoidance) {
          // object avoidance procedure using wall follower with P-Controller
          objectAvoider.wallFollower();
          // object avoidance is over --> transition to navigation
          gameState = GameState.Navigation;
        }

        /*
         * Launch
         */
        else if (gameState == GameState.Launch) {
          // TODO: launch and reload --> waiting after hardware for this part
          // when launch is completed --> transition to navigation
          gameState = GameState.Navigation;
        }

      }
    }
    LCD.clear();
    LCD.drawString("DONE", 1, 1);


  }



}
