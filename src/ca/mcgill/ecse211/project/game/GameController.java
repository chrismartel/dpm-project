package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Button;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;

public class GameController {

  public enum NAVIGATION_DESTINATION {
    TUNNEL1_ENTRANCE, TUNNEL2_ENTRANCE, LAUNCH_POINT, END_POINT, LOCALIZE
  }

  public static void main(String[] args) {
    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    LightLocalizer lightLocalizer = new LightLocalizer();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObjectAvoider objectAvoider = new ObjectAvoider();
    int buttonChoice;

    // set initial state

    gameState = GameState.Initialization;
    // gameState = GameState.Test;

      // Execute until the game reaches the Done state
      while (gameState != GameState.Done) {


        switch (gameState) {
          case Test:
            odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
            gameNavigation.squareNavigation(3, 4);
            gameState = GameState.Done;
            

            // ****** TEST 1 ******* //
            // DONE
            /*
             * robot sets the tunnel data for its zone and navigate to the entrance, then traverses the tunnel Test both
             * tunnel orientations
             */
            /*
             * Thread odometerThread = new Thread(odometer); Thread usPollerTread = new Thread(ultrasonicPoller);
             * odometerThread.start(); usPollerTread.start(); color = COLOR.GREEN; navigationDestination =
             * NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE; gameNavigation.setStartingRegion(); gameNavigation.setCorner();
             * gameNavigation.setLimits(); gameNavigation.setTunnel(); gameNavigation.updateTunnelData();
             * goalCoordinates = gameNavigation.getTunnelEntrance(); odometer.setXYT(STARTING_CORNER[0] * TILE_SIZE,
             * STARTING_CORNER[1] * TILE_SIZE, 0); gameNavigation.navigateToTunnel();
             * gameNavigation.navigateThroughTunnel(); gameState = GameState.Done;
             */

            // ***** END OF TEST 1 ****** //



            // ****** TEST 2 ******* //
            // DONE
            // Test the method returning the closest point
            /*
             * Thread odometerThread = new Thread(odometer); Thread usPollerTread = new Thread(ultrasonicPoller);
             * odometerThread.start(); usPollerTread.start(); //odometer.setXYT(1.7* TILE_SIZE, 3.5 * TILE_SIZE, 0);
             * //odometer.setXYT(3* TILE_SIZE, 3 * TILE_SIZE, 0); //odometer.setXYT(5.4* TILE_SIZE, 2.3 * TILE_SIZE, 0);
             * //odometer.setXYT(8.4* TILE_SIZE, 7 * TILE_SIZE, 0);
             * 
             * int[] point = gameNavigation.closestPoint(); System.out.println("closest point: "+point[0] + ", " +
             * point[1]); gameState = GameState.Done;
             */
            // ***** END OF TEST 2 ****** //
            break;


          case Initialization:
            // start threads
            Thread odometerThread = new Thread(odometer);
            Thread usPollerTread = new Thread(ultrasonicPoller);
            odometerThread.start();
            usPollerTread.start();

            // TODO: get data fom wifi class

            // generate map
            gameNavigation.setColor();
            gameNavigation.setStartingRegion();
            gameNavigation.setCorner();
            gameNavigation.setLimits();
            gameNavigation.setTunnel();
            gameNavigation.updateTunnelData();
            // set first waypoint
            navigationCoordinates = gameNavigation.getTunnelEntrance();
            navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;


            gameState = GameState.Test;
            LCD.clear();
            LCD.drawString("PRESS TO START", 1, 1);
            buttonChoice = Button.waitForAnyPress();
            LCD.clear();
            break;


          case UltrasonicLocalization:
            // perform ultrasonic localization using falling edge routine
            ultrasonicLocalizer.fallingEdge();
            // when us localization is done --> transition to light localization
            gameState = GameState.LightLocalization;
            localizationCoordinates = STARTING_CORNER;
            LCD.drawString("PRESS TO START LIGHT", 1, 1);
            buttonChoice = Button.waitForAnyPress();
            LCD.clear();
            break;


          case LightLocalization:
            lightLocalizer.setCoordinates(localizationCoordinates);
            odometer.setXYT(14*TILE_SIZE, 1*TILE_SIZE, 0);
            // TODO: light localization using 2 sensors at the back
            gameState = GameState.Navigation;
            // set first navigation destination
            LCD.drawString("PRESS TO START NAV", 1, 1);
            buttonChoice = Button.waitForAnyPress();
            LCD.clear();
            break;


          case Navigation:
            // navigation is considered uncompleted initially
            navigationCompleted = false;
            // check the current navigation type
            switch (navigationDestination) {
              case TUNNEL1_ENTRANCE:
                // travel to the tunnel entrance
                gameNavigation.navigateToTunnel();
                // if navigation is completed --> transition to tunnel state + update the navigation point to the launch
                // point
                if (navigationCompleted == true) {
                  gameState = GameState.Tunnel;
                  // update new checkpoint
                  navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
                  LCD.clear();
                  LCD.drawString("PRESS TO START TUNNEL", 1, 1);
                  buttonChoice = Button.waitForAnyPress();
                }
                break;
              case TUNNEL2_ENTRANCE:
                // travel to the tunnel entrance
                gameNavigation.navigateToTunnel();
                // when navigation is completed --> transition to tunnel state + update the destination point to the end
                // point
                if (navigationCompleted == true) {
                  localizationCoordinates = gameNavigation.closestPoint();
                  Navigation.travelTo(localizationCoordinates.x, localizationCoordinates.y, FORWARD_SPEED_SLOW);
                  // TODO: light localization
                  gameNavigation.navigateToTunnel();
                  gameState = GameState.Tunnel;
                  // update new checkpoint
                  navigationCoordinates = STARTING_CORNER;
                  navigationDestination = NAVIGATION_DESTINATION.END_POINT;
                }
                break;
              case LAUNCH_POINT:
                gameNavigation.squareNavigation(1, 1);
                // TODO: compute possible launch points --> waiting on hardware for this part
                // TODO: set goal coordinates
                // TODO: navigate to launch point
                // if navigation to launch point is completed --> transition to launching state + update the new
                // destination
                if (navigationCompleted == true) {
                  gameState = GameState.Launch;
                  // update new navigation destination
                  navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
                  navigationCoordinates = gameNavigation.getTunnelEntrance();
                }
                break;
              case END_POINT:
                gameNavigation.squareNavigation(STARTING_CORNER.x, STARTING_CORNER.y);
                if (navigationCompleted == true) {
                  gameState = GameState.Done;
                }
                break;
              default:
                break;
            }
            break;


          case Tunnel:
            gameNavigation.navigateThroughTunnel();
            // when navigation through tunnel is completed
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
            // reset the zone limits and the tunnel data when the traversal is completed
            gameNavigation.updateTunnelData();
            gameNavigation.setLimits();
            localizationCoordinates = gameNavigation.closestPoint();
            lightLocalizer.setCoordinates(localizationCoordinates);
            Navigation.travelTo(localizationCoordinates.x,localizationCoordinates.y, FORWARD_SPEED_SLOW);
            // TODO: localize
            // after a tunnel traversal --> transition to navigation state
            gameState = GameState.Navigation;
            break;


          case Avoidance:
            // object avoidance procedure using wall follower with P-Controller
            objectAvoider.wallFollower();
            // object avoidance is over --> transition to navigation
            gameState = GameState.Navigation;
            break;


          case Launch:
            // TODO: launch and reload --> waiting after hardware for this part
            // when launch is completed --> transition to navigation
            gameState = GameState.Navigation;
            break;

        }
      }
      LCD.clear();
      LCD.drawString("DONE", 1, 1);
    }

}




