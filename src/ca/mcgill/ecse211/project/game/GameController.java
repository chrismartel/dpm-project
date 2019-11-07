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

    LCD.drawString("PRESS TO START", 1, 1);
    buttonChoice = Button.waitForAnyPress();
    if (buttonChoice == Button.ID_ENTER) {


      // Execute until the game reaches the Done state
      while (gameState != GameState.Done) {


        switch (gameState) {
          case Test:
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

            gameState = GameState.UltrasonicLocalization;
            LCD.clear();
            LCD.drawString("PRESS TO START US", 1, 1);
            buttonChoice = Button.waitForAnyPress();
            break;


          case UltrasonicLocalization:
            // perform ultrasonic localization using falling edge routine
            ultrasonicLocalizer.fallingEdge();
            Navigation.turnTo(0, ROTATE_SPEED_SLOW);
            // when us localization is done --> transition to light localization
            gameState = GameState.LightLocalization;
            goalCoordinates = STARTING_CORNER;
            LCD.clear();
            LCD.drawString("PRESS TO START LIGHT", 1, 1);
            buttonChoice = Button.waitForAnyPress();
            break;


          case LightLocalization:
            lightLocalizer.setCoordinates(STARTING_CORNER);
            odometer.setXYT(STARTING_CORNER.x * TILE_SIZE, STARTING_CORNER.y * TILE_SIZE, 0);
            // TODO: light localization using 2 sensors at the back
            gameState = GameState.Navigation;
            // set first navigation destination
            navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;
            LCD.drawString("PRESS TO START NAV", 1, 1);
            buttonChoice = Button.waitForAnyPress();
            break;


          case Navigation:
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
                  LCD.clear();
                  LCD.drawString("PRESS TO START TUNNEL", 1, 1);
                  buttonChoice = Button.waitForAnyPress();
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
                  goalCoordinates = gameNavigation.closestPoint();
                  Navigation.travelTo(goalCoordinates.x, goalCoordinates.y, FORWARD_SPEED_SLOW);
                  // TODO: light localization
                  gameNavigation.navigateToTunnel();
                  gameState = GameState.Tunnel;
                  goalCoordinates = STARTING_CORNER;
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
                  goalCoordinates = gameNavigation.getTunnelEntrance();
                }
                break;
              case END_POINT:
                Navigation.travelTo(goalCoordinates.x, goalCoordinates.y, FORWARD_SPEED_NORMAL);
                if (navigationCompleted == true) {
                  gameState = GameState.Done;
                }
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
            goalCoordinates = gameNavigation.closestPoint();
            lightLocalizer.setCoordinates(goalCoordinates);
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
    }
    LCD.clear();
    LCD.drawString("DONE", 1, 1);


  }



}
