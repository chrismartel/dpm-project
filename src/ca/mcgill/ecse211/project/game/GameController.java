package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Display;
import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
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
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObstacleAvoider obstacleAvoider = new ObstacleAvoider();


    // TO TEST LIGHT LOCALIZATION

    // Thread odometerThread = new Thread(odometer);
    // odometerThread.start();
    //
    // Thread displayThread = new Thread(new Display());
    // displayThread.start();


    int buttonChoice;

    // set initial state

    // gameState = GameState.Initialization;
    // gameState = GameState.Test;
    gameState = GameState.LightLocalization;

    // Execute until the game reaches the Done state
    while (gameState != GameState.Done) {


      switch (gameState) {
        case Test:

          break;


        case Initialization:
          // start threads
          Thread odometerThread = new Thread(odometer);
          Thread usPollerTread = new Thread(ultrasonicPoller);
          odometerThread.start();
          usPollerTread.start();
          // Initialize map
          gameNavigation.setParameters();
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
          localizationCoordinates = STARTING_POINT;
          LCD.drawString("PRESS TO START LIGHT", 1, 1);
          buttonChoice = Button.waitForAnyPress();
          LCD.clear();
          break;


        case LightLocalization:

          Sound.beep();
          odometer.setX(0.5 * TILE_SIZE);
          odometer.setY(0.5 * TILE_SIZE);
          // lightLocalizer.setCoordinates(localizationCoordinates);
          // odometer.setXYT(14*TILE_SIZE, 1*TILE_SIZE, 0);
          // TODO: light localization using 2 sensors at the back
          // gameState = GameState.Navigation;
          gameNavigation.lightLocalize(new Point(1, 1));
          // set first navigation destination
          // LCD.drawString("PRESS TO START NAV", 1, 1);
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
                // LIGHT LOCALIZATION
                localizationCoordinates = gameNavigation.closestPoint();
                Navigation.travelTo(localizationCoordinates.x, localizationCoordinates.y, FORWARD_SPEED_SLOW);
                // TODO: light localize
                Navigation.travelTo(navigationCoordinates.x, navigationCoordinates.y, FORWARD_SPEED_SLOW);

                // transition to tunnel state
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

                // LIGHT LOCALIZATION
                localizationCoordinates = gameNavigation.closestPoint();
                Navigation.travelTo(localizationCoordinates.x, localizationCoordinates.y, FORWARD_SPEED_SLOW);
                // TODO: light localize
                Navigation.travelTo(navigationCoordinates.x, navigationCoordinates.y, FORWARD_SPEED_SLOW);

                // transition to tunnel state
                gameState = GameState.Tunnel;
                // update new checkpoint
                navigationCoordinates = STARTING_POINT;
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
                // TODO: light localize
                gameState = GameState.Launch;
                // update new navigation destination
                navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
                navigationCoordinates = gameNavigation.getTunnelEntrance();
              }
              break;
            case END_POINT:
              gameNavigation.squareNavigation(STARTING_POINT.x, STARTING_POINT.y);
              if (navigationCompleted == true) {
                gameState = GameState.Done;
              }
              break;
            default:
              break;
          }
          break;


        case Tunnel:
          // navigate through tunnel
          gameNavigation.navigateThroughTunnel();
          localizationCoordinates = gameNavigation.closestPoint();
          Navigation.travelTo(localizationCoordinates.x, localizationCoordinates.y, FORWARD_SPEED_SLOW);

          // TODO: localize
          // after a tunnel traversal --> transition to navigation state
          gameState = GameState.Navigation;
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          obstacleAvoider.wallFollower();
          // object avoidance is over --> transition to navigation
          gameState = GameState.Navigation;
          break;


        case Launch:
          // perform the launches
          ballisticLauncher.multipleLaunch();
          // transition back to navigation
          gameState = GameState.Navigation;
          break;

        default:
          break;

      }
    }
    LCD.clear();
    LCD.drawString("DONE", 1, 1);
  }



}


