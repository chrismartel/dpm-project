package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import lejos.hardware.Button;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;

public class GameController {

  public static void main(String[] args) {
    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObstacleAvoider obstacleAvoider = new ObstacleAvoider();
    int buttonChoice;

    // set initial state
    gameState = GameState.Initialization;

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
          gameNavigation.setGameParameters();
                    
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
          gameNavigation.lightLocalize(STARTING_POINT);

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

                // LIGHT LOCALIZATION
                gameState = GameState.LightLocalization;
                localizationCoordinates = gameNavigation.closestPoint();
                gameNavigation.lightLocalize(localizationCoordinates);
                gameNavigation.navigateToTunnel();


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
                gameState = GameState.LightLocalization;
                localizationCoordinates = gameNavigation.closestPoint();
                gameNavigation.lightLocalize(localizationCoordinates);
                gameNavigation.navigateToTunnel();

                // transition to tunnel state
                gameState = GameState.Tunnel;

                // update new checkpoint
                navigationCoordinates = STARTING_POINT;
                navigationDestination = NAVIGATION_DESTINATION.END_POINT;
              }
              break;
            case LAUNCH_POINT:
              gameNavigation.squareNavigation(1, 1);
              gameNavigation.calculateClosestLaunchPoint();
              gameNavigation.navigateToLaunchPoint();
              if (navigationCompleted == true) {

                // LIGHT LOCALIZATION
                gameState = GameState.LightLocalization;
                localizationCoordinates = gameNavigation.closestPoint();
                gameNavigation.lightLocalize(localizationCoordinates);
                gameNavigation.navigateToLaunchPoint();

                // Transit to launch state
                gameState = GameState.Launch;

                // update new checkpoint
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

          // LIGHT LOCALIZATION
          gameState = GameState.LightLocalization;
          localizationCoordinates = gameNavigation.closestPoint();
          gameNavigation.lightLocalize(localizationCoordinates);
          gameState = GameState.Navigation;
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          obstacleAvoider.wallFollower();
          // regenerate the launch points
          gameNavigation.generateLaunchPoints();

          // LIGHT LOCALIZATION
          gameState = GameState.LightLocalization;
          localizationCoordinates = gameNavigation.closestPoint();
          gameNavigation.lightLocalize(localizationCoordinates);
          gameState = GameState.Navigation;
          break;


        case Launch:
          // perform the launches
          ballisticLauncher.setDistance(gameNavigation.distanceFromBin(odometer.getX(), odometer.getY()));
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


