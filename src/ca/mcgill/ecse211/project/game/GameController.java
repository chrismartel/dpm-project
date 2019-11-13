package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.FORWARD_SPEED_NORMAL;
import static ca.mcgill.ecse211.project.game.GameResources.ROTATE_SPEED_NORMAL;
import static ca.mcgill.ecse211.project.game.GameResources.LCD;
import static ca.mcgill.ecse211.project.game.GameResources.STARTING_POINT;
import static ca.mcgill.ecse211.project.game.GameResources.gameState;
import static ca.mcgill.ecse211.project.game.GameResources.leftBallisticMotor;
import static ca.mcgill.ecse211.project.game.GameResources.navigationCompleted;
import static ca.mcgill.ecse211.project.game.GameResources.navigationCoordinates;
import static ca.mcgill.ecse211.project.game.GameResources.navigationDestination;
import static ca.mcgill.ecse211.project.game.GameResources.odometer;
import static ca.mcgill.ecse211.project.game.GameResources.rightBallisticMotor;
import static ca.mcgill.ecse211.project.game.GameResources.OFFSET_FROM_WHEELBASE;
import static ca.mcgill.ecse211.project.game.GameResources.ultrasonicPoller;
import ca.mcgill.ecse211.project.game.GameResources.NAVIGATION_DESTINATION;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;

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
                    
          // set first checkpoint
          navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;
          
          //Warm up motors
          Navigation.travel(2, FORWARD_SPEED_NORMAL);
          Navigation.backUp(2, FORWARD_SPEED_NORMAL);
          leftBallisticMotor.rotate(-5, true);
          rightBallisticMotor.rotate(5, false);
          
          // transit to ultrasonic localization state
          gameState = GameState.UltrasonicLocalization;
          LCD.drawString("PRESS TO START US", 1, 1);
          buttonChoice = Button.waitForAnyPress();
          LCD.clear();
          break;


        case UltrasonicLocalization:
          // perform ultrasonic localization using falling edge routine
          ultrasonicLocalizer.fallingEdge();
          // when us localization is done --> transition to light localization
          gameState = GameState.LightLocalization;
          LCD.drawString("PRESS TO START LIGHT", 1, 1);
          LCD.clear();
          break;


        case LightLocalization:

          gameNavigation.odometerInitialSet();

          //gameNavigation.lightLocalize(STARTING_POINT);
          gameNavigation.lightCorrect(30.48,30.48,FORWARD_SPEED_NORMAL);
          gameState = GameState.Navigation;

          LCD.drawString("PRESS TO START NAV", 1, 1);
          buttonChoice = Button.waitForAnyPress();
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
//                gameState = GameState.LightLocalization;
//                gameNavigation.lightLocalize(gameNavigation.closestPoint());
                
                //gameNavigation.navigateToTunnel();


                // transition to tunnel state
                gameState = GameState.Tunnel;

                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
                LCD.clear();
                LCD.drawString("PRESS TO START TUNNEL", 1, 1);
                //buttonChoice = Button.waitForAnyPress();
              }
              break;
            case TUNNEL2_ENTRANCE:
              // travel to the tunnel entrance
              gameNavigation.navigateToTunnel();
              // if navigation is completed --> transition to tunnel state + update the navigation point to the launch
              // point
              if (navigationCompleted == true) {

                // LIGHT LOCALIZATION
//                gameState = GameState.LightLocalization;
//                gameNavigation.lightLocalize(gameNavigation.closestPoint());
                
                //gameNavigation.navigateToTunnel();


                // transition to tunnel state
                gameState = GameState.Tunnel;

                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
                LCD.clear();
                LCD.drawString("PRESS TO START TUNNEL", 1, 1);
              }
              break;
            case LAUNCH_POINT:
              gameNavigation.calculateClosestLaunchPoint();
              gameNavigation.navigateToLaunchPoint();
              if (navigationCompleted == true) {

                // LIGHT LOCALIZATION
                gameState = GameState.LightLocalization;            
                gameNavigation.lightCorrect(gameNavigation.closestPoint().x,gameNavigation.closestPoint().y,120);
                gameState = GameState.Navigation;
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
          gameNavigation.twoLineDetection();
          odometer.setTheta(0);
          System.out.println(odometer.getX() + " " + odometer.getY() + " " + odometer.getTheta());
          Navigation.backUp(OFFSET_FROM_WHEELBASE);
          gameNavigation.navigateThroughTunnel();
          
          gameNavigation.lightCorrect(gameNavigation.closestPoint().x*30.48, gameNavigation.closestPoint().y*30.48, 130);
          System.out.println(odometer.getX() + " " + odometer.getY() + " " + odometer.getTheta());

          gameState = GameState.Navigation;


          // LIGHT LOCALIZATION
          //gameState = GameState.LightLocalization;
          //gameNavigation.lightCorrect(gameNavigation.closestPoint().x,gameNavigation.closestPoint().y,120);
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          obstacleAvoider.wallFollower();
          // regenerate the launch points
          gameNavigation.generateLaunchPoints();

          // LIGHT LOCALIZATION
          gameState = GameState.LightLocalization;
          gameNavigation.lightLocalize(gameNavigation.closestPoint());
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


