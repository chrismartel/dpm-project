package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Resources;
import ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Localization.UltrasonicLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class GameController {

  public static void main(String[] args) {
    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObstacleAvoider obstacleAvoider = new ObstacleAvoider();
    Point closestPoint;

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
          rightBallisticMotor.rotate(-5, false);
          leftBallisticMotor.rotate(+5, true);
          rightBallisticMotor.rotate(+5, false);
          
          // transit to ultrasonic localization state
          gameState = GameState.UltrasonicLocalization;
          break;


        case UltrasonicLocalization:
          // perform ultrasonic localization using falling edge routine
          ultrasonicLocalizer.fallingEdge(ROTATE_SPEED_NORMAL);
          // when us localization is done --> transition to light localization
          gameState = GameState.LightLocalization;
          break;


        case LightLocalization:

          gameNavigation.odometerInitialSet();

          //gameNavigation.lightLocalize(STARTING_POINT);
          gameNavigation.lightLocalize2(STARTING_POINT);
          gameState = GameState.Navigation;
          break;


        case Navigation:
          // navigation is considered uncompleted initially
          navigationCompleted = false;
          // check the current navigation type
          switch (navigationDestination) {
            case TUNNEL1_ENTRANCE:
              // square travel to the tunnel entrance using correction
              gameNavigation.navigateToTunnel();
              
              if (navigationCompleted == true) {
                // transition to tunnel state
                gameState = GameState.Tunnel;
                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
              }
              break;
            case TUNNEL2_ENTRANCE:
              // travel to the tunnel entrance
              gameNavigation.navigateToTunnel();
              // if navigation is completed --> transition to tunnel state + update the navigation point to the launch
              // point
              if (navigationCompleted == true) {

                // LIGHT LOCALIZATION
                closestPoint = gameNavigation.closestPoint();
                Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
                gameNavigation.lightLocalize2(closestPoint);
                gameState = GameState.Navigation;
                
                gameNavigation.navigateToTunnel();


                // transition to tunnel state
                gameState = GameState.Tunnel;

                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
              }
              break;
            case LAUNCH_POINT:
              gameNavigation.calculateClosestLaunchPoint();
              gameNavigation.navigateToLaunchPoint();
              if (navigationCompleted == true) {

                // LIGHT LOCALIZATION
                closestPoint = gameNavigation.closestPoint();
                Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
                gameNavigation.lightLocalize2(closestPoint);
                gameState = GameState.Navigation;
                
                // navigate to launch poitn after localizing
                gameNavigation.navigateToLaunchPoint();

                // Transit to launch state
                gameState = GameState.Launch;

                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
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
          // adjust heading
          gameNavigation.twoLineDetection();
          // position center of rotation at tunnel entrance
          Navigation.backUp(OFFSET_FROM_WHEELBASE);
          // correct odometer according to tunnel entrance data
          odometer.setXYT(gameNavigation.getTunnelEntrance().x, gameNavigation.getTunnelEntrance().y, gameNavigation.getTunnelTraversalOrientation());
          // navigate through tunnel
          gameNavigation.navigateThroughTunnel();
          // update the region, the tunnel data and the zone limits
          gameNavigation.updateParameters();
          
          // LIGHT LOCALIZATION
          // find closest point
          closestPoint = gameNavigation.closestPoint();
          // travel to closest point
          Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
          // localize according to the closest point
          gameNavigation.lightLocalize2(closestPoint);
          
          // transition back to navigation
//          gameState = GameState.Navigation;
          
          //***** FOR DEMO
          gameState = GameState.Demo;
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          obstacleAvoider.wallFollower();
          // regenerate the launch points
          gameNavigation.generateLaunchPoints();

          // LIGHT LOCALIZATION
          closestPoint = gameNavigation.closestPoint();
          Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
          gameNavigation.lightLocalize2(closestPoint);
          gameState = GameState.Navigation;

          break;


        case Launch:
          // perform the launches
          ballisticLauncher.multipleLaunch(gameNavigation.distanceFromBin(odometer.getX(), odometer.getY()));
          // transition back to navigation
          gameState = GameState.Navigation;
          break;
        case Demo:
          Navigation.travelTo(Resources.bin.x, Resources.bin.y, FORWARD_SPEED_NORMAL);
          gameNavigation.lightLocalize(Resources.bin);
          Navigation.turnTo(Resources.tnr.ur.x, ROTATE_SPEED_NORMAL);
          Sound.beep();
          Sound.beep();
          Sound.beep();
          ballisticLauncher.multipleLaunch(MAXIMAL_LAUNCH_DISTANCE);
          Sound.beep();
          break;
        default:
          break;

      }
    }
    LCD.clear();
    LCD.drawString("DONE", 1, 1);
  }



}


