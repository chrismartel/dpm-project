package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.Localization.UltrasonicLocalizer;
import lejos.hardware.Sound;

public class GameController {

  public static void main(String[] args) throws InterruptedException {
    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObstacleAvoider obstacleAvoider = new ObstacleAvoider();

    // tunnel traversal counter
    int tunnel = 0;
    // boolean indicating if the square navigation should travel on the x axis first or the y axis first
    boolean xFirst = true;

    // set initial state to initialization
    GameResources.setGameState(GameState.Initialization);

    // Execute until the game reaches the Done state
    while (GameResources.getGameState() != GameState.Done) {
      switch (GameResources.getGameState()) {
        // Test case used for testing and debugging puproses
        case Test:
          break;
        case Initialization:
          // start threads
          Thread odometerThread = new Thread(GameResources.odometer);
          Thread usPollerTread = new Thread(GameResources.ultrasonicPoller);
          // new Thread(new Display()).start();
          odometerThread.start();
          usPollerTread.start();

          // Initialize map
          gameNavigation.setGameParameters();
          // set first checkpoint
          GameResources.setNavigationDestination(NAVIGATION_DESTINATION.TUNNEL_ENTRANCE);

          // Warm up motors
          Navigation.travel(2, GameResources.FORWARD_SPEED_NORMAL);
          Navigation.backUp(2, GameResources.FORWARD_SPEED_NORMAL);
          GameResources.leftBallisticMotor.rotate(-5, true);
          GameResources.rightBallisticMotor.rotate(-5, false);
          GameResources.leftBallisticMotor.rotate(+5, true);
          GameResources.rightBallisticMotor.rotate(+5, false);

          // transit to ultrasonic localization state
          GameResources.setGameState(GameState.UltrasonicLocalization);
          break;


        case UltrasonicLocalization:
          // ultrasonic localization using falling edge routine
          ultrasonicLocalizer.fallingEdge(GameResources.ROTATE_SPEED_NORMAL);
          // transition to light localization state
          GameResources.setGameState(GameState.LightLocalization);
          break;

        case LightLocalization:
          // initial localization depending on the starting point and starting corner
          LightLocalizer.initialLightLocalize(GameResources.STARTING_POINT, GameResources.CORNER_NUMBER);

          // beep 3 times to indicate light localization is done
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();

          // navigate a small distance forward to avoid detecting the light localization line
          Navigation.travel(GameResources.INITIAL_LIGHT_LOC_ADJUSTMENT_DISTANCE, GameResources.FORWARD_SPEED_NORMAL);

          // transit to navigation state
          GameResources.gameState = GameState.Navigation;
          break;


        case Navigation:
          // navigation is considered uncompleted initially
          GameResources.navigationCompleted = false;

          // check the current navigation destination
          switch (GameResources.navigationDestination) {
            // navigation to tunnel entrance
            case TUNNEL_ENTRANCE:
              // navigate to tunnel entrance and turn to traversal orientation
              gameNavigation.navigateToTunnelEntrance(xFirst);
              if (GameResources.isNavigationCompleted()) {
                // transition to tunnel state
                GameResources.setGameState(GameState.Tunnel);
                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.LAUNCH_POINT);
              }
              break;

            // navigation to tunnel exit
            case TUNNEL_EXIT:
              // travel to the second tunnel entrance
              gameNavigation.navigateToTunnelExit(xFirst);
              if (GameResources.isNavigationCompleted()) {
                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.END_POINT);
                // transition to tunnel state
                GameResources.setGameState(GameState.Tunnel);
              }
              break;

            case LAUNCH_POINT:
              // navigate to the closest launch point from the robot position using the current navigation path
              // indicated by xFirst boolean
              gameNavigation.navigateToLaunchPoint(xFirst);
              if (GameResources.isNavigationCompleted()) {
                // localize on the launch point before launch
                LightLocalizer.lightLocalize(gameNavigation.getLaunchPoint(), false);
                // turn towards the bin
                gameNavigation.turnToTarget();
                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.TUNNEL_EXIT);
                // Transit to launch state
                GameResources.setGameState(GameState.Launch);
              }

              break;
            case END_POINT:
              // navigate back to starting point
              GameNavigation.squareNavigation(GameResources.STARTING_POINT.x, GameResources.STARTING_POINT.y, xFirst,
                  true);
              if (GameResources.isNavigationCompleted()) {
                // Transit to done state
                GameResources.setGameState(GameState.Done);
              }
              break;
            default:
              break;
          }
          break;


        case Tunnel:
          // first tunnel traversal
          if (tunnel == 0) {
            // navigate through tunnel
            gameNavigation.navigateThroughTunnel();
            // increment the tunnel traversal counter
            tunnel++;
            // compute the closest launch point
            gameNavigation.calculateClosestLaunchPoint();
          }
          // second tunnel traversal
          else if (tunnel == 1) {
            gameNavigation.navigateThroughTunnel();
          }
          // transition back to navigation
          GameResources.setGameState(GameState.Navigation);
          break;


        case Avoidance:

          // calculate the distance to backUp after an obstacle detection
          double backupDistance = gameNavigation.calculateBackwardDistance();
          Navigation.backUp(backupDistance, GameResources.FORWARD_SPEED_NORMAL);

          // OBSTACLE AVOIDANCE PATH CHANGE STRATEGY

          // Robot is navigating towards launch point
          if (GameResources.navigationDestination == NAVIGATION_DESTINATION.LAUNCH_POINT) {
            // create restricted points and determines if the launch point has to be changed
            boolean newLaunchPoint = gameNavigation.createRestrictedPoints();
            // new possible launch points considering the new restricted points
            gameNavigation.generateLaunchPoints();

            // Launch point was not changed
            if (!newLaunchPoint) {

              // Shift the robot depending on its position on the island
              obstacleAvoider.shiftRobot();

              // Change the square navigation path
              if (xFirst) {
                xFirst = false;
              } else {
                xFirst = true;
              }
            }
            // Launch point was changed
            else {
              // calculate a new launch point
              gameNavigation.calculateClosestLaunchPoint();
              // Shift the robot depending on its position on the island
              obstacleAvoider.shiftRobot();
              // Change the square navigation path
              if (xFirst) {
                xFirst = false;
              } else {
                xFirst = true;
              }

            }
          }

          // Robot is navigating towards tunnel exit
          else {
            // Shift the robot depending on its position on the island
            obstacleAvoider.shiftRobot();
            // Change the square navigation path
            if (xFirst) {
              xFirst = false;
            } else {
              xFirst = true;
            }
          }

          // transit to navigation after avoidance procedure
          GameResources.setGameState(GameState.Navigation);
          break;

        case Launch:
          // Beep 3 times to indicate the robot has reached its launch point
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          // Perform the launches
          ballisticLauncher
              .multipleLaunch(gameNavigation.distanceFromBin(GameResources.odometer.getX() / GameResources.TILE_SIZE,
                  GameResources.odometer.getY() / GameResources.TILE_SIZE));
          // transition back to navigation
          GameResources.setGameState(GameState.Navigation);
          break;
        default:
          break;
      }
    }
    // When state machine terminates, beep 5 times to indicate the robot has came back to its initial point
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
  }



}


