package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.Localization.UltrasonicLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class GameController {

  public static void main(String[] args) throws InterruptedException {
    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    ObstacleAvoider obstacleAvoider = new ObstacleAvoider();

    // closest point used for light loc
    Point closestPoint;
    // tunnel traversal counter
    int tunnel = 0;
    // boolean indicating if the square navigation should travel on the x axis first or the y axis first
    boolean xFirst = true;

    // set initial state
    GameResources.setGameState(GameState.Initialization);

    // Execute until the game reaches the Done state
    while (GameResources.getGameState() != GameState.Done) {
      switch (GameResources.getGameState()) {
        case Test:

          GameResources.setGameState(GameState.Navigation);
          GameResources.setNavigationDestination(NAVIGATION_DESTINATION.LAUNCH_POINT);
          GameResources.setCurrentRegion(REGION.ISLAND);
          gameNavigation.setLimits();
          GameResources.odometer.setXYT(7 * GameResources.TILE_SIZE, 5 * GameResources.TILE_SIZE, 270);
          xFirst = true;
          GameNavigation.squareNavigation(5, 5, xFirst, false);
          break;


        case Initialization:
          // start threads
          Thread odometerThread = new Thread(GameResources.odometer);
          Thread usPollerTread = new Thread(GameResources.ultrasonicPoller);
          new Thread(new Display()).start();
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
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          
          Navigation.travel(GameResources.INITIAL_LIGHT_LOC_ADJUSTMENT_DISTANCE, GameResources.FORWARD_SPEED_NORMAL);
          // transit to navigation state
          GameResources.gameState = GameState.Navigation;
          GameResources.setLocalized(true);
          try {
            Thread.sleep(500);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
          break;


        case Navigation:
          System.out.println(gameNavigation.getTunnelEntrance());
          System.out.println(" ");
          System.out.println(" ");
          System.out.println(" ");
          System.out.println(" ");
          System.out.println(" ");
          System.out.println(" ");
          System.out.println(" ");
          System.out.println(" ");
          GameResources.LCD.clear();
          // navigation is considered uncompleted initially
          GameResources.navigationCompleted = false;
          // check the current navigation destination
          switch (GameResources.navigationDestination) {
            // navigation to first tunnel entrance
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
            case TUNNEL_EXIT:
              // travel to the second tunnel entrance
              obstacleAvoider.setGoalPoint(gameNavigation.getTunnelExit());
              gameNavigation.navigateToTunnelExit(xFirst);
              if (GameResources.isNavigationCompleted()) {
                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.END_POINT);
                // transition to tunnel state
                GameResources.setGameState(GameState.Tunnel);
              }
              break;
            case LAUNCH_POINT:
              // navigate to the closest launch point on the current navigation path indicated by xFirst boolean
              gameNavigation.navigateToLaunchPoint(xFirst);
              if (GameResources.isNavigationCompleted()) {
                // turn towards the bin
                LightLocalizer.lightLocalize(gameNavigation.getLaunchPoint(), false);
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
                GameResources.setGameState(GameState.Done);
              }
              break;
            default:
              break;
          }
          break;


        case Tunnel:
          // adjust heading
//          LightLocalizer.twoLineDetection();
          // position center of rotation at tunnel entrance

          // correct odometer according to tunnel entrance data
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
            System.out.println(gameNavigation.getTunnelExit());
            gameNavigation.navigateThroughTunnel();
          }
          // transition back to navigation
          GameResources.setGameState(GameState.Navigation);
          break;


        case Avoidance:
          System.out.println("OBJECT DETECTED");
          // create restricted points and determines if the launch point has to be changed
          boolean newLaunchPoint = gameNavigation.createRestrictedPoints();
          // possible launch points updated
          gameNavigation.generateLaunchPoints();
          double backupDistance = gameNavigation.calculateBackwardDistance();
          Navigation.backUp(backupDistance, GameResources.FORWARD_SPEED_FAST);

          // OBSTACLE AVOIDANCE STRATEGY
            // launch point is still the same
          if (GameResources.navigationDestination == NAVIGATION_DESTINATION.LAUNCH_POINT) {
            if (!newLaunchPoint) {
              System.out.println("SAME LAUNCH POINT");
              // set the objective point of the wall follower
              obstacleAvoider.setGoalPoint(gameNavigation.getLaunchPoint());

                obstacleAvoider.shiftRobot();
                if (xFirst) {
                  xFirst = false;
                } else {
                  xFirst = true;
                }
            } else {
              System.out.println("NEW LAUNCH POINT NEEDED");
              // calculate a new launch point
              gameNavigation.calculateClosestLaunchPoint();
//              obstacleAvoider.setGoalPoint(gameNavigation.getLaunchPoint());
                obstacleAvoider.shiftRobot();
                if (xFirst) {
                  xFirst = false;
                } else {
                  xFirst = true;
                }

            }
          }else {
            obstacleAvoider.shiftRobot();
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
          // perform the launches
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          Thread.sleep(50);
          Sound.beep();
          ballisticLauncher
              .multipleLaunch(gameNavigation.distanceFromBin(GameResources.odometer.getX() / GameResources.TILE_SIZE,
                  GameResources.odometer.getY() / GameResources.TILE_SIZE));
          // transition back to navigation
          GameResources.setGameState(GameState.Navigation);
//          LightLocalizer.lightLocalize(gameNavigation.getLaunchPoint(), false);
          xFirst = false;
          break;

        default:
          break;

      }
    }
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    Thread.sleep(50);
    Sound.beep();
    
    GameResources.LCD.clear();
    GameResources.LCD.drawString("DONE", 1, 1);
  }




}


