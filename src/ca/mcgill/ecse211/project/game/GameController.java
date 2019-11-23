package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
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
    // closest point used for light loc
    Point closestPoint;
    // tunnel traversal counter
    int tunnel = 0;
    // time management tools
    double startTime = 0;
    double currentTime = 0;
    // boolean indicating if the square navigation should travel on the x axis first or the y axis first
    boolean xFirst = true;

    // set initial state
    GameResources.setGameState(GameState.Initialization);

    // Execute until the game reaches the Done state
    while (GameResources.getGameState() != GameState.Done) {
      switch (GameResources.getGameState()) {
        case Test:
          
          Button.waitForAnyPress();
          GameResources.setGameState(GameState.Navigation);
          GameResources.setCurrentRegion(REGION.ISLAND);
          gameNavigation.setLimits();
          GameResources.odometer.setXYT(3 * GameResources.TILE_SIZE, 3 * GameResources.TILE_SIZE, 0);
          xFirst = true;
          GameNavigation.squareNavigation(3, 7, xFirst, true);
          gameNavigation.setLaunchPoint(new Point(3, 5));


          break;


        case Initialization:
          // start threads
          Thread odometerThread = new Thread(GameResources.odometer);
          Thread usPollerTread = new Thread(GameResources.ultrasonicPoller);
          startTime = System.currentTimeMillis();
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
          Button.waitForAnyPress();

          ultrasonicLocalizer.fallingEdge(GameResources.ROTATE_SPEED_FAST);
          // transition to light localization state
          GameResources.setGameState(GameState.LightLocalization);
          break;


        case LightLocalization:
          // initial localization depending on the starting point and starting corner
          LightLocalizer.initialLightLocalize(GameResources.STARTING_POINT, GameResources.CORNER_NUMBER);
          // transit to navigation state
          GameResources.gameState = GameState.Navigation;
          GameResources.setLocalized(true);
          break;


        case Navigation:
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
              System.out.println(
                  "launchx: " + gameNavigation.getLaunchPoint().x + ", launchy: " + gameNavigation.getLaunchPoint().y);

              if (GameResources.isNavigationCompleted()) {

                // localize before launch if the robot is not localized anymore
                if (!GameResources.isLocalized()) {
                  // LIGHT LOCALIZATION
                  LightLocalizer.lightLocalize(gameNavigation.getLaunchPoint(), true);
                  GameResources.setLocalized(true);
                }
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
                GameResources.setGameState(GameState.Done);
              }
              break;
            default:
              break;
          }
          break;


        case Tunnel:

          // adjust heading
          LightLocalizer.twoLineDetection();
          // position center of rotation at tunnel entrance
          Navigation.backUp(GameResources.OFFSET_FROM_WHEELBASE, GameResources.FORWARD_SPEED_NORMAL);

          // correct odometer according to tunnel entrance data
          // first tunnel traversal
          if (tunnel == 0) {
            GameResources.odometer.setXYT(gameNavigation.getTunnelEntrance().x * GameResources.TILE_SIZE,
                gameNavigation.getTunnelEntrance().y * GameResources.TILE_SIZE,
                gameNavigation.getTunnelEntranceTraversalOrientation());
            // navigate through tunnel
            gameNavigation.navigateThroughTunnel();
            // increment the tunnel traversal counter
            tunnel++;

            // LIGHT LOCALIZATION
            closestPoint = gameNavigation.closestPoint();
            Navigation.travelTo(closestPoint.x, closestPoint.y, GameResources.FORWARD_SPEED_NORMAL);
            LightLocalizer.lightLocalize(closestPoint, false);
            GameResources.setLocalized(true);

            // compute the closest launch point
            gameNavigation.generateLaunchPoints();
            gameNavigation.calculateClosestLaunchPoint();
          }
          // second tunnel traversal
          else if (tunnel == 1) {
            GameResources.odometer.setXYT(gameNavigation.getTunnelExit().x * GameResources.TILE_SIZE,
                gameNavigation.getTunnelExit().y * GameResources.TILE_SIZE,
                gameNavigation.getTunnelExitTraversalOrientation());
            // navigate through tunnel
            gameNavigation.navigateThroughTunnel();

            // if more than 4 minutes and a half have passed, don't localize at the end of the second tunnel
            currentTime = System.currentTimeMillis();
            if ((currentTime - startTime) <= 270000) {
              // LIGHT LOCALIZATION
              closestPoint = gameNavigation.closestPoint();

              Navigation.travelTo(closestPoint.x, closestPoint.y, GameResources.FORWARD_SPEED_NORMAL);
              LightLocalizer.lightLocalize(closestPoint, false);
              GameResources.setLocalized(true);
            }
          }
          // transition back to navigation
          GameResources.setGameState(GameState.Navigation);
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          System.out.println("OBJECT DETECTED");
          Navigation.backUp(GameResources.OBSTACLE_BACKUP, GameResources.FORWARD_SPEED_FAST);
          // create restricted points and check if launch point was changed
          boolean newLaunchPoint = gameNavigation.createRestrictedPoints();
          gameNavigation.generateLaunchPoints();

          // no new launch point, so we have to get around obstacle
          if (!newLaunchPoint) {
            System.out.println("SAME LAUNCH POINT");
            obstacleAvoider.shiftRobot();
            // change the path of square navigation
            if (xFirst) {
              xFirst = false;
            } else {
              xFirst = true;
            }
            for (Point point : GameResources.getRestrictedPoints()) {
              System.out.println("retsricted point: " + point.x + ", " + point.y);
            }
            //gameNavigation.calculateClosestLaunchPoint();

          }
          // new launch point so we may not have to get around obstacle
          else {
            System.out.println("NEW LAUNCH POINT NEEDED");
            // compute the closest launch point
            gameNavigation.calculateClosestLaunchPoint();
          }
          // if a new launch point is computed transit to navigation
          GameResources.setGameState(GameState.Navigation);
          break;


        case Launch:
          // perform the launches
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
    GameResources.LCD.clear();
    GameResources.LCD.drawString("DONE", 1, 1);
  }



}


