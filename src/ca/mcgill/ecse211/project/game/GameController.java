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
    Point closestPoint;
    int tunnel = 0;

    // set initial state
    GameResources.setGameState(GameState.Initialization);

    // Execute until the game reaches the Done state
    while (GameResources.getGameState() != GameState.Done) {
      switch (GameResources.getGameState()) {
        case Test:
          GameResources.setCurrentRegion(REGION.ISLAND);
          GameResources.setGameState(GameState.Navigation);
          GameResources.odometer.setXYT(GameResources.TILE_SIZE, GameResources.TILE_SIZE, 0);
          System.out.println(GameResources.getGameState());
          gameNavigation.squareNavigation(1, 4);
          
          
          // LAUNCHING COEFFICIENT TEST
          // test speeds from 150 to 650 and record distances for each
/*
          for(int i = 150; i<=650; i+=25) {
            System.out.println("speed: "+i);
            ballisticLauncher.launchTest(i);
            ballisticLauncher.reload();
            Button.waitForAnyPress();
          }
          gameState = GameState.Done;
*/


        case Initialization:
          // start threads
          Thread odometerThread = new Thread(GameResources.odometer);
          Thread usPollerTread = new Thread(GameResources.ultrasonicPoller);
          odometerThread.start();
          usPollerTread.start();

          // Initialize map
          gameNavigation.setGameParameters();

          // set first checkpoint
          GameResources.setNavigationDestination(NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE);

          // Warm up motors
          Navigation.travel(2, GameResources.FORWARD_SPEED_NORMAL);
          Navigation.backUp(2, GameResources.FORWARD_SPEED_NORMAL);
          GameResources.leftBallisticMotor.rotate(-5, true);
          GameResources.rightBallisticMotor.rotate(-5, false);
          GameResources.leftBallisticMotor.rotate(+5, true);
          GameResources.rightBallisticMotor.rotate(+5, false);

          // transit to ultrasonic localization state
          Button.waitForAnyPress();
        GameResources.setGameState(GameState.UltrasonicLocalization);
   //        GameResources.setGameState(GameState.Test);


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
          // transit to navigation state
          GameResources.gameState = GameState.Navigation;
          break;


        case Navigation:
          // navigation is considered uncompleted initially
          GameResources.navigationCompleted = false;
          
          // check the current navigation destination
          switch (GameResources.navigationDestination) {
            
            // navigation to first tunnel entrance
            case TUNNEL1_ENTRANCE:
              // navigate to tunnel entrance and turn to traversal orientation
              gameNavigation.navigateToTunnelEntrance();
              
              if (GameResources.isNavigationCompleted()) {
                // transition to tunnel state
                GameResources.setGameState(GameState.Tunnel);
                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.LAUNCH_POINT);

              }

              break;
            case TUNNEL2_ENTRANCE:
              // travel to the second tunnel entrance
              System.out.println("navigation to tunnel entrance 2");
              gameNavigation.navigateToTunnelExit();
              
              if (GameResources.isNavigationCompleted()) {    
                /*
                // LIGHT LOCALIZATION
                closestPoint = gameNavigation.closestPoint();
                Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
                LightLocalizer.lightLocalize(closestPoint);
                gameState = GameState.Navigation;
                // go to tunnel entrance
                gameNavigation.navigateToTunnel();
                */
                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.END_POINT);
                // transition to tunnel state
                GameResources.setGameState(GameState.Tunnel);
              }
              else if(GameResources.isEnableAvoidance()) {
                GameResources.setGameState(GameState.Avoidance);
              }
              break;
            case LAUNCH_POINT:
              // compute the closest launch point
              gameNavigation.calculateClosestLaunchPoint();
              // navigate to the closest launch point
              gameNavigation.navigateToLaunchPoint();

              if (GameResources.isNavigationCompleted()) {
/*
                // LIGHT LOCALIZATION
                LightLocalizer.lightLocalize(gameNavigation.getLaunchPoint());
                System.out.println("CURRENT POINT: "+odometer.getX()/TILE_SIZE+", "+ odometer.getY()/TILE_SIZE);

                // navigate to launch point after localizing
                gameNavigation.navigateToLaunchPoint();*/
                gameNavigation.turnToTarget();

                // update new checkpoint
                GameResources.setNavigationDestination(NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE);

                // Transit to launch state
                GameResources.setGameState(GameState.Launch);
              }
              else if(GameResources.isEnableAvoidance()) {
                GameResources.setGameState(GameState.Avoidance);
              }
              break;
            case END_POINT:
              System.out.println("x: "+GameResources.odometer.getX()/GameResources.TILE_SIZE+", y:"+ GameResources.odometer.getY()/GameResources.TILE_SIZE+", theta: "+ GameResources.odometer.getTheta());
              System.out.println("STARTING POINT: "+GameResources.STARTING_POINT.x+", "+GameResources.STARTING_POINT.y);
              gameNavigation.squareNavigation(GameResources.STARTING_POINT.x, GameResources.STARTING_POINT.y);
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
          
          System.out.println("current region before tunnel traversal: "+ GameResources.currentRegion);

          // correct odometer according to tunnel entrance data
          // first tunnel traversal
          if(tunnel==0) {
            GameResources.odometer.setXYT(gameNavigation.getTunnelEntrance().x * GameResources.TILE_SIZE, gameNavigation.getTunnelEntrance().y * GameResources.TILE_SIZE, gameNavigation.getTunnelEntranceTraversalOrientation());
          }
          // second tunnel traversal
          else if(tunnel==1){
            System.out.println("ODOMETER SET ACCORDING TO TUNNEL EXIT");
            GameResources.odometer.setXYT(gameNavigation.getTunnelExit().x * GameResources.TILE_SIZE, gameNavigation.getTunnelExit().y * GameResources.TILE_SIZE, gameNavigation.getTunnelExitTraversalOrientation());
          }
          // navigate through tunnel
          gameNavigation.navigateThroughTunnel();
          tunnel++;
          // update new zone parameters
          gameNavigation.updateParameters();
          
          System.out.println("TUNNEL ENTRANCE: "+gameNavigation.getTunnelEntrance().x+", "+ gameNavigation.getTunnelEntrance().y);

          
          // LIGHT LOCALIZATION
          closestPoint = gameNavigation.closestPoint();
          Navigation.travelTo(closestPoint.x, closestPoint.y, GameResources.FORWARD_SPEED_NORMAL);
          LightLocalizer.lightLocalize(closestPoint);

          // transition back to navigation
          GameResources.setGameState(GameState.Navigation);
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          
//          obstacleAvoider.wallFollower(FORWARD_SPEED_NORMAL);
          Sound.beep();
          Button.waitForAnyPress();
          System.out.println("OBJECT DETECTED");
          // regenerate the launch points
          gameNavigation.generateLaunchPoints();
          obstacleAvoider.wallFollower(GameResources.FORWARD_SPEED_NORMAL);
 /*         // LIGHT LOCALIZATION
          closestPoint = gameNavigation.closestPoint();
          Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
          LightLocalizer.lightLocalize(closestPoint);*/
          GameResources.setGameState(GameState.Navigation);

          break;


        case Launch:
          // perform the launches
          ballisticLauncher.multipleLaunch(gameNavigation.distanceFromBin(GameResources.odometer.getX(), GameResources.odometer.getY()));
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


