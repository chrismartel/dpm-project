package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Resources;
import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.Localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.project.game.Navigation.Turn;
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
    gameState = GameState.Initialization;

    // Execute until the game reaches the Done state
    while (gameState != GameState.Done) {
      switch (gameState) {
        case Test:
          currentRegion = REGION.ISLAND;
          gameState = GameState.Navigation;
          odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
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
          Thread odometerThread = new Thread(odometer);
          Thread usPollerTread = new Thread(ultrasonicPoller);
          odometerThread.start();
          usPollerTread.start();

          // Initialize map
          gameNavigation.setGameParameters();

          // set first checkpoint
          navigationDestination = NAVIGATION_DESTINATION.TUNNEL1_ENTRANCE;

          // Warm up motors
          Navigation.travel(2, FORWARD_SPEED_NORMAL);
          Navigation.backUp(2, FORWARD_SPEED_NORMAL);
          leftBallisticMotor.rotate(-5, true);
          rightBallisticMotor.rotate(-5, false);
          leftBallisticMotor.rotate(+5, true);
          rightBallisticMotor.rotate(+5, false);

          // transit to ultrasonic localization state
          Button.waitForAnyPress();
          gameState = GameState.Test;

          break;


        case UltrasonicLocalization:
          // ultrasonic localization using falling edge routine
          ultrasonicLocalizer.fallingEdge(ROTATE_SPEED_NORMAL);
          // transition to light localization state
          gameState = GameState.LightLocalization;
          break;


        case LightLocalization:
          // initial localization depending on the starting point and starting corner
          LightLocalizer.initialLightLocalize(STARTING_POINT, CORNER_NUMBER);
          // transit to navigation state
          gameState = GameState.Navigation;
          break;


        case Navigation:
          // navigation is considered uncompleted initially
          navigationCompleted = false;
          
          // check the current navigation destination
          switch (navigationDestination) {
            
            // navigation to first tunnel entrance
            case TUNNEL1_ENTRANCE:
              // navigate to tunnel entrance and turn to traversal orientation
              gameNavigation.navigateToTunnelEntrance();
              
              if (navigationCompleted == true) {
                // transition to tunnel state
                gameState = GameState.Tunnel;
                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.LAUNCH_POINT;
              }
              break;
            case TUNNEL2_ENTRANCE:
              // travel to the second tunnel entrance
              System.out.println("navigation to tunnel entrance 2");
              gameNavigation.navigateToTunnelExit();
              
              if (navigationCompleted == true) {    
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
                navigationDestination = NAVIGATION_DESTINATION.END_POINT;
                // transition to tunnel state
                gameState = GameState.Tunnel;
              }
              break;
            case LAUNCH_POINT:
              // compute the closest launch point
              gameNavigation.calculateClosestLaunchPoint();
              // navigate to the closest launch point
              gameNavigation.navigateToLaunchPoint();

              if (navigationCompleted == true) {
/*
                // LIGHT LOCALIZATION
                LightLocalizer.lightLocalize(gameNavigation.getLaunchPoint());
                System.out.println("CURRENT POINT: "+odometer.getX()/TILE_SIZE+", "+ odometer.getY()/TILE_SIZE);

                // navigate to launch point after localizing
                gameNavigation.navigateToLaunchPoint();*/
                gameNavigation.turnToTarget();

                // update new checkpoint
                navigationDestination = NAVIGATION_DESTINATION.TUNNEL2_ENTRANCE;
                // Transit to launch state
                gameState = GameState.Launch;
              }
              break;
            case END_POINT:
              System.out.println("x: "+odometer.getX()/TILE_SIZE+", y:"+ odometer.getY()/TILE_SIZE+", theta: "+ odometer.getTheta());
              System.out.println("STARTING POINT: "+STARTING_POINT.x+", "+STARTING_POINT.y);
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
          LightLocalizer.twoLineDetection();
          // position center of rotation at tunnel entrance
          Navigation.backUp(OFFSET_FROM_WHEELBASE, FORWARD_SPEED_NORMAL);
          
          System.out.println("current region before tunnel traversal: "+ currentRegion);

          // correct odometer according to tunnel entrance data
          if(tunnel==0) {
            odometer.setXYT(gameNavigation.getTunnelEntrance().x * TILE_SIZE, gameNavigation.getTunnelEntrance().y * TILE_SIZE, gameNavigation.getTunnelEntranceTraversalOrientation());
          }
          else if(tunnel==1){
            System.out.println("ODOMETER SET ACCORDING TO TUNNEL EXIT");
            odometer.setXYT(gameNavigation.getTunnelExit().x * TILE_SIZE, gameNavigation.getTunnelExit().y * TILE_SIZE, gameNavigation.getTunnelExitTraversalOrientation());
          }
          // navigate through tunnel
          gameNavigation.navigateThroughTunnel();
          tunnel++;
          // update new zone parameters
          gameNavigation.updateParameters();
          
          System.out.println("TUNNEL ENTRANCE: "+gameNavigation.getTunnelEntrance().x+", "+ gameNavigation.getTunnelEntrance().y);

          
          // LIGHT LOCALIZATION
          closestPoint = gameNavigation.closestPoint();
          Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
          LightLocalizer.lightLocalize(closestPoint);

          // transition back to navigation
          gameState = GameState.Navigation;
          break;


        case Avoidance:
          // object avoidance procedure using wall follower with P-Controller
          
//          obstacleAvoider.wallFollower(FORWARD_SPEED_NORMAL);
          Button.waitForAnyPress();
          System.out.println("OBJECT DETECTED");
          // regenerate the launch points
          gameNavigation.generateLaunchPoints();
/*
          // LIGHT LOCALIZATION
          closestPoint = gameNavigation.closestPoint();
          Navigation.travelTo(closestPoint.x, closestPoint.y, FORWARD_SPEED_NORMAL);
          LightLocalizer.lightLocalize(closestPoint);*/
          gameState = GameState.Navigation;

          break;


        case Launch:
          // perform the launches
          ballisticLauncher.multipleLaunch(gameNavigation.distanceFromBin(odometer.getX(), odometer.getY()));
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


