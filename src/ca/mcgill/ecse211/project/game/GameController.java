package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;

public class GameController {

  public enum NAVIGATION_TYPE {
    TUNNEL1_ENTRANCE, TUNNEL2_ENTRANCE, LAUNCH_POINT, END_POINT
  }
  public static void main(String[] args) {
    GameNavigation gameNavigation = new GameNavigation();;
    LightLocalizer lightLocalizer = new LightLocalizer();;
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();
    Thread odometerThread = new Thread(odometer);
    Thread usPollerTread = new Thread(ultrasonicPoller);
    odometerThread.start();
    usPollerTread.start();
    gameState = GameState.Initialization;
    // Execute until the game reaches the Done state
    while(gameState != GameState.Done) {
      if(gameState == GameState.Initialization) {
        // TODO: get info from wifi class and generate map
      }
      else if(gameState == GameState.UltrasonicLocalization) {
        // TODO: us localization using falling edge
      }
      else if(gameState == GameState.LightLocalization) {
        // TODO: light localization using 2 sensors at the back
        navigationType = NAVIGATION_TYPE.TUNNEL1_ENTRANCE;
      }
      else if(gameState == GameState.Navigation) {
        // TODO: navigate to point
      }
      else if(gameState == GameState.Tunnel) {
        // TODO: navigate through tunnel
        if(navigationType == NAVIGATION_TYPE.TUNNEL1_ENTRANCE) {
          navigationType = NAVIGATION_TYPE.LAUNCH_POINT;
        }
        else if(navigationType == NAVIGATION_TYPE.TUNNEL2_ENTRANCE) {
          navigationType = NAVIGATION_TYPE.END_POINT;
        }
      }
      else if(gameState == GameState.Avoidance) {
        // TODO: object avoidance process
        
      }
      else if(gameState == GameState.Launch) {
        // TODO: launch and reload
        navigationType = NAVIGATION_TYPE.TUNNEL2_ENTRANCE;
      }
      
    }
    



  }
}
