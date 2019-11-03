package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;

public class GameController {


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
    while(gameState != GameState.Done) {
      if(gameState == GameState.Initialization) {
        
      }
      else if(gameState == GameState.UltrasonicLocalization) {
        
      }
      else if(gameState == GameState.LightLocalization) {
        
      }
      else if(gameState == GameState.Navigation) {
        
      }
      else if(gameState == GameState.Tunnel) {
        
        
      }
      else if(gameState == GameState.Avoidance) {
        
        
      }
      else if(gameState == GameState.Launch) {
        
        
      }
      
    }
    



  }
}
