package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Sound;

public class GameController {

  private GameState state;
  
  private GameNavigation gameNavigation;
  private LightLocalizer lightLocalizer;
  private UltrasonicLocalizer ultrasonicLocalizer; 
  private BallisticLauncher ballisticLauncher;
  
  public static GameController gameController;
  
  
  
  public GameController() {
    this.state = GameState.Initialization;
    this.gameNavigation = new GameNavigation();
    this.lightLocalizer = new LightLocalizer();
    this.ultrasonicLocalizer = new UltrasonicLocalizer();
    this.ballisticLauncher = new BallisticLauncher();
    System.out.println("new game controller");
  }
  
  
  public static void main(String[] args) {
    Sound.beep();
    //gameController = new GameController();
    
    
  }
}
