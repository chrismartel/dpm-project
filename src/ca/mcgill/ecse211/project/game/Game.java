package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.project.sensor.UltrasonicPoller;
import lejos.hardware.Sound;

public class Game {
  private GameState state;
  
  private GameNavigation gameNavigation;
  private LightLocalizer lightLocalizer;
  private UltrasonicLocalizer ultrasonicLocalizer; 
  private BallisticLauncher ballisticLauncher;
  private UltrasonicPoller ultrasonicPoller;
  
  public Game() {
    this.state = GameState.Initialization;
    this.gameNavigation = new GameNavigation();
    this.lightLocalizer = new LightLocalizer();
    this.ultrasonicLocalizer = new UltrasonicLocalizer();
    this.ballisticLauncher = new BallisticLauncher();
    Sound.beep();
  }
  
  public BallisticLauncher getBallisticLauncher() {
    return ballisticLauncher;
  }
  public GameNavigation getGameNavigation() {
    return gameNavigation;
  }
  public LightLocalizer getLightLocalizer() {
    return lightLocalizer;
  }
  public GameState getState() {
    return state;
  }
  public UltrasonicLocalizer getUltrasonicLocalizer() {
    return ultrasonicLocalizer;
  }

  
}
