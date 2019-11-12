package ca.mcgill.ecse211.project.test;

import static ca.mcgill.ecse211.project.game.GameResources.odometer;
import ca.mcgill.ecse211.project.game.GameNavigation;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.game.GameState;
import ca.mcgill.ecse211.project.game.Navigation;
import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.game.GameResources.*;

public class LightLocalization {
public static void main(String[] args) {
  gameState = GameState.LightLocalization;
  Thread odometerThread = new Thread(odometer);
  GameNavigation gameNavigation = new GameNavigation();
  odometerThread.start();
  new Thread(new Display()).start();
  
  while(true) {
    gameNavigation.twoLineDetection();
    Navigation.backUp(OFFSET_FROM_WHEELBASE);
    Navigation.turn(90, 100);
    odometer.setXYT(odometer.getX(), 30.48, odometer.getTheta());
    Button.waitForAnyPress();
    gameNavigation.twoLineDetection();
    Navigation.backUp(OFFSET_FROM_WHEELBASE);
    Navigation.turn(-90, 100);
    odometer.setXYT(30.48, 30.48, 0);
    Button.waitForAnyPress();


  
  }
}
}
