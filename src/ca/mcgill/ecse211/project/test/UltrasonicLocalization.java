package ca.mcgill.ecse211.project.test;

import static ca.mcgill.ecse211.project.game.GameResources.odometer;
import static ca.mcgill.ecse211.project.game.GameResources.ultrasonicPoller;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.game.GameState;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.game.GameResources.*;

public class UltrasonicLocalization {
public static void main(String[] args) {
  gameState = GameState.UltrasonicLocalization;
  Thread odometerThread = new Thread(odometer);
  Thread usPollerTread = new Thread(ultrasonicPoller);
  odometerThread.start();
  usPollerTread.start();
  new Thread(new Display()).start();
  UltrasonicLocalizer UL = new UltrasonicLocalizer();
  while(true) {
    Button.waitForAnyPress();
  UL.fallingEdge();
  
  }
}
}
