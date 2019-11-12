package ca.mcgill.ecse211.project.test;

import static ca.mcgill.ecse211.project.game.GameResources.odometer;
import static ca.mcgill.ecse211.project.game.GameResources.ultrasonicPoller;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.game.GameNavigation;
import ca.mcgill.ecse211.project.game.GameState;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.game.GameResources.*;

public class USandLL {
public static void main(String[] args) {
  gameState = GameState.UltrasonicLocalization;
  Thread odometerThread = new Thread(odometer);
  Thread usPollerTread = new Thread(ultrasonicPoller);
  GameNavigation gameNavigation = new GameNavigation();
  odometerThread.start();
  usPollerTread.start();
  new Thread(new Display()).start();
  UltrasonicLocalizer UL = new UltrasonicLocalizer();
    Button.waitForAnyPress();
    UL.fallingEdge();
    Button.waitForAnyPress();
   gameNavigation.lightCorrect(1,1,100);
    

}
}
