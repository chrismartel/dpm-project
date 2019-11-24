package test;

import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.game.GameResources;
import ca.mcgill.ecse211.project.game.GameState;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.Localization.UltrasonicLocalizer;

public class LocalizationTest {
  public static void main(String args[]) {
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    Thread odometerThread = new Thread(GameResources.odometer);
    Thread usPollerTread = new Thread(GameResources.ultrasonicPoller);
    odometerThread.start();
    usPollerTread.start();
    new Thread(new Display()).start();
    GameResources.setGameState(GameState.UltrasonicLocalization);
    GameResources.CORNER_NUMBER = 0;
    ultrasonicLocalizer.fallingEdge(GameResources.ROTATE_SPEED_FAST);
    LightLocalizer.initialLightLocalize(new Point(1,1), GameResources.CORNER_NUMBER);
  }
  
}
