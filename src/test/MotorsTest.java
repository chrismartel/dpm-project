package test;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.game.GameResources;
import ca.mcgill.ecse211.project.game.Navigation;
import lejos.hardware.Button;

public class MotorsTest {
  public static void main(String[] args) {
    new Thread(odometer).start();
    new Thread(new Display()).start();
    while(true) {
    Navigation.turn(-90, 150);
    Button.waitForAnyPress();
    }
    
  }

}
