package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import lejos.hardware.Button;

public class GameController {


  public static void main(String[] args) {
    Game game = new Game();
    Thread odometerThread = new Thread(odometer);
    Thread usPollerTread = new Thread(ultrasonicPoller);
    odometerThread.start();
    usPollerTread.start();
    int button1 = Button.waitForAnyPress();
    if(button1 == Button.ID_ENTER ) {
      Navigation.travel(5);

    }

    

    
    
  }
}
