package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Localization.*;
import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.Resources.*;


/**
 * The main class.
 */
public class Main {

  // Odometer odometer;



  public static void main(String[] args) {

    Thread odometerThread = new Thread(odometer);
    Thread usPollerThread = new Thread(ultrasonicPoller);

    LCD.clear();
    LCD.drawString("PRESS ENTER ", 1, 2);
    LCD.drawString("TO START ", 1, 3);

    int buttonChoice1 = Button.waitForAnyPress();
    int buttonChoice2 = 0;
    if (buttonChoice1 == Button.ID_ENTER) {

      // Start the odometer and the distance polling threads
      usPollerThread.start();
      odometerThread.start();
      LCD.clear();
      LCD.drawString("LEFT: STATIONARY ", 1, 2);
      LCD.drawString("RIGHT: MOBILE", 1, 3);

      // Selection of the launch to perform
      buttonChoice2 = Button.waitForAnyPress();
      LCD.clear();
    }

    // stationary launch
    if (buttonChoice2 == Button.ID_LEFT) {
      LCD.clear();
      while (true) {
        LCD.drawString("PRESS ENTER ", 1, 2);
        LCD.drawString("TO LAUNCH ", 1, 3);
        int buttonChoice3 = Button.waitForAnyPress();

        if (buttonChoice3 == Button.ID_ENTER) {
          LCD.clear();
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
          }
          ballisticLauncher.launch(120);
          try {
            Thread.sleep(500);
          } catch (InterruptedException e) {
          }
          ballisticLauncher.reload();
        }
        else if (buttonChoice3==Button.ID_ESCAPE){
          System.exit(0);
        }
      }

    }

    // mobile launch
    else if (buttonChoice2 == Button.ID_RIGHT) {
      LCD.clear();
      new Thread(new Display()).start();
      double[] launchArea = new double[2];
      launchArea[0] = 5.5;
      launchArea[1] = 7.5;
      initialLocalize();
      double[] launchLocation = ballisticLauncher.launchLocation(launchArea[0], launchArea[1]);
      navigation.travelTo(launchLocation[0], launchLocation[1]);
      navigation.turnTo(launchArea[0], launchArea[1]);
      ballisticLauncher.launch(LAUNCH_DISTANCE);

    }



  }

  private static void initialLocalize() {
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
    usLocalizer.fallingEdge();

    LightLocalizer lightLocalizer = new LightLocalizer();
    int[] coordinates = {1, 1};
    lightLocalizer.setCoordinates(coordinates);
    lightLocalizer.initialPositioning();
    lightLocalizer.lightLocalize();

  }

}
