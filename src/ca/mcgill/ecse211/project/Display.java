package ca.mcgill.ecse211.project;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.project.game.GameResources;
// static import to avoid duplicating variables and make the code easier to read


/**
 * This class is used to display the content of the odometer variables (x, y, Theta) Was used mainly for debugging
 * purposes
 */
public class Display implements Runnable {

  /**
   * Position parameters: x , y , theta
   */
  private double[] position;
  /**
   * Period of the display thread
   */
  private final long DISPLAY_PERIOD = 25;
  /**
   * Timeout value
   */
  private long timeout = Long.MAX_VALUE;

  public void run() {

    GameResources.LCD.clear();

    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = GameResources.odometer.getXYT();

      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      GameResources.LCD.drawString("X: " + numberFormat.format(position[0] / GameResources.TILE_SIZE), 0, 0);
      GameResources.LCD.drawString("Y: " + numberFormat.format(position[1] / GameResources.TILE_SIZE), 0, 1);
      GameResources.LCD.drawString("T: " + numberFormat.format(position[2]), 0, 2);

      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

  /**
   * Sets the timeout in ms.
   * 
   * @param timeout period for display
   */
  public void setTimeout(long timeout) {
    this.timeout = timeout;
  }

  /**
   * Shows the text on the LCD, line by line.
   * 
   * @param strings comma-separated list of strings, one per line
   */
  public static void showText(String... strings) {
    GameResources.LCD.clear();
    for (int i = 0; i < strings.length; i++) {
      GameResources.LCD.drawString(strings[i], 0, i);
    }
  }

}
