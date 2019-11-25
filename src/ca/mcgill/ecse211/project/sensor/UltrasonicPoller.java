package ca.mcgill.ecse211.project.sensor;

import ca.mcgill.ecse211.project.game.GameResources;
import ca.mcgill.ecse211.project.game.GameResources.NAVIGATION_DESTINATION;
import ca.mcgill.ecse211.project.game.GameResources.REGION;
// import ca.mcgill.ecse211.project.game.GameController.NAVIGATION_DESTINATION;
import ca.mcgill.ecse211.project.game.GameState;

/**
 * Ultrasonic poller class implementing a median filter. Polls data from the ultrasonic sensor in an independent thread
 */
public class UltrasonicPoller implements Runnable {
  /**
   * Array to store the fetch samples polled from the left us sensor
   */
  private float[] leftUsData;

  /**
   * Array to store the fetch samples polled from the front us sensor
   */
  private float[] frontUsData;

  /**
   * current distance seen by the left ultrasonic sensor
   */
  private int leftDistance;

  /**
   * current distance seen by the front ultrasonic sensor
   */
  private int frontDistance;



  private static UltrasonicPoller up; // Returned as singleton


  /**
   * Controllers for the left and front ultrasonic sensors
   */
  private UltrasonicController leftUsController;
  private UltrasonicController frontUsController;



  /**
   * constructor of the ultrasonic poller
   */
  private UltrasonicPoller() {
    leftUsData = new float[GameResources.leftUsSensor.sampleSize()];
    frontUsData = new float[GameResources.leftUsSensor.sampleSize()];
    leftUsController = new UltrasonicController(this.leftDistance);
    frontUsController = new UltrasonicController(this.frontDistance);
  }

  /**
   * Returns the UltrasonicPoller Object. Use this method to obtain an instance of UltrasonicPoller.
   * 
   * @return the UltrasonicPoller Object
   */
  public synchronized static UltrasonicPoller getUltrasonicPoller() {
    if (up == null) {
      up = new UltrasonicPoller();
    }
    return up;
  }

  /**
   * procedure to run once the thread is started
   */
  public void run() {
    // variables to control the period
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      this.pollSensors();
      // robot is avoiding so only process distance in the left controller
      if (GameResources.getGameState() == GameState.Avoidance) {
        // Process the fetched distance in the left and front controllers
        leftUsController.processDistance(this.leftDistance);
        frontUsController.processDistance(this.frontDistance);

      }
      // process distance of front sensor only if not in avoidance state
      else {
        // Process the fetched distance in the front controller
        frontUsController.processDistance(this.frontDistance);
        // when navigating on the island or when localizing after first tunnel, check for obstacles
        if ((GameResources.getGameState() == GameState.Navigation && GameResources.getCurrentRegion() == REGION.ISLAND)
            || (GameResources.getGameState() == GameState.Tunnel && GameResources.getCurrentRegion() == REGION.ISLAND
                && GameResources.getNavigationDestination() == NAVIGATION_DESTINATION.LAUNCH_POINT)) {
          // check for obstacles
          frontUsController.checkForObstacle();
        }
      }

      // record the ending time of the loop and make the thread sleep so the period is respected
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < GameResources.US_PERIOD) {
        try {
          Thread.sleep(GameResources.US_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

  /**
   * Method polling data samples from the ultrasonic sensors
   */
  private void pollSensors() {
    // robot is avoiding an object --> only poll left sensor
    if (GameResources.gameState == GameState.Avoidance) {
      // acquire distance data in meters
      GameResources.leftUsSensor.getDistanceMode().fetchSample(leftUsData, 0);
      // set the initial distance seen by the sensor
      this.leftDistance = (int) (leftUsData[0] * 100);
    }
    // robot is navigating --> only poll front sensor
    else if ((GameResources.gameState == GameState.Navigation
        || GameResources.gameState == GameState.UltrasonicLocalization)) {
      // acquire distance data in meters
      GameResources.frontUsSensor.getDistanceMode().fetchSample(frontUsData, 0);
      // set the distance seen by the sensor
      this.frontDistance = (int) (frontUsData[0] * 100);
    }
  }

  public UltrasonicController getFrontUsController() {
    return frontUsController;
  }

  public UltrasonicController getLeftUsController() {
    return leftUsController;
  }
}

