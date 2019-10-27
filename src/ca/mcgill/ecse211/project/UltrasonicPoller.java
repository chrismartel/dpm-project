package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Ultrasonic poller class implementing a median filter. Polls data from the ultrasonic sensor in an independent thread
 */
public class UltrasonicPoller implements Runnable {
  /**
   * Array to store the fetch samples polled from the left us sensor
   */
  private float[] leftUsData;
  
  /**
   * Array to store the fetch samples polled from the right us sensor
   */
  private float[] rightUsData;
  
  /**
   * current distance seen by the left ultrasonic sensor poller
   */
  private int leftDistance;
  
  /**
   * current distance seen by the left ultrasonic sensor poller
   */
  private int rightDistance;




  private static UltrasonicPoller up; // Returned as singleton
  
  
  /**
   * Controllers for the left and right ultrasonic sensors
   */
  private UltrasonicController leftUsController;
  private UltrasonicController rightUsController;

;
  /**
   * constructor of the ultrasonic poller
   */
  private UltrasonicPoller() {
    this.pollSensors(); 
    leftUsController = new UltrasonicController(this.leftDistance);
    rightUsController = new UltrasonicController(this.rightDistance);
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
      // Process the fetched distance in the controllers
      leftUsController.processDistance(this.leftDistance);
      rightUsController.processDistance(this.rightDistance);
      // record the ending time of the loop and make the thread sleep so the period is respected
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < US_PERIOD) {
        try {
          Thread.sleep(US_PERIOD - (updateEnd - updateStart));
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
    // acquire distance data in meters
    leftUsSensor.getDistanceMode().fetchSample(leftUsData, 0);
    // set the initial distance seen by the sensor
    this.leftDistance = (int) (leftUsData[0]*100);
    
    // acquire distance data in meters
    rightUsSensor.getDistanceMode().fetchSample(rightUsData, 0);
    // set the initial distance seen by the sensor
    this.rightDistance = (int) (rightUsData[0]*100);
  }

}
