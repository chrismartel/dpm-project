package ca.mcgill.ecse211.project;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Ultrasonic poller class implementing a median filter. Polls data from the ultrasonic sensor in an independent thread
 */
public class UltrasonicPoller implements Runnable {
  /**
   * Array to store the fetch samples
   */
  private float[] usData;
  /**
   * current distance seen by the ultrasonic sensor poller
   */
  private int distance;

  // Thread control tools
  /**
   * Fair lock for concurrent writing
   */
  private static Lock lock = new ReentrantLock(true);
  /**
   * Indicates if a thread is trying to reset any position parameters
   */
  private volatile boolean isResetting = false;
  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();


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
    // acquire distance data in meters
    usSensor.getDistanceMode().fetchSample(usData, 0);
    // set the initial distance seen by the sensor
    this.distance = (int) (usData[0]*100);
    leftUsController = new UltrasonicController(this.distance);
    rightUsController = new UltrasonicController(this.distance);
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
      // acquire distance data in meters from the sensor and store it in float array
      usSensor.getDistanceMode().fetchSample(usData, 0);
      // scale the distance fetched and update the current distance
      this.distance = (int) (usData[0]*100);
      leftUsController.processDistance(this.distance);
      rightUsController.processDistance(this.distance);

      
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
   * method to get the current distance seen by the sensor
   * 
   * @return the current distance seen by the sensor
   */
  public int getDistance() {
    int distance = this.distance;
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return distance;
  }

}
