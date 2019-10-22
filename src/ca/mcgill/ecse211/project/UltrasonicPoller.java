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
   * window to store the data polled from the UsSensor in a queue
   */
  private Queue<Float> usDataQueue;

  /**
   * linked list to sort the data from the window of samples
   */
  private LinkedList<Float> usDataSortedList;

  /**
   * Array to store the fetch samples
   */
  private float[] usData;


  /**
   * current distance processed by the ultrasonic sensor poller
   */
  private int distance;

  /**
   * last distance processed by the ultrasonic sensor poller
   */
  private int lastDistance;


  /**
   * filter control value used for the filter method
   */
  private int filterControl;


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
   * constructor of the ultrasonic poller
   */
  private UltrasonicPoller() {
    // float array to store the data fetched
    usData = new float[usSensor.sampleSize()];
    // initialize the data lists
    usDataQueue = new LinkedList<Float>();
    usDataSortedList = new LinkedList<Float>();
    // acquire distance data in meters
    usSensor.getDistanceMode().fetchSample(usData, 0);
    // set the initial distance seen by the sensor
    this.distance = (int) usData[0];
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

    // temporary distance to use before the filter
    int temporaryDistance;

    // element used for the transfers between queue and sorted list
    float element;
    while (true) {
      updateStart = System.currentTimeMillis();


      // acquire distance data in meters from the sensor and store it in float array
      usSensor.getDistanceMode().fetchSample(usData, 0);

      // if the lists are full remove the oldest sample from the queue and from the sorted list
      if (usDataQueue.size() == US_WINDOW) {
        // retrieves the element at head of the queue
        element = usDataQueue.element();
        // removes the element at top of the queue from the sorted list
        usDataSortedList.remove(element);
        // removes the element at head of the queue
        usDataQueue.remove();
      }

      // add the new fetched sample to the end of the queue and to the sorted list
      usDataQueue.add(usData[0]);
      usDataSortedList.add(usData[0]);

      // sort the data list
      Collections.sort(usDataSortedList);

      this.lastDistance = this.distance;

      // set the temporary distance to be the median of the sorted list
      temporaryDistance = (int) (usDataSortedList.get((int) (usDataSortedList.size() / 2)) * 100.0);

      // filter the temporary distance the get rid of aberrant values and update the current distance
      filter(temporaryDistance);

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
   * method to get the current and last distances seen by the sensor
   * 
   * @return an array containing the current and last distances seen by the sensor
   */
  public int[] getDistances() {
    int[] distances = new int[2];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }
      distances[0] = this.distance;
      distances[1] = this.lastDistance;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return distances;
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

  /**
   * method to filter the aberrant values seen by the ultrasonic sensor
   * 
   * @param the distance in cm to filter
   */
  void filter(int distance) {
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the filter value
      filterControl++;
    } else if (distance >= 255) {
      // Repeated large values, so there is nothing there: leave the distance alone
      this.distance = 255;
    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      this.distance = distance;
    }
  }



}
