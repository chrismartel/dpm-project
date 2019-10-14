package ca.mcgill.ecse211.project;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import static ca.mcgill.ecse211.project.Resources.*;

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
  private int currentDistance;


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


  /**
   * constructor of the ultrasonic poller
   */
  public UltrasonicPoller() {
    // float array to store the data fetched
    usData = new float[usSensor.sampleSize()];
    // initialize the data lists
    usDataQueue = new LinkedList<Float>();
    usDataSortedList = new LinkedList<Float>();

    // acquire distance data in meters
    usSensor.getDistanceMode().fetchSample(usData, 0);

    // set the initial and last distances seen by the sensor
    this.currentDistance = (int) usData[0];
  }

  /**
   * procedure to run once the thread is started
   */
  public void run() {

    int temporaryDistance;
    // element used for the transfers between queue and sorted list
    float element;
    while (true) {

      // acquire distance data in meters
      usSensor.getDistanceMode().fetchSample(usData, 0);

      // remove the oldest sample from the queue and from the sorted list
      if (usDataQueue.size() == WINDOW) {
        element = usDataQueue.element();
        usDataSortedList.remove(element);
        usDataQueue.remove();
      }

      // add the new fetch sample to the queue and to the sorted list
      usDataQueue.add(usData[0]);
      usDataSortedList.add(usData[0]);

      // sort the data list
      Collections.sort(usDataSortedList);

        // set the temporary distance to be the median of the sorted list
        temporaryDistance = (int) (usDataSortedList.get((int) (usDataSortedList.size() / 2)) * 100.0);

        // filter the temporary distance the get rid of aberrant values and update the current distance
        filter(temporaryDistance);
      
      try {
        Thread.sleep(US_PERIOD);
      } catch (Exception e) {
      }
    }
  }


  /**
   * method to get the current distance seen by the sensor
   * 
   * @return the current distance seen by the sensor
   */
  public int getCurrentDistance() {
    int currentDistance = this.currentDistance;
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
    return currentDistance;
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
      this.currentDistance = 255;
    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      this.currentDistance = distance;
    }
  }



}
