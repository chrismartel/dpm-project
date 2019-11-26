package ca.mcgill.ecse211.project.sensor;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import ca.mcgill.ecse211.project.game.GameResources;
import ca.mcgill.ecse211.project.game.GameState;
import lejos.hardware.Sound;

public class UltrasonicController {

  /**
   * current distance processed by the ultrasonic controller
   */
  private int currentDistance;

  /**
   * counter to determine if an obstacle is detected
   */
  private int obstacleDetectionCounter;

  /**
   * last distance processed by the ultrasonic controller
   */
  private int previousDistance;


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
   * window to store the data polled from the UsSensor in a queue
   */
  private Queue<Integer> usDataQueue;

  /**
   * linked list to sort the data from the window of samples
   */
  private LinkedList<Integer> usDataSortedList;

  /**
   * filter control value used for the filter method
   */
  private int filterControl;

  UltrasonicController(int initialDistance) {

    // initialize the data lists
    this.usDataQueue = new LinkedList<Integer>();
    this.usDataSortedList = new LinkedList<Integer>();
    // set the initial distances of the controller
    this.currentDistance = initialDistance;
    this.previousDistance = initialDistance;
  }


  public void processDistance(int distance) {

    lock.lock();
    isResetting = true;
    try {
      int temporaryDistance;
      // distance initially not changed by the filter out method
      this.previousDistance = this.currentDistance;
      // use median filter only if distance changed in filter out method
      // if the lists are full remove the oldest sample from the queue and from the sorted list
      if (usDataQueue.size() == GameResources.US_WINDOW) {
        // removes the element at top of the queue from the sorted list
        usDataSortedList.remove(usDataQueue.element());
        // removes the element at head of the queue
        usDataQueue.remove();
      }
      // add the new fetched sample to the end of the queue and to the sorted list
      usDataQueue.add(distance);
      usDataSortedList.add(distance);

      // sort the data list
      Collections.sort(usDataSortedList);

      // set the temporary distance to be the median of the sorted list
      if (usDataQueue.size() == GameResources.US_WINDOW) {
        temporaryDistance = (int) (usDataSortedList.get((int) (GameResources.US_WINDOW / 2)));
        filter(temporaryDistance);
      }
      // filter the temporary distance the get rid of aberrant values and update the current distance
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * method to filter the aberrant values processed by the controller
   * 
   * @param the distance in cm to filter
   */
  public void filter(int distance) {
    if (distance >= 255 && filterControl < GameResources.FILTER_OUT) {
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

  /**
   * method to get the current and last distances processed by the controller
   * 
   * @return an array containing the current and last distances processed by the controller
   */
  public int[] getDistances() {
    int[] distances = new int[2];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }
      distances[0] = this.currentDistance;
      distances[1] = this.previousDistance;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return distances;
  }

  /**
   * method to get the current distance processed by the controller
   * 
   * @return the current distance processed by the controller
   */
  public int getDistance() {
    int distance = 0;
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }
      distance = this.currentDistance;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return distance;
  }

  public void checkForObstacle() {
    lock.lock();
    isResetting = true;
    try {
      // obstacle ahead
      if (currentDistance <= GameResources.OBSTACLE_DETECTION_DISTANCE) {
        // increment the counter, when 3 detections have been made --> obstacle detected
        obstacleDetectionCounter++;
        if (obstacleDetectionCounter == 3) {
          GameResources.setObstacleDetected(true);
          GameResources.setGameState(GameState.Avoidance);
          Sound.beep();
        }
      } else {
        obstacleDetectionCounter = 0;


      }
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }

  }

}
