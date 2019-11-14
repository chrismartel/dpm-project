package ca.mcgill.ecse211.project.sensor;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;
import ca.mcgill.ecse211.project.game.GameState;

public class UltrasonicController {

  /**
   * current distance processed by the ultrasonic controller
   */
  private int currentDistance;

  /**
   * last distance processed by the ultrasonic controller
   */
  private int previousDistance;


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
    int temporaryDistance;
    // distance initially not changed by the filter out method
    this.previousDistance = this.currentDistance;
    // use median filter only if distance changed in filter out method
      // if the lists are full remove the oldest sample from the queue and from the sorted list
      if (usDataQueue.size() == US_WINDOW) {
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
      if (usDataQueue.size() == US_WINDOW) {
        temporaryDistance = (int) (usDataSortedList.get((int) (US_WINDOW / 2)));
        filter(temporaryDistance);
      }
      // filter the temporary distance the get rid of aberrant values and update the current distance
    
  }

  /**
   * method to filter the aberrant values processed by the controller
   * 
   * @param the distance in cm to filter
   */
  public void filter(int distance) {
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

  /**
   * method to get the current and last distances processed by the controller
   * 
   * @return an array containing the current and last distances processed by the controller
   */
  public int[] getDistances() {
    int[] distances = new int[2];
    distances[0] = this.currentDistance;
    distances[1] = this.previousDistance;
    return distances;
  }

  /**
   * method to get the current distance processed by the controller
   * 
   * @return the current distance processed by the controller
   */
  public int getDistance() {
    int distance = this.currentDistance;
    return distance;
  }

  public void checkForObstacle() {
    // obstacle ahead
    if (currentDistance <= OBSTACLE_DETECTION_DISTANCE) {
      gameState = GameState.Avoidance;
    }
  }

}
