package ca.mcgill.ecse211.project.odometry;

import static ca.mcgill.ecse211.project.game.GameResources.*;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class implements the odometer of the robot establishing a local coordinate system to enable the robot to
 * navigate accurately through the field. This classes implements a lock to avoid concurrent writing when getting or
 * setting values. It is implemented as a thread.
 */
public class Odometer implements Runnable {

  /**
   * The x-axis position in cm.
   */
  private volatile double x;

  /**
   * The y-axis position in cm.
   */
  private volatile double y; // y-axis position

  /**
   * The orientation in degrees.
   */
  private volatile double theta; // Head angle

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

  private static Odometer odo; // Returned as singleton

  // Motor-related variables
  private static int leftMotorTachoCount = 0;
  private static int rightMotorTachoCount = 0;

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It cannot be accessed
   * externally.
   */
  private Odometer() {
    setXYT(0, 0, 0);
  }

  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   * 
   * @return the Odometer Object
   */
  public synchronized static Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }

    return odo;
  }

  /**
   * This method is where the logic for the odometer will run.
   */
  public void run() {
    long updateStart, updateEnd;
    while (true) {
      updateStart = System.currentTimeMillis();

      /**
       * get the actual tachoCount for both motors
       */
      int nowLeftMotorTachoCount = leftMotor.getTachoCount();
      int nowRightMotorTachoCount = rightMotor.getTachoCount();

      /**
       * get the current coordinates of the odometer
       */
      double[] position = getXYT();


      /**
       * Compute both wheel displacements
       */
      double distLeftWheel = Math.PI * WHEEL_RADIUS * (nowLeftMotorTachoCount - getLeftMotorTachoCount()) / 180;
      double distRightWheel = Math.PI * WHEEL_RADIUS * (nowRightMotorTachoCount - getRightMotorTachoCount()) / 180;

      /**
       * Compute vehicle heading change
       */
      double deltaTheta = (distLeftWheel - distRightWheel) / TRACK;

      /**
       * Compute the variation in angle in degrees
       */
      double deltaThetaDegrees = Math.toDegrees(deltaTheta);

      /**
       * temporary angle
       */
      position[2] += deltaTheta;

      /**
       * Update the tachoCounts of both motors for the odometer
       */
      setLeftMotorTachoCount(nowLeftMotorTachoCount);
      setRightMotorTachoCount(nowRightMotorTachoCount);

      /**
       * Compute vehicle displacement
       */
      double vehicleDisplacement = 0.5 * (distLeftWheel + distRightWheel);

      /**
       * Compute the X component of displacement
       */
      double deltaX = vehicleDisplacement * Math.sin(Math.toRadians(position[2]));

      /**
       * Compute the Y component of displacement
       */
      double deltaY = vehicleDisplacement * Math.cos(Math.toRadians(position[2]));

      /**
       * Update the odometer values with the new calculated values
       */
      odometer.update(deltaX, deltaY, deltaThetaDegrees);

      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }


  /**
   * Returns the Odometer data.
   * 
   * @return the odometer data.
   */
  public double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;
  }

  /**
   * Returns the Odometer x value.
   * 
   * @return the odometer x position.
   */
  public double getX() {
    double result;
    synchronized (lock) {
      result = x;
    }
    return result;
  }

  /**
   * Returns the Odometer y value.
   * 
   * @return the odometer y position.
   */
  public double getY() {
    double result;
    synchronized (lock) {
      result = y;
    }
    return result;
  }

  /**
   * Returns the Odometer theta value.
   * 
   * @return the odometer theta position.
   */
  public double getTheta() {
    double result;
    synchronized (lock) {
      result = theta;
    }
    return result;
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for odometry.
   * 
   * @param dx
   * @param dy
   * @param dtheta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates within 360 degrees
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }

  }

  /**
   * Overrides the values of x, y and theta. Used for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXYT(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites y. Used for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Used for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * return the rightMotorTachoCount
   */
  public int getRightMotorTachoCount() {
    synchronized (lock) {
      return rightMotorTachoCount;
    }
  }

  /**
   * return the leftMotorTachoCount
   */
  public int getLeftMotorTachoCount() {
    synchronized (lock) {
      return leftMotorTachoCount;
    }
  }

  /**
   * set the rightMotorTachoCount
   */
  public void setRightMotorTachoCount(int rightMotorTachoCount) {
    synchronized (lock) {
      Odometer.rightMotorTachoCount = rightMotorTachoCount;
    }
  }

  /**
   * set the leftMotorTachoCount
   */
  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
    synchronized (lock) {
      Odometer.leftMotorTachoCount = leftMotorTachoCount;
    }
  }

}
