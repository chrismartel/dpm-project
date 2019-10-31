package ca.mcgill.ecse211.project.sensor;

import ca.mcgill.ecse211.project.game.Resources.*;

public class LightPoller implements Runnable{

  
  /**
   * Array to store the fetch samples polled from the left light sensor
   */
  private float[] leftLightData;

  /**
   * Array to store the fetch samples polled from the right light sensor
   */
  private float[] rightLightData;
  
  /**
   * current color seen by the left light sensor 
   */
  private int leftLight;

  /**
   * current color seen by the right light sensor
   */
  private int rightLight;
  
  private static LightPoller lp; // Returned as singleton

  
  /**
   * Controllers for the left and right light sensors
   */
  private LightSensorController leftLightController;
  private LightSensorController rightLightController;
  
  /**
   * constructor of the ultrasonic poller
   */
  private LightPoller() {
    this.pollSensors();
    leftLightController = new LightSensorController(this.leftLight);
    rightLightController = new LightSensorController(this.rightLight);
  }
  
  /**
   * Returns the LightPoller Object. Use this method to obtain an instance of LightPoller.
   * 
   * @return the LightPoller Object
   */
  public synchronized static LightPoller getLightPoller() {
    if (lp == null) {
      lp = new LightPoller();
    }
    return lp;
  }
  @Override
  public void run() {}

    /**
     * Method polling data samples from the ultrasonic sensors
     */
    private void pollSensors() {

    }

    public LightSensorController getRightLightController() {
      return rightLightController;
    }

    public LightSensorController getLeftLightController() {
      return leftLightController;
    }
}
