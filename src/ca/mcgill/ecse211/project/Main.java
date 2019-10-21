package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Localization.*;
import lejos.hardware.Button;

import static ca.mcgill.ecse211.project.Resources.*;


/**
 * The main class.
 */
public class Main {
	
//	Odometer odometer;
	
	
	
  public static void main(String[] args) {
    
	new Thread(odometer).start();
	new Thread(ultrasonicPoller).start();
	new Thread(new Display()).start();
	
	double[] launchArea = new double[2];
	launchArea[0] = 5.5;
	launchArea[1] = 7.5;
	
	//To remove when running 
	Button.waitForAnyPress();
	navigation.travel(3*TILE_SIZE);
	// Test1
	odometer.setXYT(1*TILE_SIZE, 1*TILE_SIZE, 0);
	//navigation.turn(360, ROTATE_SPEED);
	/*
	//Test3
	  UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
	   usLocalizer.fallingEdge();
	   */
	/*
	 // Test 4
	    LightLocalizer lightLocalizer = new LightLocalizer();
	    odometer.setTheta(45);
	    int[] coordinates = {1,1};
	    lightLocalizer.setCoordinates(coordinates);
	    lightLocalizer.lightLocalize();
*/
	    
	    
    initialLocalize();


	double[] launchLocation = ballisticLauncher.launchLocation(launchArea[0], launchArea[1]);

	
	navigation.travelTo(launchLocation[0], launchLocation[1]);
	
	navigation.turnTo(launchArea[0], launchArea[1]);
	
    double distance = 40;

    ballisticLauncher.launch(distance);
    

    


  }

private static void initialLocalize() {
	UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
	usLocalizer.fallingEdge();
	
	LightLocalizer lightLocalizer = new LightLocalizer();
	int[] coordinates = {1,1};
	lightLocalizer.setCoordinates(coordinates);
	lightLocalizer.initialPositioning();
	lightLocalizer.lightLocalize();
	
}
  
}
