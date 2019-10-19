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
	launchArea[0] = 4.5;
	launchArea[1] = 6.5;
	
	//To remove when running 
	Button.waitForAnyPress();

	
    initialLocalize();


	localize();
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
	lightLocalizer.initialPositioning();
	lightLocalizer.lightLocalize();
	
}
  
}
