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
	
	//To remove when running 
	Button.waitForAnyPress();
	
    initialLocalize();
	//navigation.travelTo(2,2);
	//navigation.turnTo(2, 3);
	
	
    BallisticLauncher launcher = new BallisticLauncher();
    double distance = 40;
    //launcher.launch(distance);
    


  }

private static void initialLocalize() {
	UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
	usLocalizer.fallingEdge();
	
	LightLocalizer lightLocalizer = new LightLocalizer();
	lightLocalizer.initialPositioning();
	lightLocalizer.lightLocalize();
	
}
  
}
