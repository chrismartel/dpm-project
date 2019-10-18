package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.Localization.UltrasonicLocalizer;
import lejos.hardware.Button;

import static ca.mcgill.ecse211.project.Resources.*;


/**
 * The main class.
 */
public class Main {
	
//	Odometer odometer;
	
	
	
  public static void main(String[] args) {
	new Thread(odometer).start();
	new Thread(new Display()).start();
	
	//To remove when running 
	Button.waitForAnyPress();
	
	localize();
	Navigation navigator = new Navigation();
	navigator.travelTo(2,2);
	navigator.turnTo(2, 3);
	
	
    BallisticLauncher launcher = new BallisticLauncher();
    double distance = 40;
    launcher.launch(distance);
    


  }

private static void localize() {
	UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
	usLocalizer.fallingEdge();
	
	LightLocalizer lightLocalizer = new LightLocalizer();
	lightLocalizer.lightLocalize();
	
}
  
}
