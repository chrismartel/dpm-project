package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Localization.*;
import lejos.hardware.Button;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * The main class.
 */
public class Main {

  /**
   * The main method
   */
	public static void main(String[] args) {

		int buttonChoice;
		// Initialize the odometer and the distance polling threads
		Thread odometerThread = new Thread(odometer);
		odometerThread.start();

		buttonChoice = chooseFirstChoice();
		// stationary launch
		if (buttonChoice == Button.ID_LEFT) {
			LCD.clear();
			// Launches as many times as the enter button is pressed.
			while (true) {
				LCD.drawString("PRESS ENTER ", 1, 2);
				LCD.drawString("TO LAUNCH ", 1, 3);
				int buttonChoice3 = Button.waitForAnyPress();
				if (buttonChoice3 == Button.ID_ENTER) {
					LCD.clear();
					ballisticLauncher.launch(LAUNCH_DISTANCE);
					ballisticLauncher.reload();
				} else if (buttonChoice3 == Button.ID_ESCAPE) {
					System.exit(0);
				}
			}
		}
		// mobile launch
		else if (buttonChoice == Button.ID_RIGHT) {
			// LCD.clear();
			new Thread(new Display()).start();
			double[] targetArea = new double[2];
			targetArea[0] = 6.5;
			targetArea[1] = 1.5;
			double[] launchArea = new double[2];
			launchArea[0] = 2;
			launchArea[1] = 1.5;
			
			//Draw Launch and Target Location to the LCD

			LCD.drawString("Target X:" + targetArea[0], 0, 3);
			LCD.drawString("Target Y:" + targetArea[1], 0, 4);
			LCD.drawString("Launch X:" + launchArea[0], 0, 5);
			LCD.drawString("Launch Y:" + launchArea[1], 0, 6);
			
			//Localize to 1,1 and point to 0
			initialLocalize();
			
			navigation.travelTo(launchArea[0], launchArea[1]);
			navigation.turnTo(targetArea[0], targetArea[1]);
			

			// Launches as many times as the enter button is pressed.
			while (true) {
				int buttonChoice3 = Button.waitForAnyPress();
				if (buttonChoice3 == Button.ID_ENTER) {
					ballisticLauncher.launch(LAUNCH_DISTANCE);
					ballisticLauncher.reload();
				} else if (buttonChoice3 == Button.ID_ESCAPE) {
					System.exit(0);
				}
			}
		}
	}

	/**
	 * Asks the user which launch option to execute.
	 * 
	 * @return the user choice
	 */
	private static int chooseFirstChoice() {
		int buttonChoice;
		Display.showText(
						"< Left  | Right  >",
						"        |         ",
						"Station-| Mobile  ",
						"  ary   | Launch  ",
						"Launch  |         ");
		do {
			buttonChoice = Button.waitForAnyPress(); // left or right press
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		return buttonChoice;
		
	}

	/**
	 * Initial Localization at 1,1 coordinate pointing to theta 0.
	 */
	private static void initialLocalize() {
		Thread usPollerThread = new Thread(ultrasonicPoller);
		usPollerThread.start();
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
		usLocalizer.fallingEdge();
		LightLocalizer lightLocalizer = new LightLocalizer();
		int[] coordinates = { 1, 1 };
		lightLocalizer.setCoordinates(coordinates);
		lightLocalizer.initialPositioning();
		lightLocalizer.lightLocalize();
		
//		try {
//			usPollerThread.sleep(8888888);
//		} catch (InterruptedException e) {
//			e.printStackTrace();
//		}

	}
	
}
