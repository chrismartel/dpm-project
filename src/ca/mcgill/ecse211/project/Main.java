package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Localization.*;
import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * The main class.
 */
public class Main {

	// Odometer odometer;

	public static void main(String[] args) {

		int buttonChoice;
		// Start the odometer and the distance polling threads
		Thread odometerThread = new Thread(odometer);
		Thread usPollerThread = new Thread(ultrasonicPoller);
		usPollerThread.start();
		odometerThread.start();

		buttonChoice = choosePathFirstChoice();

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
					ballisticLauncher.launch(120);
					ballisticLauncher.reload();
				} else if (buttonChoice3 == Button.ID_ESCAPE) {
					System.exit(0);
				}
			}
		}
		// mobile launch
		else if (buttonChoice == Button.ID_RIGHT) {
			LCD.clear();
			new Thread(new Display()).start();
			double[] targetArea = new double[2];
			targetArea[0] = 5.5;
			targetArea[1] = 7.5;
			double[] launchArea = new double[2];
			launchArea[0] = 5.5;
			launchArea[1] = 3.5;
			initialLocalize();

			// double[] launchArea = ballisticLauncher.launchLocation(targetArea[0],
			// targetArea[1]);
			navigation.travelTo(launchArea[0], launchArea[1]);
			navigation.turnTo(targetArea[0], targetArea[1]);
			ballisticLauncher.launch(LAUNCH_DISTANCE);

		}

	}

	/**
	 * Asks the user which launch option to execute.
	 * 
	 * @return the user choice
	 */
	private static int choosePathFirstChoice() {
		int buttonChoice;
		Display.showText("<  Left   |  Right  >",
						"          |         ",
						"Stationary|  Mobile ",
						"  Launch  |  Launch ",
						"          |         ");
		do {
			buttonChoice = Button.waitForAnyPress(); // left or right press
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		return buttonChoice;
		
	}

	private static void initialLocalize() {
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();
		usLocalizer.fallingEdge();

		LightLocalizer lightLocalizer = new LightLocalizer();
		int[] coordinates = { 1, 1 };
		lightLocalizer.setCoordinates(coordinates);
		lightLocalizer.initialPositioning();
		lightLocalizer.lightLocalize();

	}

}
