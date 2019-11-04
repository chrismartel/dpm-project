package ca.mcgill.ecse211.project.game;

import static ca.mcgill.ecse211.project.game.Resources.*;
import ca.mcgill.ecse211.project.localization.LightLocalizer;
import ca.mcgill.ecse211.project.localization.UltrasonicLocalizer;
import lejos.hardware.Button;

public class GameController {

  public enum NAVIGATION_TYPE {
    TUNNEL1_ENTRANCE, TUNNEL2_ENTRANCE, LAUNCH_POINT, END_POINT
  }

  public static void main(String[] args) {

    // initialize class instances needed
    GameNavigation gameNavigation = new GameNavigation();;
    LightLocalizer lightLocalizer = new LightLocalizer();;
    UltrasonicLocalizer ultrasonicLocalizer = new UltrasonicLocalizer();
    BallisticLauncher ballisticLauncher = new BallisticLauncher();

    // initiliaze threads
    Thread odometerThread = new Thread(odometer);
    Thread usPollerTread = new Thread(ultrasonicPoller);

    // start threads
    odometerThread.start();
    usPollerTread.start();

    // initial state
    gameState = GameState.Initialization;

    // Execute until the game reaches the Done state
    while (gameState != GameState.Done) {
      // initialization process
      if (gameState == GameState.Initialization) {
        // TODO: get info from wifi class and generate map
        // set currentRegion and color depending on wifi
        gameState = GameState.UltrasonicLocalization;
      }
      // ultrasonic localization process
      else if (gameState == GameState.UltrasonicLocalization) {
        // TODO: us localization using falling edge
        gameState = GameState.LightLocalization;
      }
      // light localization process
      else if (gameState == GameState.LightLocalization) {
        // TODO: light localization using 2 sensors at the back
        gameState = GameState.Navigation;
        navigationType = NAVIGATION_TYPE.TUNNEL1_ENTRANCE;
      }
      // navigation process
      else if (gameState == GameState.Navigation) {
        // TODO: navigate to point
        navigationCompleted = false;
        // determine where to navigate
        switch (navigationType) {
          case TUNNEL1_ENTRANCE:
            Navigation.travelTo(gameNavigation.getTunnelEntrance()[0], gameNavigation.getTunnelEntrance()[1],
                FORWARD_SPEED_NORMAL);
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationType = NAVIGATION_TYPE.LAUNCH_POINT;
            }
          case TUNNEL2_ENTRANCE:
            Navigation.travelTo(gameNavigation.getTunnelEntrance()[0], gameNavigation.getTunnelEntrance()[1],
                FORWARD_SPEED_NORMAL);
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationType = NAVIGATION_TYPE.END_POINT;
            }
          case LAUNCH_POINT:
            // TODO: navigate to launch point
            if (navigationCompleted == true) {
              gameState = GameState.Tunnel;
              navigationType = NAVIGATION_TYPE.TUNNEL2_ENTRANCE;
            }
          case END_POINT:
            Navigation.travelTo(STARTING_CORNER[0], STARTING_CORNER[1], FORWARD_SPEED_NORMAL);
            if (navigationCompleted == true) {
              gameState = GameState.Done;
            }

        }
      }
      // tunnel traversal process
      else if (gameState == GameState.Tunnel) {
        // TODO: navigate through tunnel
      }
      // object avoidance process
      else if (gameState == GameState.Avoidance) {
        // TODO: object avoidance process

      }
      // launch process
      else if (gameState == GameState.Launch) {
        // TODO: launch and reload
        navigationType = NAVIGATION_TYPE.TUNNEL2_ENTRANCE;
      }

    }



  }
}
