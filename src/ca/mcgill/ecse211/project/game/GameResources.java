package ca.mcgill.ecse211.project.game;

import ca.mcgill.ecse211.project.game.BallisticLauncher;
import static ca.mcgill.ecse211.project.Resources.*;
import java.util.LinkedList;
import ca.mcgill.ecse211.project.Resources.Point;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.game.Navigation;
import ca.mcgill.ecse211.project.odometry.Odometer;
import ca.mcgill.ecse211.project.sensor.UltrasonicPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid cluttering the rest of the
 * codebase.
 */

public class GameResources {



  // HARDWARE DESIGN CONSTANTS

  /**
   * The wheel radius.
   */
  public static final double WHEEL_RADIUS = 2.08;

  /**
   * The robot width.
   */
  public static final double TRACK = 15.52;// 14.77;



  // ENVIRONMENT CONSTANTS

  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;
  /*
   * Width of the black lines (cm)
   */
  public static final double LINE_WIDTH = 0.5;



  // MOTORS CONSTANTS

  /**
   * The speeds at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED_SLOW = 100;
  public static final int FORWARD_SPEED_NORMAL = 150;
  public static final int FORWARD_SPEED_FAST = 200;

  /**
   * The speeds at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED_SLOW = 80;
  public static final int ROTATE_SPEED_NORMAL = 100;
  public static final int ROTATE_SPEED_FAST = 150;



  // ULTRASONIC POLLER AND CONTROLLER CONSTANTS

  /**
   * Window size for the ultrasonic sensor data polling
   */
  public static final int US_WINDOW = 5;

  /**
   * The ultrasonic sensor update period in ms. Was calculated in order to have approximately 3 pollings per degree of
   * rotation during slow rotation
   */
  public static final long US_PERIOD = 25;

  /**
   * Filter out constant to filter the distance seen by the us sensor
   */
  public static final int FILTER_OUT = 5;



  // ULTRASONIC LOCALIZATION CONSTANTS

  /**
   * Arbitrary threshold constant for rising and falling edge cases for the ultrasonic localizer
   */
  public static final int FALLINGEDGE_D = 44;

  /**
   * Noise margin constant for falling edge ultrasonic localizer
   */
  public static final int FALLINGEDGE_K = 2;



  // LIGHT LOCALIZATION CONSTANTS

  /**
   * Differential value to determine if a black line is detected or not
   */
  public static final int DIFFERENTIAL_LINE_THRESHOLD = 15; // HAS TO BE DETERMINE BY TESTING

  /**
   * Window size for the ultrasonic sensor data polling
   */

  public static final int LL_WINDOW = 1;
  /**
   * Period of the light sensor operations
   */

  public static final long LIGHT_SENSOR_PERIOD = 50; // HAS TO BE DETERMINED BY TESTING was 235
  /**
   * Light sensor to center of wheel-base distance
   */
  public static final double OFFSET_FROM_WHEELBASE = 11.4;



  // BALLISTIC LAUNCHER CONSTANTS

  /**
   * Coefficient of the launching process to adjust the speed of the motors in function of the distance
   */
  public static final double LAUNCH_COEFFICIENT = 6.8; // HAS TO BE DETERMINED BY TESTING
  /**
   * Angle of rotation of the launching motors during launch
   */
  public static final int LAUNCHING_ANGLE = 120;
  
  /**
   * Angle of rotation of the launching motors during reload
   */
  public static final int RELOAD_ANGLE = 130;

  /**
   * Distance to launch the ball
   */
  public static final int MAXIMAL_LAUNCH_DISTANCE = 200;

  /**
   * period of sleeping before launch
   */
  public static final int LAUNCH_SLEEP = 2000;

  /**
   * period of sleeping after launch
   */
  public static final int RELOAD_SLEEP = 500;

  /**
   * The launching motors acceleration.
   */
  public static final int LAUNCH_ACCELERATION = 3000;
  public static final int RELOAD_ACCELERATION = 3000;
  
  /**
   * The number of balls that the robot holds
   */
  public static final int NUMBER_OF_BALLS = 1;




  // MOTORS AND SENSORS

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);

  /**
   * The left ballistic motor.
   */
  public static final EV3LargeRegulatedMotor leftBallisticMotor = new EV3LargeRegulatedMotor(MotorPort.B);

  /**
   * The right ballistic motor.
   */
  public static final EV3LargeRegulatedMotor rightBallisticMotor = new EV3LargeRegulatedMotor(MotorPort.C);

  /**
   * The left ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor leftUsSensor = new EV3UltrasonicSensor(SensorPort.S3);

  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor frontUsSensor = new EV3UltrasonicSensor(SensorPort.S1);

  /**
   * The color sensor.
   */
  public static final EV3ColorSensor leftColorSensor = new EV3ColorSensor(SensorPort.S4);

  /**
   * The color sensor.
   */
  public static final EV3ColorSensor rightColorSensor = new EV3ColorSensor(SensorPort.S2);

  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();



  // SINGLETONS

  /**
   * The odometer singleton.
   */
  public static Odometer odometer = Odometer.getOdometer();

  /**
   * The ultrasonic poller singleton.
   */
  public static UltrasonicPoller ultrasonicPoller = UltrasonicPoller.getUltrasonicPoller();

  
  /**
   * Light localizer
   */
  public static LightLocalizer lightLocalizer = LightLocalizer.getLightLocalizer();
  
  

  // MAP CONSTANTS
  public static Region Tunnel = new Region(tng.ll, tng.ur); // default to green tunnel
  public static Point STARTING_POINT = new Point(0, 0); // default to 0,0
  public static int CORNER_NUMBER = 1;

  // GAME CONSTANTS
  /**
   * Current state of the game state machine
   */
  public static GameState gameState;
  
  /**
   * Indicates if the navigation was completed or not
   */
  public static boolean navigationCompleted = false;
  
  /**
   * Indicates if the odometry correction enable dor not
   */
  public static boolean enableCorrection = false;
  
  /**
   * Limits of the current zone
   */
  public static double currentLeftLimit;
  public static double currentRightLimit;
  public static double currentTopLimit;
  public static double currentBottomLimit;
  /**
   * Current navigation destination
   */
  public static NAVIGATION_DESTINATION navigationDestination;
  /**
   * Team color
   */
  public static COLOR color;
  /**
   * Current region the robot is on
   */
  public static REGION currentRegion;




  // OBSTACLE AVOIDANCE
  /*
   * Error acceptable for the orientation check during the wall folowing
   */
  public static final int ORIENTATION_CHECK_ERROR = 3;
  /*
   * band center for the wall following
   */
  public static final int BAND_CENTER = 20;
  /*
   * band width for the wall following
   */
  public static final int BAND_WIDTH = 3;
  /*
   * gain constant used for the P-controller
   */
  public static final double GAIN_CONSTANT = 10;
  /*
   * minimum speed used during wall following
   */
  public static final double MIN_AVOID_SPEED = 100;
  /*
   * maximum speed used during wall following
   */
  public static final double MAX_AVOID_SPEED = 300;
  /*
   * Maximal bound on the error
   */
  public static final int MAXIMAL_ERROR = 10;
  /*
   * Minimal bound on the error
   */
  public static final int MINIMAL_ERROR = -5;
  /*
   * Period of the check for distance in the wall following process
   */
  public static final int OBSTACLE_AVOIDANCE_PERIOD = 50; // in ms

  /*
   * Distance of obstacle detection
   */
  public static final int OBSTACLE_DETECTION_DISTANCE = 4;

  /*
   * Restricted points on the island
   */
  public static LinkedList<Point> restrictedLaunchPoints = new LinkedList<Point>();

  // ENUMS
  public enum COLOR {
    GREEN, RED
  }

  public enum REGION {
    RED, WATER, TUNNEL_RED, TUNNEL_GREEN, GREEN, ISLAND
  }
  
  public enum NAVIGATION_DESTINATION {
    TUNNEL1_ENTRANCE, TUNNEL2_ENTRANCE, LAUNCH_POINT, END_POINT, LOCALIZE
  }
}
