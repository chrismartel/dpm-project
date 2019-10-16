package ca.mcgill.ecse211.project;



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

public class Resources {

  // HARDWARE DESIGN CONSTANTS

  /**
   * The wheel radius.
   */
  public static final double WHEEL_RADIUS = 2.08;

  /**
   * The robot width.
   */
  public static final double TRACK = 13.3;

  // ENVIRONMENT CONSTANTS

  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;

  // MOTORS CONSTANTS

  /**
   * The speed at which the robot moves forward in degrees per second.
   */

  public static final int FORWARD_SPEED = 150;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 100;

  /**
   * The speed at which the robot rotates slowly in degrees per second.
   */
  public static final int ROTATE_SPEED_SLOW = 50;

  /**
   * The acceleration.
   */
  public static final int ACCELERATION = 9999;

  
  
  // ULTRASONIC POLLER CONSTANTS

  /**
   * Window size for the ultrasonic sensor data polling
   */
  public static final int US_WINDOW = 5;

  /**
   * The ultrasonic sensor update period in ms. Was calculated in order to have approximately 3 pollings per degree of
   * rotation during slow rotation
   */
  public static final long US_PERIOD = 23;

  /**
   * Filter out constant to filter the distance seen by the us sensor
   */
  public static final int FILTER_OUT = 5;


  
  // ULTRASONIC LOCALIZATION CONSTANTS

  /**
   * Arbitrary threshold constant for rising and falling edge cases for the ultrasonic localizer
   */
  public static final int COMMON_D = 40;

  /**
   * Noise margin constant for falling edge ultrasonic localizer
   */
  public static final int FALLINGEDGE_K = 1;

  /**
   * Noise margin constant for rising edge ultrasonic localizer
   */
  public static final int RISINGEDGE_K = 3;

  // LIGHT LOCALIZATION CONSTANTS

  /**
   * Threshold value to determine if a black line is detected or not
   */
  public static final int LINE_THRESHOLD = 35;
  
  /**
   * Differential value to determine if a black line is detected or not
   */
  public static final int DIFFERENTIAL_LINE_THRESHOLD = 7;

  /*
   * Width of the black lines (cm)
   */
  public static final double LINE_WIDTH = 0.5;

  /**
   * Period of the light sensor operations
   */
  public static final long LIGHT_SENSOR_PERIOD = 150;
  /**
   * Light sensor to center of wheel-base distance
   */
  public static final int OFFSET_FROM_WHEELBASE = 12;

  // BALLISTIC LAUNCHER CONSTANTS
  
  /**
   * Coefficient of the launching process to adjust the speed of the motors in function of the distance
   */
  public static final int LAUNCH_COEFFICIENT = 12;
  
  /**
   * Angle of rotation of the launching motors during launch
   */
  public static final int LAUNCHING_ANGLE = 360;
  
  // MOTORS AND SENSORS

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.D);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.A);
  
  /**
   * The left ballistic motor.
   */
  public static final EV3LargeRegulatedMotor leftBallisticMotor = new EV3LargeRegulatedMotor(MotorPort.B);

  /**
   * The right ballistic motor.
   */
  public static final EV3LargeRegulatedMotor rightBallisticMotor = new EV3LargeRegulatedMotor(MotorPort.C);

  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S4);

  /**
   * The color sensor.
   */
  public static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);

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
   * The navigation singleton.
   */
  public static Navigation navigation = Navigation.getNavigation();

  /**
   * The ultrasonic poller singleton.
   */
  public static UltrasonicPoller ultrasonicPoller = UltrasonicPoller.getUltrasonicPoller();


}
