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

//NEED TO BE ORGANIZED
public class Resources {

  /**
   * The wheel radius.
   */
  public static final double WHEEL_RADIUS = 2.08;

  /**
   * The robot width.
   */
  public static final double TRACK = 13.3;

  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;


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
  public static final int ACCELERATION = 3000;

  /**
   * Window size for the ultrasonic sensor data polling
   */
  public static final int WINDOW = 5;

  /**
   * The ultrasonic sensor update period in ms. Was calculated in order to have approximately 3 pollings per degree of
   * rotation during slow rotation
   */
  public static final long US_PERIOD = 23;

  /**
   * Ultrasonic poller counter constant
   */
  public static final int USCC = 15;


  /**
   * The rising/falling wall threshold
   */
  public static final double WALL_THRESHOLD = 30;// home 35

  /**
   * The wall threshold error
   */
  public static final double WALL_THRESHOLD_ERROR = 2;


  /**
   * Filter out constant to filter the distance seen by the us sensor
   */
  public static final int FILTER_OUT = 5;

  /**
   * The black line detection threshold
   */
  public static final double LINE_THRESHOLD = 6; // home 8

  /*
   * Width of the black lines (cm)
   */
  public static final double LINE_WIDTH = 0.5;

  /**
   * Period of the light sensor operations
   */
  public static final long LIGHT_SENSOR_PERIOD = 150;// home 175
  /**
   * Distance between center of rotation and light sensor
   */
  public static final double LIGHT_SENSOR_OFFSET = 13.8;

  /**
   * Distance that the robot needs to be from the wall before light localizing
   */
  public static final double INITIAL_POSITIONING_WALL_DISTANCE = 10;

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);

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

  /**
   * The odometer singleton.
   */
  public static Odometer odometer = Odometer.getOdometer();

  /**
   * The navigation singleton.
   */
  public static Navigation navigation = Navigation.getNavigation();


}