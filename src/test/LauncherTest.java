package test;

import lejos.hardware.Button;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.Localization.LightLocalizer;
import ca.mcgill.ecse211.project.game.BallisticLauncher;
import ca.mcgill.ecse211.project.game.Navigation;
import ca.mcgill.ecse211.project.odometry.Odometer;
public class LauncherTest {

  
  
  public static void main(String[] args) throws InterruptedException {
    Odometer od = Odometer.getOdometer();
    od.setXYT(0, 0, 0);
    new Thread(Odometer.getOdometer()).start();
    new Thread(new Display()).start();
    BallisticLauncher bl = new BallisticLauncher();
    int speed = 275;
    Thread.sleep(2250);
    while(true) {

    
    LightLocalizer.twoLineDetection();
    if(speed == 225)
      od.setXYT(30.48, 30.48*5+11.4, 0);
    else if(speed == 275)
      od.setXYT(30.48, 30.48*3+11.4, 0);
    
    else
      od.setXYT(30.48, 30.48*4+11.4, 0);
    
    Thread.sleep(2250);
    Navigation.turnTo(0.28, 11, 225);
    bl.launch(0,speed);
    Thread.sleep(2250);
    bl.reload();
    
    Thread.sleep(2250);
    bl.launch(0,speed);
    Thread.sleep(2250);
    bl.reload();
    
    Thread.sleep(2250);
    bl.launch(0,speed);
    Thread.sleep(2250);
    bl.reload();
    Thread.sleep(2250);
    
    
    bl.launch(0,speed);
    Thread.sleep(2250);
    bl.reload();
    Thread.sleep(2250);
    

    bl.launch(0,speed);
    Thread.sleep(2250);
    bl.reload();
    Thread.sleep(2250);
    
    

    
   if(speed == 225)
     speed = 250;
   else if (speed == 250)
     speed = 275;
   else
     speed = 225;
   
    Button.waitForAnyPress();
  }
  }
}   
