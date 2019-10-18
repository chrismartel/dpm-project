package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

public class BallisticLauncher {

  private int motorLaunchingSpeed;

  void launch(double distance) {
    // motorLaunchingSpeed = (int) (LAUNCH_COEFFICIENT * distance);
    motorLaunchingSpeed = 800;

    leftBallisticMotor.setSpeed(motorLaunchingSpeed);
    rightBallisticMotor.setSpeed(motorLaunchingSpeed);
    leftBallisticMotor.setAcceleration(ACCELERATION);
    rightBallisticMotor.setAcceleration(ACCELERATION);
    leftBallisticMotor.rotate(-LAUNCHING_ANGLE, true);
    rightBallisticMotor.rotate(-LAUNCHING_ANGLE, false);


  }
}
