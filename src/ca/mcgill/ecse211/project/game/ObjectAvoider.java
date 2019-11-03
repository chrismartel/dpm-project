package ca.mcgill.ecse211.project.game;
import static ca.mcgill.ecse211.project.game.Resources.*;

public class ObjectAvoider {

  private int objectDistance;
  private int[] goalCoordinates = new int[2];
  
  public void wallFollower() {
    Navigation.turn(90, Resources.ROTATE_SPEED);
    // wall follows until it faces its desired coordinates
    while(!orientationCheck()) {
      // TODO: execute wall following process
    }
  }
  
  /**
   * Method indicating if the orientation of the robot is correct or not during the wall following process
   * @return true if the orientation of the robot 
   * */
  public boolean orientationCheck() {
    double currentX = odometer.getX();
    double currentY = odometer.getY();
    double goalX = goalCoordinates[0] * TILE_SIZE;
    double goalY = goalCoordinates[1] * TILE_SIZE;
    double dX = goalX - currentX;
    double dY = goalY - currentY;
    double turnToAngle = Math.abs(Math.toDegrees(Math.atan2(dX, dY)));
    if(turnToAngle <= ORIENTATION_CHECK_ERROR) {
      return true;
    }
    return false;  
  }
  
  
  
  public int getObjectDistance() {
    return objectDistance;
  }
  public void setObjectDistance(int objectDistance) {
    this.objectDistance = objectDistance;
  }
  
public int[] getGoalCoordinates() {
  return goalCoordinates;
}
public void setGoalCoordinates(int[] goalCoordinates) {
  this.goalCoordinates = goalCoordinates;
}
}
