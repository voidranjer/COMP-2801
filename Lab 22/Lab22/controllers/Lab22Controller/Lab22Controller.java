// Author: Mark Lanthier (SN:100000001)
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Display;

public class Lab22Controller {

  //private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  private static final float   ADJACENCY_TOLERANCE = 0.15f; // 15cm to be considered on same wall
  private static final float   TOO_CLOSE_DISTANCE = 0.52f; // 50cm to be considered too close
  private static final float   TOO_CLOSE_SIDE_DISTANCE = 0.30f; // 25cm to be considered too close
  private static final int     MAX_SPEED    = 12;
  private static final int     WORLD_WIDTH  = 1000;
  private static final int     WORLD_HEIGHT = 1000;
  
  private static Supervisor  robot;
  private static Compass     compass;
  private static Motor       leftMotor;
  private static Motor       rightMotor;
  
  private static Field       TranslationField;
  
  // The app for displaying the lidar data
  private static LidarDisplayApp  displayApp;

  // Read the compass
  private static int getCompassReadingInDegrees() {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI/2) / Math.PI * 180.0;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }
  
  // This method turns away
  private static void turnAway(int leftSpeed, int rightSpeed) {
    leftMotor.setVelocity(leftSpeed);
    rightMotor.setVelocity(rightSpeed);
  }
  
  // This method moves from point (x1,y1) to point (x2, y2) with a slight curve
  private static void moveFrom(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double turn = Math.atan2(yDiff, xDiff) * 180 / Math.PI;
    turn = (turn - getCompassReadingInDegrees()) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;

    if (turn > 0) {
      leftMotor.setVelocity((int)(MAX_SPEED*((90-turn)/90.0)*0.85));
      rightMotor.setVelocity(MAX_SPEED);
    }
    else {//if (turn < -10) {
      leftMotor.setVelocity(MAX_SPEED);
      rightMotor.setVelocity((int)(MAX_SPEED*((90+turn)/90.0)*0.85));
    }
  }

  private static double sin(double angle) {
    return Math.sin(Math.toRadians(angle));
  }

  private static double cos(double angle) {
    return Math.cos(Math.toRadians(angle));
  }
	  
  
  // This is where it all begins
  public static void main(String[] args) {
    // create the Robot instance.
    robot = new Supervisor();
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");
    
    // get the time step of the current world.
    int TimeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the display app for the lidar
    displayApp = new LidarDisplayApp(500, 500, robot.getDisplay("display"));

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(TimeStep);
    
    //  Prepare the Lidar sensor
    Lidar lidar = new Lidar("Sick LMS 291");
    lidar.enable(TimeStep);

    //int lidarWidth = lidar.getHorizontalResolution();
    //int lidarLayers = lidar.getNumberOfLayers();
    //double maxRange = lidar.getMaxRange();
    
    /*System.out.println("Horizontal Field of View = " + lidar.getFov()*180/Math.PI); // 180
    System.out.println("Horizontal Resolution = " + lidarWidth); // 180
    System.out.println("Number of Layers = " + lidarLayers);     // 1
    System.out.println("Maximum Range = " + maxRange);           // 80*/
    
    float lidarValues[] = null;
    
    // Run the robot
    while (robot.step(TimeStep) != -1) {
      // Get the actual robot location
      double values[] = TranslationField.getSFVec3f();
      int x = (int)(values[0]*100) + WORLD_WIDTH/2;
      int y = -(int)(values[2]*100) + WORLD_HEIGHT/2;
      lidarValues = lidar.getRangeImage();
      displayApp.applyLidarReading(lidarValues, x, y, getCompassReadingInDegrees());
      
      
      // WRITE YOUR CODE HERE
      final int ADJACENCY_TOLERANCE = 15; //cm
      int leftX, leftY, rightX, rightY;
      leftX = leftY = rightX = rightY = -1;

      // Find left doorway gap
      for (int i = 0; i < lidarValues.length - 1; i++) {
        // System.out.println(Math.abs(lidarValues[i] - lidarValues[i + 1]) + " : " + (Math.abs(lidarValues[i] - lidarValues[i + 1]) > ADJACENCY_TOLERANCE));

        double rangeCM = lidarValues[i] * 100;
        double nextRangeCM = lidarValues[i + 1] * 100;

        if (Math.abs(rangeCM - nextRangeCM) > ADJACENCY_TOLERANCE) {
          int leftDoorwayAngle = getCompassReadingInDegrees() + 90 - i;

          leftX = (int)(rangeCM * cos(leftDoorwayAngle));
          leftY = (int)(rangeCM * sin(leftDoorwayAngle));

          break;
        }
      }

      // Find right doorway gap
      for (int i = lidarValues.length - 1; i > 0; i--) {
        double rangeCM = lidarValues[i] * 100;
        double prevRangeCM = lidarValues[i - 1] * 100;

        if (Math.abs(rangeCM - prevRangeCM) > ADJACENCY_TOLERANCE) {
          int rightDoorwayAngle = getCompassReadingInDegrees() + 90 - i;

          rightX = (int)(rangeCM * cos(rightDoorwayAngle));
          rightY = (int)(rangeCM * sin(rightDoorwayAngle));

          break;
        }
      }

      if (leftX == -1 || leftY == -1 || rightX == -1 || rightY == -1) {
        displayApp.setDoorwayPoints(0,0,0,0, x, y);
      } else {
        displayApp.setDoorwayPoints(leftX, leftY, rightX, rightY, x, y);
      }
      
    }
  }
}
