// James Yap [101276054]

import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Lidar;

public class ProjectController2 {

  private static final byte     CAMERA_WIDTH = 64;
  private static final byte     CAMERA_HEIGHT = 64;
  private static final double   GRIPPER_MOTOR_MAX_SPEED = 0.1;
  private static final double   MAX_SPEED = 12.3; // maximum speed of the epuck robot
  private static final double   COMPASS_THRESH = 1; // degrees
  
  // Various modes for the robot to be in
  private static final byte STRAIGHT  = 0;
  private static final byte SPIN_LEFT  = 1;
  private static final byte SPIN_RIGHT  = 2;
  private static final byte CURVE_LEFT  = 3;
  private static final byte CURVE_RIGHT  = 4;
  private static final byte PIVOT_LEFT  = 5;
  private static final byte PIVOT_RIGHT  = 6;
  private static final byte STOP  = 7;
  private static final String[] MODE_NAMES = {"STRAIGHT", "SPIN_LEFT", "SPIN_RIGHT", "CURVE_LEFT", "CURVE_RIGHT", "PIVOT_LEFT", "PIVOT_RIGHT", "STOP"};
  
  private static Robot           robot;
  private static Motor           leftMotor;
  private static Motor           rightMotor;
  private static Motor           gripperLift;
  private static Motor           gripperLeftSide;
  private static Motor           gripperRightSide;
  private static DistanceSensor  leftSideSensor; 
  private static DistanceSensor  rightSideSensor;
  private static DistanceSensor  leftAheadSensor; 
  private static DistanceSensor  rightAheadSensor;
  private static DistanceSensor  leftAngledSensor; 
  private static DistanceSensor  rightAngledSensor;
  private static TouchSensor     jarDetectedSensor;
  private static Compass         compass;
  private static Accelerometer   accelerometer;
  private static Camera          camera;

  private static byte            currentMode;
  private static double          forwardDistance;
  private static int             step = 0;
  
  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }
  
  // Put the gripper up/down to the given position.
  // -0.025 is "up all the way" and 0.001 is down as much as it can
  public static void liftLowerGripper(float position) {
    gripperLift.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLift.setPosition(position);
  }

  // Put the gripper open/closed to the given position
  // 0.099 is "open all the way" and 0.01 is closed as much as it can
  public static void openCloseGripper(float position) {
    gripperLeftSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperRightSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLeftSide.setPosition(position);
    gripperRightSide.setPosition(position);
  }

  private static double sin(double angle) {
    return Math.sin(Math.toRadians(angle));
  }

  private static double cos(double angle) {
    return Math.cos(Math.toRadians(angle));
  }
	  
  // This method checks whether distance between two doorway points is ≥ 70cm and ≤ 110cm
  private static boolean isDoorway(int x1, int y1, int x2, int y2) {
    double distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    return distance >= 70 && distance <= 110;
  }

  // Read the compass
  private static int getCompassReadingInDegrees() {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI/2) / Math.PI * 180.0;
    if (bearing > 360)
      bearing = 360 - bearing;
    if (bearing < 0)
      bearing = 360 + bearing;
    // if (bearing > 180)
    //   bearing = 360 - bearing;
    // if (bearing < -180)
    //   bearing = 360 + bearing;
    return (int)(bearing);
  }

  private static byte alignTo(int bearing) {
    int currentBearing = getCompassReadingInDegrees();
    if (currentBearing < bearing - COMPASS_THRESH) return SPIN_LEFT;
    if (currentBearing > bearing + COMPASS_THRESH) return SPIN_RIGHT;
    return STOP;
  }

  private static boolean checkBearing(int bearing) {
    int currentBearing = getCompassReadingInDegrees();
    // System.out.println(currentBearing);
    return currentBearing >= bearing - COMPASS_THRESH && currentBearing <= bearing + COMPASS_THRESH;
  }

  private static byte step0() {
    if (!checkBearing(270)) return alignTo(270);
    
    if (forwardDistance <= 100) {
      step = 1;
      return STOP;
    }
    return STRAIGHT;
  }

  private static byte step1() {
    if (!checkBearing(185)) return alignTo(185);

    step = 2;
    return STOP;
  }

  // This is where it all begins
  public static void main(String[] args) {
    robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");
    gripperLeftSide = robot.getMotor("left finger motor");
    gripperRightSide = robot.getMotor("right finger motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Get and enable the distance sensors
    leftSideSensor = robot.getDistanceSensor("so0"); 
    leftAngledSensor = robot.getDistanceSensor("so1"); 
    leftAheadSensor = robot.getDistanceSensor("so3"); 
    rightAheadSensor = robot.getDistanceSensor("so4"); 
    rightAngledSensor = robot.getDistanceSensor("so6"); 
    rightSideSensor = robot.getDistanceSensor("so7"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);
    
    // Prepare the accelerometer
    accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);
    
    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    // Prepare the jar detecting sensor
    jarDetectedSensor = new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    //  Prepare the Lidar sensor
    Lidar lidar = new Lidar("Sick LMS 291");
    lidar.enable(timeStep);
    float lidarValues[] = null;
      
    // Run the robot   
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors
      lidarValues = lidar.getRangeImage();
      forwardDistance = lidarValues[89] * 100;

      // // Lidar
      // final int ADJACENCY_TOLERANCE = 15; //cm
      // int leftDoorwayAngle, rightDoorwayAngle;
      // leftDoorwayAngle = rightDoorwayAngle = -1;

      // // Find left doorway gap
      // for (int i = 0; i < lidarValues.length - 1; i++) {
      //   double rangeCM = lidarValues[i] * 100;
      //   double nextRangeCM = lidarValues[i + 1] * 100;
      //   if (Math.abs(rangeCM - nextRangeCM) > ADJACENCY_TOLERANCE) {
      //     leftDoorwayAngle = getCompassReadingInDegrees() + 90 - i;
      //     break;
      //   }
      // }

      // // Find right doorway gap
      // for (int i = lidarValues.length - 1; i > 0; i--) {
      //   double rangeCM = lidarValues[i] * 100;
      //   double prevRangeCM = lidarValues[i - 1] * 100;
      //   if (Math.abs(rangeCM - prevRangeCM) > ADJACENCY_TOLERANCE) {
      //     rightDoorwayAngle = getCompassReadingInDegrees() + 90 - i;
      //     break;
      //   }
      // }

      // int averageAngle = (leftDoorwayAngle + rightDoorwayAngle) / 2;
      // System.out.println(averageAngle);

      // Find max value of lidar values
      // double max = 0;
      // int maxIndex = 0;
      // for (int i = 0; i < lidarValues.length; i++) {
      //   if (lidarValues[i] > max) {
      //     max = lidarValues[i];
      //     maxIndex = i;
      //   }
      // }
      // System.out.println(maxIndex);

      // final int LOW_THRESH = 80;
      // final int HIGH_THRESH = 100;
      // final int RELEASE = 0;

      // THINK: Make a decision as to what MODE to be in
      switch (step) {
        case 0:
          currentMode = step0();
          break;
        case 1:
          currentMode = step1();
          break;
      }
           
      // REACT: Move motors accordingly
      System.out.println(MODE_NAMES[currentMode]);
      switch (currentMode) {
        case STRAIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case SPIN_LEFT:
          leftMotor.setVelocity(-1 * MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case SPIN_RIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(-1 * MAX_SPEED);
          break;
        case PIVOT_LEFT:
          leftMotor.setVelocity(-1 * MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case PIVOT_RIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(-1 * MAX_SPEED);
          break;
        case CURVE_LEFT:
          leftMotor.setVelocity(0.75 * MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case CURVE_RIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(0.75 * MAX_SPEED);
          break;
        default:
          leftMotor.setVelocity(0);
          rightMotor.setVelocity(0);
          break;
      }
    }
  }
}
