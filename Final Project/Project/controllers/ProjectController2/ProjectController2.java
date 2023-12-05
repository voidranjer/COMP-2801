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

  // #region
  private static final byte CAMERA_WIDTH = 64;
  private static final byte CAMERA_HEIGHT = 64;
  private static final double GRIPPER_MOTOR_MAX_SPEED = 0.1;

  // CUSTOM
  private static final double MAX_SPEED = 12.3;
  private static final double COMPASS_THRESH = 3; // degrees
  private static final double CAM_THRESH = 10;

  // Various modes for the robot to be in
  private static final byte STRAIGHT = 0;
  private static final byte SPIN_LEFT = 1;
  private static final byte SPIN_RIGHT = 2;
  private static final byte CURVE_LEFT = 3;
  private static final byte CURVE_RIGHT = 4;
  private static final byte PIVOT_LEFT = 5;
  private static final byte PIVOT_RIGHT = 6;
  private static final byte STOP = 7;
  private static final byte STRAIGHT_SLOW = 8;
  private static final byte LIDAR = 9;
  private static final String[] MODE_NAMES = { "STRAIGHT", "SPIN_LEFT", "SPIN_RIGHT", "CURVE_LEFT", "CURVE_RIGHT",
      "PIVOT_LEFT", "PIVOT_RIGHT", "STOP", "STRAIGHT_SLOW", "LIDAR" };

  private static Robot robot;
  private static Motor leftMotor;
  private static Motor rightMotor;
  private static Motor gripperLift;
  private static Motor gripperLeftSide;
  private static Motor gripperRightSide;
  private static DistanceSensor leftSideSensor;
  private static DistanceSensor rightSideSensor;
  private static DistanceSensor leftAheadSensor;
  private static DistanceSensor rightAheadSensor;
  private static DistanceSensor leftAngledSensor;
  private static DistanceSensor rightAngledSensor;
  private static TouchSensor jarDetectedSensor;
  private static Compass compass;
  private static Accelerometer accelerometer;
  private static Camera camera;

  private static byte currentMode = LIDAR;
  private static double forwardDistance;
  private static int timeStep = 0;
  private static int step = 0;
  private static float lidarValues[] = null;

  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds) {
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

  // This method checks whether distance between two doorway points is ≥ 70cm and
  // ≤ 110cm
  private static boolean isDoorway(int x1, int y1, int x2, int y2) {
    double distance = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    return distance >= 70 && distance <= 110;
  }

  // Read the compass
  private static int getCompassReadingInDegrees() {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI / 2) / Math.PI * 180.0;
    // if (bearing > 360)
    // bearing = 360 - bearing;
    // if (bearing < 0)
    // bearing = 360 + bearing;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int) (bearing);
  }

  private static byte alignTo(int bearing) {
    int currentBearing = getCompassReadingInDegrees();
    if (currentBearing < bearing - COMPASS_THRESH) {
      return SPIN_LEFT;
    }
    if (currentBearing > bearing + COMPASS_THRESH) {
      return SPIN_RIGHT;
    }
    return STOP;
  }

  private static boolean bearingPasses(int bearing) {
    int currentBearing = getCompassReadingInDegrees();
    return Math.abs(currentBearing - bearing) <= COMPASS_THRESH;
  }

  private static boolean isGreen(int red, int green, int blue) {
    return (red < 150) && (green > 100) && (blue < 150);
  }

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
      leftMotor.setVelocity((int) (MAX_SPEED * ((90 - turn) / 90.0) * 0.85));
      rightMotor.setVelocity(MAX_SPEED);
    } else {// if (turn < -10) {
      leftMotor.setVelocity(MAX_SPEED);
      rightMotor.setVelocity((int) (MAX_SPEED * ((90 + turn) / 90.0) * 0.85));
    }
  }

  // #endregion

  // This is where it all begins
  public static void main(String[] args) {
    // #region
    robot = new Robot();
    timeStep = (int) Math.round(robot.getBasicTimeStep());

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

    // Prepare the Lidar sensor
    Lidar lidar = new Lidar("Sick LMS 291");
    lidar.enable(timeStep);
    // #endregion

    // Run the robot
    openCloseGripper(0.099f);
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors
      lidarValues = lidar.getRangeImage();

      // #region
      // THINK: Make a decision as to what MODE to be in
      // switch (step) {
      // case 0: // Go to bottom
      // currentMode = hallwayStep(270, 100, 1);
      // break;
      // case 1: // Turn towards window
      // currentMode = hallwayStep(190, Integer.MAX_VALUE, 2);
      // break;
      // case 2: // Home into jar and pick up jar
      // currentMode = step2();
      // break;
      // case 3: // Reverse gear back to bottom center position
      // currentMode = step3();
      // break;
      // case 4: // First hallway
      // currentMode = hallwayStep(90, 150, 5);
      // break;
      // case 5: // second hallway
      // currentMode = hallwayStep(1, 100, 6);
      // break;
      // case 6: // third hallway
      // currentMode = hallwayStep(270, 120, 7);
      // break;
      // case 7: // fourth hallway
      // currentMode = hallwayStep(1, 100, 8);
      // break;

      // }
      // #endregion

      // REACT: Move motors accordingly
      System.out.println(MODE_NAMES[currentMode]);
      switch (currentMode) {
        case STRAIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case SPIN_LEFT:
          leftMotor.setVelocity(-1 * MAX_SPEED * 0.1);
          rightMotor.setVelocity(MAX_SPEED * 0.1);
          break;
        case SPIN_RIGHT:
          leftMotor.setVelocity(MAX_SPEED * 0.1);
          rightMotor.setVelocity(-1 * MAX_SPEED * 0.1);
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
          leftMotor.setVelocity(0.75 * MAX_SPEED * 0.1);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case CURVE_RIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(0.75 * MAX_SPEED * 0.1);
          break;
        case STRAIGHT_SLOW:
          leftMotor.setVelocity(MAX_SPEED * 0.1);
          rightMotor.setVelocity(MAX_SPEED * 0.1);
          break;
        case STOP:
          leftMotor.setVelocity(0);
          rightMotor.setVelocity(0);
          break;
        case LIDAR:
          lidarGuided();
          break;
        default:
          break;
      }
    }
  }

  // #region
  private static byte lidarGuided() {
    double centerX, centerY;
    centerX = centerY = 0;
    for (int i = 0; i < lidarValues.length; i++) {
      double d = lidarValues[i] * 100;
      double px = d * cos(getCompassReadingInDegrees() + 90 - i);
      double py = d * sin(getCompassReadingInDegrees() + 90 - i);
      centerX += px;
      centerY += py;
    }
    centerX /= lidarValues.length;
    centerY /= lidarValues.length;
    moveFrom(0, 0, centerX, centerY);
    return LIDAR;
  }

  private static byte step0() {
    openCloseGripper(0.099f);

    if (!bearingPasses(270))
      return alignTo(270);

    if (forwardDistance <= 100) {
      step = 1;
      return STOP;
    }
    return STRAIGHT;
  }

  private static byte hallwayStep(int bearing, int distance, int nextStep) {
    if (!bearingPasses(bearing))
      return alignTo(bearing);
    if (forwardDistance <= distance) {
      step = nextStep;
      return STOP;
    }
    return STRAIGHT;
  }

  private static byte step2() {
    int[] image = camera.getImage();

    int leftCount = 0;
    int rightCount = 0;
    int centerCount = 0;
    int r, g, b = 0;

    int Y_SCAN_OFFSET = 15;
    // int yLvl = CAMERA_HEIGHT / 2;
    for (int yLvl = CAMERA_HEIGHT / 2 - Y_SCAN_OFFSET; yLvl < CAMERA_HEIGHT / 2 +
        Y_SCAN_OFFSET; yLvl++) {
      for (int x = 0; x < CAMERA_WIDTH; x++) {
        r = Camera.imageGetRed(image, CAMERA_WIDTH, x, yLvl);
        g = Camera.imageGetGreen(image, CAMERA_WIDTH, x, yLvl);
        b = Camera.imageGetBlue(image, CAMERA_WIDTH, x, yLvl);
        if (isGreen(r, g, b)) {
          if (x < CAMERA_WIDTH / 3)
            leftCount++;
          else if (x > CAMERA_WIDTH * 2 / 3)
            rightCount++;
          else
            centerCount++;
        }
      }
    }

    boolean detectedLeft = leftCount > (rightCount + CAM_THRESH);
    boolean detectedRight = rightCount > (leftCount + CAM_THRESH);
    boolean detectedCenter = centerCount > 350;
    // boolean detectedCenter = centerCount > (leftCount + CAM_THRESH) &&
    // centerCount > (rightCount + CAM_THRESH);
    boolean notDetected = !detectedLeft && !detectedCenter && !detectedRight;

    if (jarDetectedSensor.getValue() == 1) {
      openCloseGripper(0.01f);
      delay(1000);
      liftLowerGripper(-0.025f);
      delay(1000);
      step = 3;
      return STOP;
    }

    if (detectedLeft)
      return CURVE_LEFT;
    if (detectedRight)
      return CURVE_RIGHT;
    if (detectedCenter || notDetected)
      return STRAIGHT_SLOW;

    return STOP;
  }

  private static byte step3() {
    if (!bearingPasses(180))
      return alignTo(180);

    // Reverse
    leftMotor.setVelocity(-1 * MAX_SPEED * 0.3);
    rightMotor.setVelocity(-1 * MAX_SPEED * 0.3);
    delay(2000);

    step = 4;
    return STOP;
  }
  // #endregion
}
