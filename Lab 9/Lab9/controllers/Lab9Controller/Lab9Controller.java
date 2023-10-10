// James Yap: 101276054
// Mac Zhang: 101185293
// Room 43

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;

public class Lab9Controller {

  // Various modes that the robot may be in
  static final byte STRAIGHT = 0;
  static final byte SPIN_LEFT = 1;
  static final byte PIVOT_RIGHT = 2;
  static final byte CURVE_LEFT = 3;
  static final byte CURVE_RIGHT = 4;
  static final byte HEAD_TO_GOAL = 5;
  static final byte ORIENT_TO_GOAL = 6;
  static final String[] MODES = { "STRAIGHT", "SPIN_LEFT", "PIVOT_RIGHT", "CURVE_LEFT", "CURVE_RIGHT", "HEAD_TO_GOAL",
      "ORIENT_TO_GOAL" };

  static final double MAX_SPEED = 1; // maximum speed of the epuck robot
  static final double WHEEL_RADIUS = 2.05; // cm
  static final double WHEEL_BASE = 5.80; // cm

  static PositionSensor LeftEncoder;
  static PositionSensor RightEncoder;
  static double LeftReading, RightReading, PreviousLeft, PreviousRight;

  private static void println(Object o) {
    System.out.println(o);
  }

  // Store the (x, y) location amd angle (degrees) estimate as well ad radius and
  // theta-delta
  static double X, Y, A, R, TD;
  static double sx, sy;

  // Read the compass
  private static double getCompassReadingInDegrees(Compass compass) {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI / 2) / Math.PI * 180.0;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return bearing;
  }

  // Update the estimated position
  static void updateEstimate(byte previousMode, Compass compass) {
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();

    switch (previousMode) {
      case ORIENT_TO_GOAL:
        // ADD CODE HERE
        A = getCompassReadingInDegrees(compass);
        break;
      case SPIN_LEFT:
        A = getCompassReadingInDegrees(compass);
        break;

      case PIVOT_RIGHT:
      case CURVE_LEFT:
      case CURVE_RIGHT:
        if (RightReading == LeftReading) {
          X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
          Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
        } else {
          R = WHEEL_BASE * (LeftReading / (RightReading - LeftReading)) + WHEEL_BASE / 2;
          TD = (RightReading - LeftReading) * WHEEL_RADIUS / WHEEL_BASE / Math.PI * 180;
          X = X + (R * Math.cos(Math.toRadians(TD)) * Math.sin(Math.toRadians(A))) +
              (R * Math.cos(Math.toRadians(A)) * Math.sin(Math.toRadians(TD))) - (R * Math.sin(Math.toRadians(A)));
          Y = Y + (R * Math.sin(Math.toRadians(TD)) * Math.sin(Math.toRadians(A))) -
              (R * Math.cos(Math.toRadians(A)) * Math.cos(Math.toRadians(TD))) + (R * Math.cos(Math.toRadians(A)));
          A = getCompassReadingInDegrees(compass);
        }
        break;
      case HEAD_TO_GOAL:
        // ADD CODE HERE
        X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
        Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));

        break;
      case STRAIGHT:
        X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
        Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
        break;
    }
  }

  static double getDist(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
  }

  public static void main(String[] args) {
    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Code required for being able to get the robot's location
    Node robotNode = robot.getSelf();
    Field translationField = robotNode.getField("translation");

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    rightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo

    // Get and enable the sensors
    DistanceSensor leftAheadSensor = robot.getDistanceSensor("ps7");
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0");
    DistanceSensor rightAngledSensor = robot.getDistanceSensor("ps1");
    DistanceSensor rightSideSensor = robot.getDistanceSensor("ps2");
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);

    // Get the encoders
    LeftEncoder = robot.getPositionSensor("left wheel sensor");
    RightEncoder = robot.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(timeStep);
    RightEncoder.enable(timeStep);
    PreviousLeft = 0;
    PreviousRight = 0;

    // Get the Compass sensor
    Compass compass = robot.getCompass("compass");
    compass.enable(timeStep);

    // Initialize the logic variables for turning
    byte currentMode = HEAD_TO_GOAL;
    byte previousMode = STRAIGHT;
    double leftSpeed = 0, rightSpeed = 0;
    boolean leftTheLine = false; // Set to true when we start wall-following and are far enough away from the
                                 // start-to-goal line

    // Set the first estimate to match the current location
    double values[] = translationField.getSFVec3f();
    X = (values[0] * 100);
    Y = -(values[2] * 100); // Need to negate the Y value
    A = 0;
    System.out.printf("Robot starts at (x, y) = (%2.1f, %2.1f, %2.0f degrees)\n", (values[0] * 100), -(values[2] * 100),
        A);
    boolean PrintState = true;

    // My vars
    sx = X;
    sy = Y;
    int timestepSinceLeftLine = 0;
    boolean isCloserToGoal = false;
    boolean isOnTheLineNow = false;
    final int goalX = 170;
    final int goalY = 0;
    final int DIST_THRESH = 5;

    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      boolean sideTooClose = rightSideSensor.getValue() > 300;
      boolean sideTooFar = rightSideSensor.getValue() < 81;
      boolean frontTooClose = (rightAheadSensor.getValue() > 81) || (leftAheadSensor.getValue() > 81)
          || (rightAngledSensor.getValue() > 81);
      boolean lostContact = rightSideSensor.getValue() < 81;

      // THINK: Check for obstacle and decide how we need to turn
      switch (currentMode) {
        case STRAIGHT: // if (PrintState) System.out.printf("STRAIGHT\n");
          if (lostContact) {
            currentMode = PIVOT_RIGHT;
            break;
          }
          if (sideTooFar) {
            currentMode = CURVE_RIGHT;
            break;
          }
          if (sideTooClose) {
            currentMode = CURVE_LEFT;
            break;
          }
          if (frontTooClose) {
            currentMode = SPIN_LEFT;
            break;
          }
          // ADD CODE HERE
          final int TIMESTEPS_THRESHOLD = 10;

          if (Math.abs(Y) > DIST_THRESH)
            timestepSinceLeftLine++;
          else
            timestepSinceLeftLine = 0;

          if (!leftTheLine)
            leftTheLine = (timestepSinceLeftLine > TIMESTEPS_THRESHOLD);
          isCloserToGoal = (getDist(X, Y, goalX, goalY) < getDist(sx, sy, goalX, goalY));
          isOnTheLineNow = (Math.abs(Y) < DIST_THRESH);

          if (leftTheLine && isCloserToGoal && isOnTheLineNow) {
            currentMode = ORIENT_TO_GOAL;
            break;
          }

          break;
        case CURVE_RIGHT: // if (PrintState) System.out.printf("CURVE RIGHT\n");
          if (!sideTooFar) {
            currentMode = STRAIGHT;
            break;
          }
          if (sideTooClose) {
            currentMode = CURVE_LEFT;
            break;
          }
          if (frontTooClose)
            currentMode = SPIN_LEFT;
          break;
        case PIVOT_RIGHT: // if (PrintState) System.out.printf("PIVOT RIGHT\n");
          if (sideTooClose) {
            currentMode = CURVE_LEFT;
            break;
          }
          if (frontTooClose)
            currentMode = SPIN_LEFT;
          break;
        case SPIN_LEFT: // if (PrintState) System.out.printf("SPIN LEFT\n");
          if (!frontTooClose)
            currentMode = STRAIGHT;
          break;
        case CURVE_LEFT: // if (PrintState) System.out.printf("CURVE LEFT\n");
          if (!sideTooClose)
            currentMode = STRAIGHT;
          break;
        case HEAD_TO_GOAL: // if (PrintState) System.out.printf("HEAD_TO_GOAL\n");
          // ADD CODE HERE
          if (frontTooClose) {
            leftTheLine = false;
            sx = X;
            sy = Y;
            currentMode = SPIN_LEFT;
          }
          break;
        case ORIENT_TO_GOAL:// if (PrintState) System.out.printf("ORIENT TO GOAL\n");
          // ADD CODE HERE
          double bearing = getCompassReadingInDegrees(compass);
          final int ANGLE_THRESH = 2;

          // If bearing is within +/- of ANGLE_THRESH
          boolean isFacingGoal = (Math.abs(bearing) < ANGLE_THRESH);

          if (isFacingGoal && leftTheLine)
            currentMode = HEAD_TO_GOAL;

          break;
      }
      updateEstimate(previousMode, compass);
      PrintState = previousMode != currentMode;
      previousMode = currentMode;

      // ADD CODE HERE
      // Arrived at goal
      if (getDist(X, Y, goalX, goalY) < DIST_THRESH) {
        robot.step(timeStep);
        break;
      }
      System.out.printf("(%d, %d) %d\n", (int) X, (int) Y, (int) getDist(X, Y, goalX, goalY));

      // REACT: Move motors accordingly
      switch (currentMode) {
        case ORIENT_TO_GOAL:

          // ADD CODE HERE
          double bearing = getCompassReadingInDegrees(compass);

          // If (bearing < 180), spin left
          if (bearing < 180) {
            leftSpeed = -1 * MAX_SPEED;
            rightSpeed = 1 * MAX_SPEED;
          }

          // If (bearing > 180), spin right
          else if (bearing > 180) {
            leftSpeed = 1 * MAX_SPEED;
            rightSpeed = -1 * MAX_SPEED;
          }

          break;
        case SPIN_LEFT:
          leftSpeed = -1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case PIVOT_RIGHT:
          leftSpeed = 1 * MAX_SPEED;
          rightSpeed = 0.25 * MAX_SPEED;
          break;
        case CURVE_LEFT:
          leftSpeed = 0.9 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case CURVE_RIGHT:
          leftSpeed = 1 * MAX_SPEED;
          rightSpeed = 0.9 * MAX_SPEED;
          break;
        case HEAD_TO_GOAL:
          // ADD CODE HERE
          leftSpeed = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case STRAIGHT:
          leftSpeed = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
      }
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
}
