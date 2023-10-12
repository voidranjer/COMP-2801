// James Yap 101276054
// Braeden Hall 101143403
// Room 36

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Device;
import com.cyberbotics.webots.controller.Display;

public class Lab10Controller {

  static final double MAX_SPEED = 19; // 19.1 is maximum but value of 1 is required for simulator to work properly for
                                      // kinematics
  static final double WHEEL_RADIUS = 2.12; // cm
  static final double WHEEL_BASE = 9.356; // cm

  static final double TOO_CLOSE_THRESHOLD = 0.03; // 3cm is considered too close for comfort.

  // Various modes that the robot may be in
  static final byte STRAIGHT = 0;
  static final byte SPIN_LEFT = 1;
  static final byte SPIN_RIGHT = 2;
  static final byte CURVE_LEFT = 3;
  static final byte CURVE_RIGHT = 4;

  // Store the (x, y) location amd angle (degrees) estimate
  static double X, Y, A, R, TD;

  // Encoder variables
  static PositionSensor LeftEncoder;
  static PositionSensor RightEncoder;
  static double LeftReading, RightReading, PreviousLeft, PreviousRight;
  static DistanceSensor rangeSensors[];

  // Read the compass
  private static int getCompassReadingInDegrees(Compass compass) {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI / 2) / Math.PI * 180.0;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int) (bearing);
  }

  private static double sin(double degrees) {
    return Math.sin(Math.toRadians(degrees));
  }

  private static double cos(double degrees) {
    return Math.cos(Math.toRadians(degrees));
  }

  private static void setEstimatedA(double LeftReading, double RightReading) {
    A = A + ((RightReading - LeftReading) * WHEEL_RADIUS /
        WHEEL_BASE / Math.PI * 180);
    if (A > 180)
      A = A - 360;
    if (A < -180)
      A = 360 + A;
  }

  // Add a map point from each sensor as readings from the given position
  static void addSensorReadingsToMap(MapperApp mapper, Field translationField, double x, double y, double a) {
    // Use the actual location. Comment out these three lines when using estimated
    // position
    double values[] = translationField.getSFVec3f();
    x = (values[0] * 100);
    // y = -(values[2] * 100);
    y = -(values[2] * 100) + 15;

    // Offsets for all IR sensors (indices 0-8)
    float xOffsets[] = { -4.038f, 1.481f, 4.885f, 6.560f, 6.560f, 4.885f, 1.481f, -4.038f, -5.502f };
    float yOffsets[] = { 4.956f, 6.296f, 4.538f, 1.552f, -1.552f, -4.538f, -6.296f, -4.956f, 0.000f };
    int aOffsets[] = { 140, 76, 43, 13, -13, -43, -76, -140, 180 };

    // ADD YOUR CODE HERE TO READ ALL 9 SENSORS, COMPUTE THE OBJECT LOCATIONS,
    // AND APPLY THEM TO THE MAP Y ADDING OBJECT POINTS
    final int numSensors = 9;
    for (int i = 0; i < numSensors; i++) {
      DistanceSensor sensor = rangeSensors[i];
      float xOffset = xOffsets[i];
      float yOffset = yOffsets[i];
      int aOffset = aOffsets[i];
      double d = sensor.getValue() * 100;

      if (d < 10) {
        double xObj = x + (xOffset * cos(a)) - (yOffset * sin(a)) + (d * cos(a + aOffset));
        double yObj = y + (yOffset * cos(a)) + (xOffset * sin(a)) + (d * sin(a + aOffset));
        mapper.addObjectPoint((float) xObj, (float) yObj);
      }

    }

  }

  // Update the estimated position
  static void updateEstimate(byte previousMode, Compass compass) {
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();

    switch (previousMode) {
      case SPIN_LEFT:
      case SPIN_RIGHT:
        A = getCompassReadingInDegrees(compass);
        // setEstimatedA(LeftReading, RightReading);
        break;
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
          // setEstimatedA(LeftReading, RightReading);
        }
        break;
      default:
        X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
        Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
        break;
    }
  }

  public static void main(String[] args) {
    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get the floor dimensions and set up the MapperApp
    Node floor = robot.getFromDef("floor");
    Field fSize = floor.getField("floorSize");
    double[] sizes = fSize.getSFVec2f();
    MapperApp mapper = new MapperApp((int) (sizes[1] * 100), (int) (sizes[0] * 100), robot.getDisplay("display"));

    // Code required for being able to get the robot's location
    Node robotNode = robot.getSelf();
    Field translationField = robotNode.getField("translation");

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // Get and enable the sensors
    rangeSensors = new DistanceSensor[9];
    for (int i = 0; i < 9; i++) {
      rangeSensors[i] = robot.getDistanceSensor("ds" + i);
      rangeSensors[i].enable(timeStep);
    }

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

    // Initialize the logic variable for turning
    byte lastTurn = STRAIGHT;
    byte desiredTurn = STRAIGHT;
    byte turnDecision;
    double leftSpeed, rightSpeed;

    // Set the first estimate to match the current location
    double values[] = translationField.getSFVec3f();
    X = (values[0] * 100);
    Y = -(values[2] * 100); // Need to negate the Y value
    A = 90;
    System.out.printf("Robot starts at (x, y) = (%2.1f, %2.1f, %2.2f degrees)\n", (values[0] * 100), -(values[2] * 100),
        A);
    addSensorReadingsToMap(mapper, translationField, X, Y, A);

    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      boolean leftAheadObj = rangeSensors[3].getValue() < TOO_CLOSE_THRESHOLD;
      boolean leftAngleObj = rangeSensors[2].getValue() < TOO_CLOSE_THRESHOLD;
      boolean rightAheadObj = rangeSensors[4].getValue() < TOO_CLOSE_THRESHOLD;
      boolean rightAngleObj = rangeSensors[5].getValue() < TOO_CLOSE_THRESHOLD;
      boolean leftSideObj = rangeSensors[1].getValue() < TOO_CLOSE_THRESHOLD;
      boolean rightSideObj = rangeSensors[6].getValue() < TOO_CLOSE_THRESHOLD;

      // THINK: Check for obstacle and decide how we need to turn
      switch (desiredTurn) {
        case SPIN_LEFT:
          if (!rightAheadObj && !rightAngleObj && !leftAheadObj && !leftAngleObj)
            desiredTurn = STRAIGHT;
          break;
        case SPIN_RIGHT:
          if (!leftAheadObj && !leftAngleObj && !rightAheadObj && !rightAngleObj)
            desiredTurn = STRAIGHT;
          break;
        case CURVE_LEFT:
          if (!rightSideObj || rightAheadObj || rightAngleObj || leftAheadObj || leftAngleObj)
            desiredTurn = STRAIGHT;
          break;
        case CURVE_RIGHT:
          if (!leftSideObj || rightAheadObj || rightAngleObj || leftAheadObj || leftAngleObj)
            desiredTurn = STRAIGHT;
          break;
        default:
          if (rightAheadObj || leftAheadObj) {
            if ((int) (Math.random() * 2) % 2 == 0)
              desiredTurn = SPIN_LEFT;
            else
              desiredTurn = SPIN_RIGHT;
          } else if (rightAngleObj)
            desiredTurn = SPIN_LEFT;
          else if (leftAngleObj)
            desiredTurn = SPIN_RIGHT;
          else if (leftSideObj && rightSideObj)
            desiredTurn = STRAIGHT;
          else if (rightSideObj)
            desiredTurn = CURVE_LEFT;
          else if (leftSideObj)
            desiredTurn = CURVE_RIGHT;
          break;
      }

      updateEstimate(lastTurn, compass);

      // ADD CODE HERE TO APPLY THE SENSOR READINGS TO THE MAP
      if (desiredTurn != SPIN_LEFT && desiredTurn != SPIN_RIGHT)
        addSensorReadingsToMap(mapper, translationField, X, Y, A);

      lastTurn = desiredTurn;

      // REACT: Move motors accordingly
      switch (desiredTurn) {
        case SPIN_LEFT:
          leftSpeed = -0.5 * MAX_SPEED;
          rightSpeed = 0.5 * MAX_SPEED;
          break;
        case SPIN_RIGHT:
          leftSpeed = 0.5 * MAX_SPEED;
          rightSpeed = -0.5 * MAX_SPEED;
          break;
        case CURVE_LEFT:
          leftSpeed = 0.75 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case CURVE_RIGHT:
          leftSpeed = 1 * MAX_SPEED;
          rightSpeed = 0.75 * MAX_SPEED;
          break;
        default:
          leftSpeed = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
      }
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
}
