// Author: Mark Lanthier (SN:100000001)

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab2Controller {

  // Various modes that the robot may be in
  static final byte STRAIGHT = 0;
  static final byte SPIN_LEFT = 1;
  static final byte PIVOT_RIGHT = 2;
  static final byte CURVE_LEFT = 3;
  static final byte CURVE_RIGHT = 4;

  static final double MAX_SPEED = 5; // maximum speed of the epuck robot

  // Inclusive
  public static boolean inRangeOf(int lowerBound, int upperBound, int value) {
    return lowerBound <= value && value <= upperBound;
  }

  // Inclusive (0 = too low, 1 = too high, 2 = just right)
  public static int rangeCompare(int lowerBound, int upperBound, int value) {
    if (value < lowerBound)
      return 0;
    if (value > upperBound)
      return 1;
    return 2;
  }

  public static void main(String[] args) {

    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

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

    final int FAR_THRESH = 120;
    final int CLOSE_THRESH = 140;

    // Initialize the logic variable for turning
    byte currentMode = STRAIGHT;
    double leftSpeed, rightSpeed;

    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors

      double rightSideValue = rightSideSensor.getValue();
      double rightAngledValue = rightAngledSensor.getValue();
      double rightAheadValue = rightAheadSensor.getValue();
      double leftAheadValue = leftAheadSensor.getValue();

      boolean rightSideCollide = rightSideValue > CLOSE_THRESH;
      boolean rightAngledCollide = rightAngledValue > CLOSE_THRESH;
      boolean rightAheadCollide = rightAheadValue > CLOSE_THRESH;
      boolean leftAheadCollide = leftAheadValue > CLOSE_THRESH;

      boolean rightSideDetach = rightSideValue < FAR_THRESH;

      boolean frontCollide = leftAheadCollide && rightAheadCollide;

      // THINK: Check for obstacle and decide how we need to turn
      switch (currentMode) {
        case STRAIGHT:
          System.out.println("STRAIGHT");

          if (frontCollide)
            currentMode = SPIN_LEFT;

          else if (rightSideCollide)
            currentMode = CURVE_LEFT;

          else if (rightSideDetach)
            currentMode = CURVE_RIGHT;

          break;
        case CURVE_RIGHT:
          System.out.println("CURVE RIGHT");
          if (rightAngledCollide || rightSideCollide)
            currentMode = SPIN_LEFT;
          break;
        case PIVOT_RIGHT:
          System.out.println("PIVOT RIGHT");
          break;
        case SPIN_LEFT:
          System.out.println("SPIN LEFT");
          if (!rightAngledCollide && !rightAheadCollide && !leftAheadCollide)
            currentMode = STRAIGHT;
          break;
        case CURVE_LEFT:
          System.out.println("CURVE LEFT");
          if (rightSideDetach)
            currentMode = STRAIGHT;
          if (rightSideCollide || rightAngledCollide)
            currentMode = SPIN_LEFT;
          break;
      }

      // Final catch-all
      if (frontCollide)
        currentMode = SPIN_LEFT;

      // REACT: Move motors accordingly
      switch (currentMode) {
        case SPIN_LEFT:
          leftMotor.setVelocity(-1 * MAX_SPEED * 0.5);
          rightMotor.setVelocity(MAX_SPEED * 0.5);
          break;
        case PIVOT_RIGHT:
          leftMotor.setVelocity(MAX_SPEED * 0.5);
          rightMotor.setVelocity(0);
          break;
        case CURVE_LEFT:
          leftMotor.setVelocity(MAX_SPEED * 0.75);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case CURVE_RIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED * 0.75);
          break;
        default: // This handles the STRAIGHT case
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
      }
    }
  }
}
