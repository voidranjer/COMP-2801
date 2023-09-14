// Author: James Yap (SN: 101276054)
// Caleb Aitken (SN: 101189835)
// Room: 24

import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab1Controller {

  // Various modes that the robot may be in
  static final byte STRAIGHT = 0;
  static final byte SPIN_LEFT = 1;
  static final byte SPIN_RIGHT = 2;
  static final byte CURVE_LEFT = 3;
  static final byte CURVE_RIGHT = 4;

  static final double MAX_SPEED = 6.28; // maximum speed of the epuck robot
  static final int THRESH = 100; // threshold for distance sensor

  public static void main(String[] args) {
    byte desiredTurn = STRAIGHT;

    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    rightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo

    // Get and enable the sensors
    DistanceSensor leftAheadSensor = robot.getDistanceSensor("ps7");
    DistanceSensor leftAngledSensor = robot.getDistanceSensor("ps6");
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0");
    DistanceSensor rightAngledSensor = robot.getDistanceSensor("ps1");
    DistanceSensor leftSideSensor = robot.getDistanceSensor("ps5");
    DistanceSensor rightSideSensor = robot.getDistanceSensor("ps2");
    leftAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);

    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      double lf = leftAheadSensor.getValue();
      double rf = rightAheadSensor.getValue();
      double la = leftAngledSensor.getValue();
      double ra = rightAngledSensor.getValue();
      double ls = leftSideSensor.getValue();
      double rs = leftSideSensor.getValue();

      boolean lfDetected = leftAheadSensor.getValue() > THRESH;
      boolean rfDetected = rightAheadSensor.getValue() > THRESH;
      boolean laDetected = leftAngledSensor.getValue() > THRESH;
      boolean raDetected = rightAngledSensor.getValue() > THRESH;
      boolean lsDetected = leftSideSensor.getValue() > THRESH;
      boolean rsDetected = leftSideSensor.getValue() > THRESH;

      boolean shouldExitSpin = !laDetected && !lfDetected && !raDetected && !rfDetected;
      boolean shouldExitCurve = laDetected || lfDetected || rfDetected || raDetected;

      // THINK: Check for obstacle and decide how we need to turn
      switch (desiredTurn) {
        case SPIN_LEFT:
          if (shouldExitSpin)
            desiredTurn = STRAIGHT;
          break;
        case SPIN_RIGHT:
          if (shouldExitSpin)
            desiredTurn = STRAIGHT;
          break;
        case CURVE_LEFT:
          if (!rsDetected || shouldExitCurve)
            desiredTurn = STRAIGHT;
          break;
        case CURVE_RIGHT:
          if (!lsDetected || shouldExitCurve)
            desiredTurn = STRAIGHT;
          break;
        default:

          // Should SPIN_LEFT or SPIN_RIGHT randomly
          if (lfDetected || rfDetected) {
            boolean leftOrRight = Math.random() < 0.5;
            desiredTurn = leftOrRight ? SPIN_LEFT : SPIN_RIGHT;
          }

          else if (lsDetected && rsDetected)
            desiredTurn = STRAIGHT;
          else if (laDetected)
            desiredTurn = SPIN_RIGHT;
          else if (raDetected)
            desiredTurn = SPIN_LEFT;
          else if (lsDetected)
            desiredTurn = CURVE_LEFT;
          else if (rsDetected)
            desiredTurn = CURVE_RIGHT;
          break;
      }

      // REACT: Set motor speeds according to the current state
      switch (desiredTurn) {
        case SPIN_LEFT:
          leftMotor.setVelocity(-1 * MAX_SPEED * 0.5);
          rightMotor.setVelocity(MAX_SPEED * 0.5);
          break;
        case SPIN_RIGHT:
          leftMotor.setVelocity(MAX_SPEED * 0.5);
          rightMotor.setVelocity(-1 * MAX_SPEED * 0.5);
          break;
        case CURVE_LEFT:
          leftMotor.setVelocity(MAX_SPEED * 0.75);
          rightMotor.setVelocity(MAX_SPEED);
          break;
        case CURVE_RIGHT:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED * 0.75);
          break;
        default:
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
      }
    }
  }
}
