// Author: Mark Lanthier (SN:100000001)
//Room: 41
//Author: Kuba Potera (SN:101115432)
//Author: Leah Durham (SN:101192067)

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

    // Initialize the logic variable for turning
    byte currentMode = STRAIGHT;
    double leftSpeed, rightSpeed;

    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      // THINK: Check for obstacle and decide how we need to turn
      // THINK: Check for obstacle and decide how we need to turn
      // System.out.println("RS value: "+rightSideSensor.getValue());
      switch (currentMode) {
        case STRAIGHT:
          // System.out.println("STRAIGHT");
          if ((rightSideSensor.getValue() > 70) && (rightSideSensor.getValue() < 90)) {
            currentMode = CURVE_RIGHT;
          } else if ((rightSideSensor.getValue() > 80) || (rightAngledSensor.getValue() > 80)) {
            currentMode = CURVE_LEFT;
          }
          if ((rightAheadSensor.getValue() > 80) || leftAheadSensor.getValue() > 80) {
            currentMode = SPIN_LEFT;
          } else if (rightSideSensor.getValue() < 70) {
            currentMode = PIVOT_RIGHT;
          }
          // check side if too close/far
          // too close curve left, too far curve right
          // check in front for obsticles, spin left

          break;
        case CURVE_RIGHT:
          // System.out.println("CURVE RIGHT");
          // check if close enough to go straight
          if (rightSideSensor.getValue() < 80) {
            currentMode = STRAIGHT;
          } else if ((rightAheadSensor.getValue() > 80) || (leftAheadSensor.getValue() > 80)) {
            currentMode = SPIN_LEFT;
          }
          break;
        case PIVOT_RIGHT:
          // System.out.println("PIVOT RIGHT");
          if (rightAngledSensor.getValue() > 60) {
            currentMode = STRAIGHT;
          } else if (leftAheadSensor.getValue() > 80) {
            currentMode = SPIN_LEFT;
          }
          break;
        case SPIN_LEFT:
          // System.out.println("SPIN LEFT");
          // check if
          if (rightAheadSensor.getValue() < 90) {
            currentMode = STRAIGHT;
          }
          break;
        case CURVE_LEFT:
          // System.out.println("CURVE LEFT");
          if (rightSideSensor.getValue() < 80) {
            currentMode = STRAIGHT;
          }
          if ((leftAheadSensor.getValue() > 90) || (rightAheadSensor.getValue() > 90)) {
            currentMode = SPIN_LEFT;
          }
          break;
      }

      // REACT: Move motors accordingly
      switch (currentMode) {
        case SPIN_LEFT:
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
        default: // This handles the STRAIGHT case
          leftMotor.setVelocity(MAX_SPEED);
          rightMotor.setVelocity(MAX_SPEED);
          break;
      }
    }
  }
}
