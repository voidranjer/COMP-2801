// Author: Mark Lanthier (SN:100000001)

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab3Controller {

  static final byte CAMERA_WIDTH = 52;
  static final byte CAMERA_HEIGHT = 39;

  static final byte WANDER = 0;
  static final byte HOME_IN = 1;
  static final byte PUSH_BALL = 2;
  static final byte TURN_AROUND = 3;
  static final byte GO_FORWARD = 4;
  static final byte AVOID = 5;

  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte NONE = 2;
  static final byte FULL_SPEED = 6;

  public static void main(String[] args) {
    Robot robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // Get and enable the distance sensors
    DistanceSensor leftAheadSensor = robot.getDistanceSensor("ps7");
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0");
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);

    // Get and enable the accelerometer
    Accelerometer accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);

    // Set up the camera
    Camera camera = new Camera("camera");
    camera.enable(timeStep);

    int leftSpeed, rightSpeed;
    leftSpeed = rightSpeed = 0;

    byte currentMode = WANDER;
    byte turnCount = 0;
    byte turnDirection = NONE;

    byte accelIndex = 0;
    double accelValues[][] = new double[10][3];
    double accelTemp[] = new double[3];

    // Thresh
    double ACCEL_THRESH = 2.5;

    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors
      accelValues[accelIndex] = accelerometer.getValues();
      accelIndex = (byte) ((accelIndex + 1) % 10);

      // Total up the past accel values
      for (int j = 0; j < 3; j++)
        accelTemp[j] = 0;
      for (int i = 0; i < 10; i++)
        for (int j = 0; j < 3; j++)
          accelTemp[j] += accelValues[i][j];
      for (int j = 0; j < 3; j++)
        accelTemp[j] = accelTemp[j] / 10.0;

      boolean tippedForward = accelTemp[0] < 0.0 - ACCEL_THRESH;
      boolean tippedBackward = accelTemp[0] > 0.0 + ACCEL_THRESH;
      boolean tippedSideways = Math.abs(accelTemp[1]) > ACCEL_THRESH;
      boolean flat = Math.abs(accelTemp[0]) < ACCEL_THRESH && Math.abs(accelTemp[1]) < ACCEL_THRESH;

      // THINK: Make a decision as to what MODE to be in
      switch (currentMode) {
        case WANDER:
          if (!flat)
            currentMode = TURN_AROUND;
          break;
        case HOME_IN:
          break;
        case PUSH_BALL:
          break;
        case TURN_AROUND:
          if (tippedBackward && !tippedForward && !tippedSideways)
            currentMode = GO_FORWARD;
          break;
        case GO_FORWARD:
          if (!tippedBackward && !flat)
            currentMode = TURN_AROUND;
          break;
        case AVOID:
          break;
      }

      // REACT: Move motors according to the MODE
      leftSpeed = FULL_SPEED;
      rightSpeed = FULL_SPEED;
      switch (currentMode) {
        case WANDER:
          if (turnCount > 0) {
            if (turnDirection == LEFT)
              leftSpeed -= 2;
            else
              rightSpeed -= 2;
            turnCount--;
            if (--turnCount == 0)
              turnDirection = NONE;
          } else {
            if ((byte) (Math.random() * 5) == 0) {// turn 20% of the time
              turnDirection = (byte) (Math.random() * 2);
              turnCount = (byte) (Math.random() * 51 + 25);
            }
          }
          break;
        case HOME_IN:
          turnCount = 0;
          break;
        case PUSH_BALL:
          turnCount = 0;
          break;
        case TURN_AROUND:
          turnCount = 0;
          leftSpeed = -FULL_SPEED;
          rightSpeed = FULL_SPEED;
          break;
        case GO_FORWARD:
          turnCount = 0;
          leftSpeed = rightSpeed = FULL_SPEED;
          break;
        case AVOID:
          turnCount = 0;
      }

      System.out.println("AccelTemp: " + accelTemp[0] + ", " + accelTemp[1] + ", "
          + accelTemp[2]);
      System.out.println("Tipped forward: " + tippedForward);
      System.out.println("Tipped backward: " + tippedBackward);
      System.out.println("Tipped sideways: " + tippedSideways);
      System.out.println("Flat: " + flat);
      // leftSpeed = rightSpeed = 0;
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
}
