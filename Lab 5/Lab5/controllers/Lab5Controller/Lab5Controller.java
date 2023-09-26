// Room 4
// James Yap (101276054)
// Alex Hemphill (101187232)

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;

public class Lab5Controller {

  static final double MAX_SPEED = 0.25; // 6.28 is maximum but value of 1 is required for simulator to work properly for
                                        // kinematics
  static final double WHEEL_RADIUS = 2.05; // cm
  static final double WHEEL_BASE = 5.80; // cm

  static Robot Epuck;
  static Motor LeftMotor;
  static Motor RightMotor;
  static PositionSensor rightEncoder;
  static double PreviousReading = 0;
  static int TimeStep;

  // The locations to visit in sequence, and starting angle and position
  static int x[] = { 0, 30, 30, -10, 10, -10, -60, -50, -40, -30, -20, 0 };
  static int y[] = { 0, 30, 60, 70, 50, 50, 40, 30, 30, 20, 20, 0 };
  static double startAngle = 90;

  // Set each motor to a specific speed and wait until the left sensor reaches
  // the specified number of radians.
  private static void move(double leftSpeed, double rightSpeed, double thisManyRadians) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    while (true) {
      double reading = rightEncoder.getValue() - PreviousReading;
      if ((thisManyRadians > 0) && (reading >= thisManyRadians))
        break;
      if ((thisManyRadians < 0) && (reading <= thisManyRadians))
        break;
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = rightEncoder.getValue();
    Epuck.step(TimeStep);
  }

  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation serv
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);

    // Get the encoders
    rightEncoder = Epuck.getPositionSensor("right wheel sensor");
    rightEncoder.enable(TimeStep);

    // Travel through the points in the array one at a time in sequence
    double newAngle, rightReading, deltaAngle;
    int prevX, prevY, destX, destY;
    prevX = x[0];
    prevY = y[0];
    for (int i = 1; i < x.length; i++) {
      destX = x[i];
      destY = y[i];

      newAngle = Math.toDegrees(Math.atan2((destY - prevY), (destX - prevX)));
      deltaAngle = (newAngle - startAngle) % 360;

      // Spin to target angle
      rightReading = Math.toRadians(deltaAngle) * (WHEEL_BASE / WHEEL_RADIUS / 2);
      if (deltaAngle > 0)
        move(-MAX_SPEED, MAX_SPEED, rightReading); // Spin Left
      else
        move(MAX_SPEED, -MAX_SPEED, rightReading); // Spin Right

      // Move straight to target point
      rightReading = Math.sqrt(Math.pow(destX - prevX, 2) + Math.pow(destY - prevY, 2)) / WHEEL_RADIUS;
      move(MAX_SPEED, MAX_SPEED, rightReading);

      // Store prev values
      startAngle = newAngle;
      prevX = destX;
      prevY = destY;
    }

  }
}
