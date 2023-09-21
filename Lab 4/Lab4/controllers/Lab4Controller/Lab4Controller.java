// James Yap: 101276054 (jamesyap@cmail.carleton.ca)
// Ted Liu: 101200401 (tedliu@cmail.carleton.ca)
// Room 4

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;

public class Lab4Controller {

  static final double MAX_SPEED = 1; // 6.28 is maximum but value of 1 is required for simulator to work properly for
                                     // kinematics
  static final double WHEEL_RADIUS = 2.05; // cm
  static final double WHEEL_BASE = 5.80; // cm
  static final double DIFF_THRESH = 0.01; // cm

  static Robot Epuck;
  static Motor LeftMotor;
  static Motor RightMotor;
  static PositionSensor LeftEncoder;
  static PositionSensor RightEncoder;
  static double LeftReading, RightReading, PreviousLeft, PreviousRight;
  static int TimeStep;

  static double x, y;
  static double a = Math.toRadians(90);

  // Set each motor to a specific speed and wait for te given amount of seconds
  // Then stop the motors and update the position sensor readings.
  private static void Move(double leftSpeed, double rightSpeed, double seconds) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    for (double time = 0.0; time < seconds; time += (TimeStep / 1000.0)) {
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();
    Epuck.step(TimeStep);

    // System.out.println("Left Reading = " + LeftReading + " Right Reading = " +
    // RightReading);

    String direction = "";
    String curveInfo = "";
    double alphaTheta = (WHEEL_RADIUS * RightReading - WHEEL_RADIUS * LeftReading) / WHEEL_BASE; // amount of turning in
                                                                                                 // rad

    // Straight
    if (Math.abs(LeftReading - RightReading) < DIFF_THRESH) {
      double d = LeftReading * WHEEL_RADIUS; // Can use either left or right reading because left==right
      x += d * Math.cos(a);
      y += d * Math.sin(a);
      direction = "Straight";
    }

    // Spin
    else if (Math.abs(Math.abs(LeftReading) - Math.abs(RightReading)) < DIFF_THRESH) {
      a += alphaTheta;
      direction = "Spin";
    }

    // Curve
    else {
      double r = WHEEL_BASE
          * (WHEEL_RADIUS * LeftReading / (WHEEL_RADIUS * RightReading - WHEEL_RADIUS * LeftReading) + 0.5);

      x = (r * Math.cos(alphaTheta) * Math.sin(a)) + (r * Math.cos(a) * Math.sin(alphaTheta)) + x - (r * Math.sin(a));
      y = (r * Math.sin(alphaTheta) * Math.sin(a)) - (r * Math.cos(a) * Math.cos(alphaTheta)) + y + (r * Math.cos(a));
      a += alphaTheta;

      double iccX = (x - r * Math.sin(a));
      double iccY = (y + r * Math.cos(a));

      direction = "Curve";
      curveInfo = String.format(" | iccRadius = %6.2f | icc = (%6.2f, %6.2f) | deltaAlpha = %6.2f(deg)", r, iccX, iccY,
          Math.toDegrees(alphaTheta));
      // IMPORTANT: To get rid of negative radius values, use Math.abs(r) in the above
      // format string
    }

    // Angle correction
    if (a > Math.toRadians(180))
      a -= Math.toRadians(360);
    if (a < Math.toRadians(-180))
      a += Math.toRadians(360);

    System.out.printf("%-10s x = %6.2f | y = %6.2f | a = %5.2f(deg) %s\n",
        direction, x, y, Math.toDegrees(a), curveInfo);
  }

  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());

    System.out.println("Time Step = " + TimeStep);

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation serv
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);

    // Get the encoders
    LeftEncoder = Epuck.getPositionSensor("left wheel sensor");
    RightEncoder = Epuck.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(TimeStep);
    RightEncoder.enable(TimeStep);
    PreviousLeft = 0;
    PreviousRight = 0;

    // Store the (x, y) location amd angle (degrees) estimate
    double x = 0.0, y = 0.0, a = 90.0, r = 0.0, td = 0.0;

    // Move the robot forward for 5 seconds
    Move(MAX_SPEED, MAX_SPEED, 5);

    // Spin the robot right for 6 seconds
    Move(MAX_SPEED, -MAX_SPEED, 6);

    // Curve the robot right for 10 seconds with right speed 0.2 less than full left
    // speed
    Move(MAX_SPEED, MAX_SPEED - 0.2, 10);

    // Curve the robot left for 20 seconds with left speed 1/3 of full right speed
    Move(MAX_SPEED / 3, MAX_SPEED, 20);

    // Move the robot forward for 10 seconds
    Move(MAX_SPEED, MAX_SPEED, 10);

    // Spin the robot left for 7.5 seconds
    Move(-MAX_SPEED, MAX_SPEED, 7.5);

    // Move the robot forward for 20 seconds
    Move(MAX_SPEED, MAX_SPEED, 20);

  }
}
