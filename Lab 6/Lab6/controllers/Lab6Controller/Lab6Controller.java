// Room 38
// James Yap 101276054 (jamesyap@cmail.carleton.ca)
// Marco Wang 101183623 (marcowang@cmail.carleton.ca)

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class Lab6Controller {

  // *******************
  // Camera dimensions
  // *******************
  static final byte CAMERA_WIDTH = 52;
  static final byte CAMERA_HEIGHT = 39;

  // ****************************
  // The FIXED beacon locations
  // ****************************
  static final int xr = -60, yr = 30; // RED Beacon location
  static final int xg = 0, yg = -70; // GREEN Beacon location
  static final int xb = 60, yb = 0; // BLUE Beacon location
  // static final int x1 = -60, y1 = 30; // RED Beacon location
  // static final int x2 = 0, y2 = -70; // GREEN Beacon location
  // static final int x3 = 60, y3 = 0; // BLUE Beacon location

  // *****************************************************************************************************
  // The locations to visit in sequence. These numbers do not match the numbers in
  // the LAB instructions.
  // They have been tweaked slightly due to the innacurate movements of the robot.
  // DO NOT change any
  // of them because they have been adjusted so that your estimations will be as
  // accurate as possible.
  // *****************************************************************************************************
  static final int x[] = { 0, -34, 10, 39, 70, 72, 94, 83, 87, 56, 16, 3, -29, -49, -71, -74 };
  static final int y[] = { 0, 40, 41, 62, 67, 15, 17, 0, -45, -38, -62, -32, -26, -27, -9, -72 };

  static final double MAX_SPEED = 1; // Need to go this slow for accurate position estimation
  static final double WHEEL_RADIUS = 2.05; // cm
  static final double WHEEL_BASE = 5.80; // cm

  // *******************************************
  // Variables needed by the various functions
  // *******************************************
  static Robot Epuck;
  static Motor LeftMotor, RightMotor;
  static PositionSensor LeftEncoder, RightEncoder;
  static Camera Cam;
  static int TimeStep;
  static double PreviousReading = 0;
  static double startAngle = 90; // Angle that that robot starts at (i.e., 90 degrees)
  // static double currentAngle = startAngle;

  static final int RED = 0;
  static final int GREEN = 1;
  static final int BLUE = 2;
  static final int NONE = 3;

  static final int SCAN_Y_LVL = CAMERA_HEIGHT / 2;

  // *************************************************************************************
  // Set each motor to a specific speed and wait until the left sensor reaches
  // the specified number of radians. DO NOT CHANGE IT.
  // *************************************************************************************
  private static void move(double leftSpeed, double rightSpeed, double thisManyRadians) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    while (true) {
      double reading = RightEncoder.getValue() - PreviousReading;
      if ((thisManyRadians > 0) && (reading > thisManyRadians))
        break;
      if ((thisManyRadians < 0) && (reading < thisManyRadians))
        break;
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = RightEncoder.getValue();
    Epuck.step(TimeStep);
  }

  // *************************************************************************************
  // This method moves forward from point (x1,y1) to point (x2, y2). DO NOT CHANGE
  // IT.
  // *************************************************************************************
  private static void moveAhead(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double radiansToTurn = (Math.sqrt(xDiff * xDiff + yDiff * yDiff) / WHEEL_RADIUS);

    move(MAX_SPEED, MAX_SPEED, radiansToTurn);
  }

  // *************************************************************************************
  // This method spins from point (x1,y1) to point (x2, y2). DO NOT CHANGE IT.
  // *************************************************************************************
  private static void makeTurn(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    double turn = Math.toDegrees(Math.atan2(yDiff, xDiff));
    turn = (turn - startAngle) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;

    // Determine how many radians to spin for
    double radiansToTurn = Math.toRadians(turn) * (1.0 / (WHEEL_RADIUS / WHEEL_BASE * 2.0));
    if (turn > 0)
      move(-MAX_SPEED, MAX_SPEED, radiansToTurn);
    else
      move(MAX_SPEED, -MAX_SPEED, radiansToTurn);
    startAngle += turn;
  }

  static boolean isRed(int red, int green, int blue) {
    return (red > 60 && green < 50 && blue < 50);
  }

  static boolean isGreen(int red, int green, int blue) {
    return (red < 50 && green > 60 && blue < 50);
  }

  static boolean isBlue(int red, int green, int blue) {
    return (red < 50 && green < 50 && blue > 60);
  }

  static int getColor(int red, int green, int blue) {
    if (isRed(red, green, blue)) {
      return RED;
    } else if (isGreen(red, green, blue)) {
      return GREEN;
    } else if (isBlue(red, green, blue)) {
      return BLUE;
    } else {
      return NONE;
    }
  }

  // *************************************************************************************
  // This method spins the robot clockwise 360 degrees looking for the three
  // beacons.
  // It then calculates its position based on triangulation. The angle
  // passed in is the angle that the robot started at before spinning.
  // The point number can be used for printing to indicate which point is
  // being calculated at this time.
  // *************************************************************************************
  private static void calculatePosition(double angle, byte pointNumber) {
    // SPIN for 360 degrees DO NOT CHANGE THIS CODE
    double spinRadians = (Math.PI * WHEEL_BASE) / WHEEL_RADIUS;
    LeftMotor.setVelocity(-MAX_SPEED);
    RightMotor.setVelocity(MAX_SPEED);

    // *************************************************************************************
    // ADD ANY INTIALIZATION CODE YOU WANT HERE
    // *************************************************************************************
    final int PIXEL_THRESH = 2; // left and right balancing allows for an imbalance of 2 pixels

    int prevColor = NONE;

    int leftPixels = -1;
    int rightPixels = -1;

    double angles[] = { -1, -1, -1 };

    int r, g, b;
    int currentColor = NONE;
    // boolean firstDetected = false;

    // Do NOT alter the looping code!!! ... just insert inside it at the location
    // shown
    double reading = 0;
    while ((Epuck.step(TimeStep) != -1) && (reading < spinRadians)) {
      reading = RightEncoder.getValue() - PreviousReading;

      // *************************************************************************************
      // INSERT YOUR BEACON-FINDING CODE HERE

      // GET THE CAMERA IMAGE
      // LOOP THROUGH ALL PIXELS IN THE CENTER ROW OF THE IMAGE
      int image[] = Cam.getImage();

      int centerR = Camera.imageGetRed(image, CAMERA_WIDTH, CAMERA_WIDTH / 2, SCAN_Y_LVL);
      int centerG = Camera.imageGetGreen(image, CAMERA_WIDTH, CAMERA_WIDTH / 2, SCAN_Y_LVL);
      int centerB = Camera.imageGetBlue(image, CAMERA_WIDTH, CAMERA_WIDTH / 2, SCAN_Y_LVL);

      for (int x = 0; x < CAMERA_WIDTH; x++) {
        r = Camera.imageGetRed(image, CAMERA_WIDTH, x, SCAN_Y_LVL);
        g = Camera.imageGetGreen(image, CAMERA_WIDTH, x, SCAN_Y_LVL);
        b = Camera.imageGetBlue(image, CAMERA_WIDTH, x, SCAN_Y_LVL);

        currentColor = getColor(r, g, b);

        // Reset the left and right pixels if the color changes
        if (prevColor != currentColor || prevColor == NONE) {
          leftPixels = -1;
          rightPixels = -1;

          // if (currentColor == NONE)
          // firstDetected = false;
        }
        prevColor = currentColor;

        // If we made it to this point without a color change (leftPixels won't reset to
        // -1), then we can start
        if (leftPixels == -1) {
          leftPixels = x;
        }
        rightPixels = CAMERA_WIDTH - x;

        if (currentColor != NONE && Math.abs(leftPixels - rightPixels) <= PIXEL_THRESH
            && getColor(centerR, centerG, centerB) == currentColor) {
          // if (!firstDetected) {
          double deltaTheta = (2.0 * WHEEL_RADIUS / WHEEL_BASE) * reading;
          angles[prevColor] = (angle + Math.toDegrees(deltaTheta)) % 360;
          // firstDetected = true;
          // }
        }
      }

      // CHECK IF THE PIXEL IS RED
      // IF IT IS ... REMEMBER THE FIRST RED, LAST RED and WHETHER THE CENTER OF THE
      // ROW IS RED
      // CHECK IF THE PIXEL IS GREEN
      // IF IT IS ... REMEMBER THE FIRST GREEN, LAST GREEN and WHETHER THE CENTER OF
      // THE ROW IS GREEN
      // CHECK IF THE PIXEL IS BLUE
      // IF IT IS ... REMEMBER THE FIRST BLUE, LAST BLUE and WHETHER THE CENTER OF THE
      // ROW IS BLUE
      // IF THERE WAS A RED IN THE CENTER AND THE NON-RED PIXELS ON THE LEFT IS EQUAL
      // TO THE NON-RED PIXELS ON THE RIGHT
      // THEN WE ARE CENTERED AT THE RED BEACON, SO COMPUTE AND STORE ITS ANGLE
      // IF THERE WAS A GREEN IN THE CENTER AND THE NON-GREEN PIXELS ON THE LEFT IS
      // EQUAL TO THE NON-GREEN PIXELS ON THE RIGHT
      // THEN WE ARE CENTERED AT THE GREEN BEACON, SO COMPUTE AND STORE ITS ANGLE
      // IF THERE WAS A BLUE IN THE CENTER AND THE NON-BLUE PIXELS ON THE LEFT IS
      // EQUAL TO THE NON-BLUE PIXELS ON THE RIGHT
      // THEN WE ARE CENTERED AT THE BLUE BEACON, SO COMPUTE AND STORE ITS ANGLE

      // *************************************************************************************

      // DO NOT CHANGE THE NEXT SIX LINES OF CODE!!
    }

    // Determine if we found all three beacons
    boolean foundAll = true;
    for (int i = 0; i < angles.length; i++) {
      if (angles[i] == -1) {
        foundAll = false;
      }
    }

    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = RightEncoder.getValue();
    Epuck.step(TimeStep);

    // *************************************************************************************
    // WRITE CODE HERE TO CALCULATE AND DISPLAY THE ROBOT LOCATION

    // STEP 1: compute the modified beacon coordinates
    double xPrime1 = xr - xg;
    double yPrime1 = yr - yg;
    double xPrime3 = xb - xg;
    double yPrime3 = yb - yg;

    // STEP 2: compute the three cotangents
    double t12 = 1.0 / Math.tan(Math.toRadians(angles[GREEN] - angles[RED]));
    double t23 = 1.0 / Math.tan(Math.toRadians(angles[BLUE] - angles[GREEN]));
    double t31 = (1.0 - t12 * t23) / (t12 + t23);

    // STEP 3: compute the modified circle center coordinates
    double xPrime12 = xPrime1 + t12 * yPrime1;
    double yPrime12 = yPrime1 - t12 * xPrime1;
    double xPrime23 = xPrime3 - t23 * yPrime3;
    double yPrime23 = yPrime3 + t23 * xPrime3;
    double xPrime31 = (xPrime3 + xPrime1) + t31 * (yPrime3 - yPrime1);
    double yPrime31 = (yPrime3 + yPrime1) - t31 * (xPrime3 - xPrime1);

    // STEP 4: compute k'31
    double kPrime31 = xPrime1 * xPrime3 + yPrime1 * yPrime3 + t31 * (xPrime1 * yPrime3 - xPrime3 * yPrime1);

    // STEP 5: compute d
    double d = (xPrime12 - xPrime23) * (yPrime23 - yPrime31) - (yPrime12 - yPrime23) * (xPrime23 - xPrime31);

    // STEP 6: compute the robot position (x, y)
    double x = xg + kPrime31 * (yPrime12 - yPrime23) / d;
    double y = yg + kPrime31 * (xPrime23 - xPrime12) / d;

    boolean noSolution = !foundAll || d == 0;
    boolean isInaccurate = Math.abs(d) < 100;

    // USE pointNumber PARAMETER WHEN PRINTING THE COORDINATE NUMBER

    // IF THERE WERE THREE ANGLES FOUND, THEN COMPUTE AND DISPLAY THE LOCATION
    // OTHERWISE INDICATE THAT THE LOCATION CANNOT BE COMPUTED

    // System.out.println("R: " + angles[RED] + " G: " + angles[GREEN] + " B: " +
    // angles[BLUE]);

    // System.out.println("d: " + d);

    if (noSolution) {
      System.out.printf("(x%d, y%d) = CANNOT COMPUTE LOCATION\n", pointNumber, pointNumber);
    } else {
      System.out.printf("(x%d, y%d) = (%d, %d) %s\n", pointNumber, pointNumber, Math.round(x), Math.round(y),
          isInaccurate ? " INACCURATE" : "");
    }

    // *************************************************************************************
  }

  // *******************************************************************************************
  // DO NOT CHANGE THE MAIN FUNCTION ... EXCEPT FOR COMMENTING THINGS OUT WHILE
  // YOU ARE TESTING
  // *******************************************************************************************
  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);

    // Get the encoders
    LeftEncoder = Epuck.getPositionSensor("left wheel sensor");
    RightEncoder = Epuck.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(TimeStep);
    RightEncoder.enable(TimeStep);

    // Set up the camera
    Cam = new Camera("camera");
    Cam.enable(TimeStep);

    // Travel through the points in the array one at a time in sequence and
    // determine the location at each point by using triangulation
    for (byte i = 0; i < x.length - 1; i++) {
      calculatePosition(startAngle, i);
      makeTurn(x[i], y[i], x[i + 1], y[i + 1]);
      moveAhead(x[i], y[i], x[i + 1], y[i + 1]);
    }
    calculatePosition(startAngle, (byte) (x.length - 1)); // Get the last one
  }
}
