// Room 38
// James Yap 101276054
// Marco Wang 101183623

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
  static final int x1 = -60, y1 = 30; // RED Beacon location
  static final int x2 = 0, y2 = -70; // GREEN Beacon location
  static final int x3 = 60, y3 = 0; // BLUE Beacon location

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

    double angles[] = new double[3];

    int r, g, b;

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

        int currentColor = getColor(r, g, b);

        // Reset the left and right pixels if the color changes
        if (prevColor != currentColor || prevColor == NONE) {
          leftPixels = -1;
          rightPixels = -1;
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
          double deltaTheta = (2.0 * WHEEL_RADIUS / WHEEL_BASE) * reading;
          angles[prevColor] = (startAngle + Math.toDegrees(deltaTheta)) % 360;
        }
      }

      System.out.println("R: " + angles[RED] + " G: " + angles[GREEN] + " B: " + angles[BLUE]);

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
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = RightEncoder.getValue();
    Epuck.step(TimeStep);

    // *************************************************************************************
    // WRITE CODE HERE TO CALCULATE AND DISPLAY THE ROBOT LOCATION

    // USE pointNumber PARAMETER WHEN PRINTING THE COORDINATE NUMBER

    // IF THERE WERE THREE ANGLES FOUND, THEN COMPUTE AND DISPLAY THE LOCATION
    // OTHERWISE INDICATE THAT THE LOCATION CANNOT BE COMPUTED

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
