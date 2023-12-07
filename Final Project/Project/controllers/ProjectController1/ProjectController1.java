// Author: Adam Luczaj (SN:101196424)
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.TouchSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Robot;

public class ProjectController1 {

  private static final byte    CAMERA_WIDTH = 64;
  private static final byte    CAMERA_HEIGHT = 64;
  private static final double  GRIPPER_MOTOR_MAX_SPEED = 0.1;
  
  // Various modes for the robot to be in
  static final byte GREEN_WANDER = 0;
  static final byte GREEN_HOME_IN     = 1;
  static final byte GREEN_AVOID       = 2;
  static final byte GREEN_STOP_AND_PICK_UP = 3;

  static final byte TURN_TO_WALL_ONE = 4; //go south
  static final byte BOOK_IT_TO_WALL_ONE = 5; //move south
  static final byte TURN_TO_WALL_TWO = 6; //turn east
  static final byte BOOK_IT_TO_WALL_TWO = 7; //move east

  static final byte DROP_OFF_THEN_BACK_UP = 8; //drop off and back up
  
  static final byte LEFT        = 9;
  static final byte RIGHT       = 10;
  static final byte NONE        = 11;

  static final byte SAFE_SPEED  = 5;
  static final byte FULL_SPEED  = 7;
  
  static final byte    STRAIGHT = 13; //TODO: Fix this
  // static final byte    SPIN_LEFT = 14;
  // static final byte    PIVOT_RIGHT = 15;
  // static final byte    CURVE_LEFT = 16;
  // static final byte    CURVE_RIGHT = 17;
  
  private static Robot           robot;
  private static Motor           leftMotor;
  private static Motor           rightMotor;
  private static Motor           gripperLift;
  private static Motor           gripperLeftSide;
  private static Motor           gripperRightSide;
  private static DistanceSensor  leftSideSensor; 
  private static DistanceSensor  rightSideSensor;
  private static DistanceSensor  leftAheadSensor; 
  private static DistanceSensor  rightAheadSensor;
  private static DistanceSensor  leftAngledSensor; 
  private static DistanceSensor  rightAngledSensor;
  private static TouchSensor     jarDetectedSensor;
  private static Compass         compass;
  private static Accelerometer   accelerometer;
  private static Camera          camera;
    
    
  // Wait for a certain number of milliseconds
  private static void delay(int milliseconds, int timeStep) {
    int elapsedTime = 0;
    while (elapsedTime < milliseconds) {
      robot.step(timeStep);
      elapsedTime += timeStep;
    }
  }
  
  // Put the gripper up/down to the given position.
  // -0.025 is "up all the way" and 0.001 is down as much as it can
  public static void liftLowerGripper(float position) {
    gripperLift.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLift.setPosition(position);
  }

  // Put the gripper open/closed to the given position
  // 0.099 is "open all the way" and 0.01 is closed as much as it can
  public static void openCloseGripper(float position) {
    gripperLeftSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperRightSide.setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    gripperLeftSide.setPosition(position);
    gripperRightSide.setPosition(position);
  }
  
  //Read the compass
  //south is -90, east is 0
  private static int getCompassReadingInDegrees() {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI/2) / Math.PI * 180.0;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }

  /*
  Adam Notes:
  - Base majority of code on lab 3
  - First we want to home into a jar (will be a problem for jars already delivered but we don't care about that currently), we want to
  home in on the colour green. 
  - When we are right in front of the green jar, we want to stop, open the arms, go forward a little bit,
  clamp onto the jar and then home in on the colour blue which is the destination.
  - When we reach the destination we drop off the can and increment our droppedOffJarsCount variable.
  - If you detect something to the left and right and not ahead then you are inside the doorway
  - Jar needs to be placed inside the doorway.
  */
  
  /*
  - TODO: Need better robot pickup code.
  - TODO: Need some avoid code for when looking for the jar.
  - TODO: Need some better drop off the jar code (when you drop it off where should)
          the robot be.
  */

  /* ASK TA:
   * - Show them program running, what they think of it.
   * - Ask about how this is going to be marked how to make the video etc.
   * - Ask about robot spinning at the end.
   * - Ask about robot tipping over other jars when I deliver them.
   */
  
  // This is where it all begins
  public static void main(String[] args) {
    robot = new Robot();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Set up the motors
    leftMotor = robot.getMotor("left wheel");
    rightMotor = robot.getMotor("right wheel");
    gripperLift = robot.getMotor("lift motor");
    gripperLeftSide = robot.getMotor("left finger motor");
    gripperRightSide = robot.getMotor("right finger motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0); 
    
    // Get and enable the distance sensors
    leftSideSensor = robot.getDistanceSensor("so0"); 
    leftAngledSensor = robot.getDistanceSensor("so1"); 
    leftAheadSensor = robot.getDistanceSensor("so3"); 
    rightAheadSensor = robot.getDistanceSensor("so4"); 
    rightAngledSensor = robot.getDistanceSensor("so6"); 
    rightSideSensor = robot.getDistanceSensor("so7"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    leftAngledSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    leftSideSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);
    
    // Prepare the accelerometer
    accelerometer = new Accelerometer("accelerometer");
    accelerometer.enable(timeStep);
    
    // Prepare the camera
    camera = new Camera("camera");
    camera.enable(timeStep);
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);
    
    // Prepare the jar detecting sensor
    jarDetectedSensor= new TouchSensor("touch sensor");
    jarDetectedSensor.enable(timeStep);
    
    // Run the robot
    byte currentMode = GREEN_WANDER;

    //related to jars and homing
    int droppedOffJarsCount = 0;
    boolean homeOnGreen = true; //when true home in on green, when false home in on blue
    boolean deliveringBlueAreaDetected = false;
    int deliveringToBlueAreaNoLongerDetected = 0;

    byte PREVIOUS_MODE = GREEN_WANDER;
    
    //related to speed/turning
    int leftSpeed, rightSpeed;
    leftSpeed = rightSpeed = 0;
    byte turnCount = 0;
    //byte currentDirection = NONE;

    int TURN_TO_WALL_ONE_THRESHOLD_1 = -92;
    int TURN_TO_WALL_ONE_THRESHOLD_2 = -88;

    int TURN_TO_WALL_TWO_THRESHOLD_1 = -1;
    int TURN_TO_WALL_TWO_THRESHOLD_2 = 1;
    
    byte currentDirection = STRAIGHT;
    
    //Open the claws
    liftLowerGripper((float)0.001);
    for (int i = 0; i < 50; i++) { robot.step(timeStep); } 
    openCloseGripper((float)0.099);
    for (int i = 0; i < 50; i++) { robot.step(timeStep); } 
    
    
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the sensors
      int[] image = camera.getImage();
      
      //get side sensor values.
      boolean leftAhead = (leftAheadSensor.getValue() < 0.5);
      boolean rightAhead = (rightAheadSensor.getValue() < 0.5);
      boolean leftSide = (leftSideSensor.getValue() < 0.5);
      //boolean leftAngled = (leftSideSensor.getValue() < 0.5);

      //get green jar sensor values
      int jarLeftCount = 0;
      int jarCenterCount = 0;
      int jarRightCount = 0;
      boolean jarDetectedLeft = false;
      boolean jarDetectedCenter = false;
      boolean jarDetectedRight = false;
      boolean jarNotDetected = false;

      //READ CAMERAS IF LOOKING FOR GREEN JAR
      if (currentMode == GREEN_WANDER || currentMode == GREEN_HOME_IN || currentMode == GREEN_AVOID) {

        //Left Camera Area
        for (int i = 0; i < 21; i++) {
          for (int j = 20; j < 64; j++) {
            int red = Camera.imageGetRed(image, 64, i, j);
            int green = Camera.imageGetGreen(image, 64, i, j);
            int blue = Camera.imageGetBlue(image, 64, i, j); 
            if (green > 60 && red < 50 && blue < 50) {
              jarLeftCount++;
              //System.out.println("Robot 1: leftCount++");
              break;
            }
          }
        }

        //Center Camera Area
        for (int i = 21; i < 43; i++) {
            for (int j = 30; j < 64; j++) {
                int red = Camera.imageGetRed(image, 64, i, j);
                int green = Camera.imageGetGreen(image, 64, i, j);
                int blue = Camera.imageGetBlue(image, 64, i, j); 
                if (green > 60 && red < 50 && blue < 50) {
                jarCenterCount++;
                //System.out.println("Robot 1: leftCount++");
                break;
                }
            }
        }

        //Right Camera Area
        for (int i = 43; i < 64; i++) {
            for (int j = 30; j < 64; j++) {
                int red = Camera.imageGetRed(image, 64, i, j);
                int green = Camera.imageGetGreen(image, 64, i, j);
                int blue = Camera.imageGetBlue(image, 64, i, j); 
                if (green > 60 && red < 50 && blue < 50) {
                jarRightCount++;
                //System.out.println("Robot 1: leftCount++");
                break;
                }
            }
        }

        System.out.println("Robot 1: We are looking for a Green Jar -- leftCount: " + jarLeftCount + " centerCount " + jarCenterCount + " rightCount " + jarRightCount);

        if (jarLeftCount > jarRightCount) {
          System.out.println("Robot 1: Green Jar is on the left side");
          jarDetectedLeft = true;
        } else if (jarRightCount > jarLeftCount) {
          System.out.println("Robot 1: Green Jar is on the right side");
          jarDetectedRight = true;
        } else if (jarCenterCount > 1) {
          System.out.println("Robot 1: Green Jar is in the middle.");
          jarDetectedCenter = true;
        } else {
          System.out.println("Robot 1: Green jar not detected.");
          jarNotDetected = true;
        }

      }

      int compassValue = getCompassReadingInDegrees();

      // THINK: Make a decision as to what MODE to be in (for green jar)
      switch(currentMode) {

        case GREEN_WANDER:
        
          System.out.println("Robot 1 -- THINK -- GREEN_WANDER");

          //if the green jar is detected, we want to home in on it
          if ((jarDetectedLeft || jarDetectedCenter || jarDetectedRight)) {
            currentMode = GREEN_HOME_IN;
            break;
          }
        
          break;
        
        case GREEN_HOME_IN: 
        
          System.out.println("Robot 1 -- THINK -- GREEN_HOME_IN");

          //we detect the green jar
          if (jarDetectedSensor.getValue() >= 0.9) {
            currentMode = GREEN_STOP_AND_PICK_UP;
            break;
          }
        
          break;
        
        case GREEN_AVOID:
        
          System.out.println("Robot 1 -- THINK -- GREEN_AVOID");

          //TODO: add for side sensors.

          //we no longer are near a wall.
          if (!(leftAhead || rightAhead)) {
            currentMode = PREVIOUS_MODE;
            break;
          }

          break;
          
        case GREEN_STOP_AND_PICK_UP:
        
          System.out.println("Robot 1 -- THINK -- GREEN_STOP_AND_PICK_UP");
          currentMode = TURN_TO_WALL_ONE;

          break;
        
        case TURN_TO_WALL_ONE:

          System.out.println("Robot 1 -- THINK -- TURN_TO_WALL_ONE");
          
          //System.out.println("compassValue = " + compassValue);
          
          //System.out.println(TURN_TO_WALL_ONE_THRESHOLD_1);
          //System.out.println(TURN_TO_WALL_ONE_THRESHOLD_2); 
          
          if (compassValue >= TURN_TO_WALL_ONE_THRESHOLD_1 && compassValue <= TURN_TO_WALL_ONE_THRESHOLD_2) {
            currentMode = BOOK_IT_TO_WALL_ONE;
            break;
          }

          break;

        case BOOK_IT_TO_WALL_ONE:

          System.out.println("Robot 1 -- THINK -- BOOK_IT_TO_WALL_ONE");
          
          //System.out.println("robotLeftAhead: " + leftAheadSensor.getValue() + " " + leftAhead);
          //System.out.println("robotRightAhead: " + rightAheadSensor.getValue() + " " + rightAhead);

          if (leftAhead && rightAhead) {
            currentMode = TURN_TO_WALL_TWO;
            break;
          }

          break;

        case TURN_TO_WALL_TWO:

          System.out.println("Robot 1 -- THINK -- TURN_TO_WALL_TWO");

          // System.out.println("compassValue = " + compassValue);
          
          // System.out.println(TURN_TO_WALL_TWO_THRESHOLD_1);
          // System.out.println(TURN_TO_WALL_TWO_THRESHOLD_2);
          
          
          if (compassValue >= TURN_TO_WALL_TWO_THRESHOLD_1 && compassValue <= TURN_TO_WALL_TWO_THRESHOLD_2) {
            currentMode = BOOK_IT_TO_WALL_TWO;
            break;
          }

          break;

        case BOOK_IT_TO_WALL_TWO:

          System.out.println("Robot 1 -- THINK -- BOOK_IT_TO_WALL_TWO");

          if (leftSide) {
            currentMode = DROP_OFF_THEN_BACK_UP;
            break;
          }

          break;

        case DROP_OFF_THEN_BACK_UP:

          System.out.println("Robot 1 -- THINK -- DROP_OFF_THEN_BACK_UP");

          currentMode = GREEN_WANDER;
          break;
      
      }
      
      double random = 0.0;
      leftSpeed = SAFE_SPEED;
      rightSpeed = SAFE_SPEED;

      // REACT: Move motors according to the MODE
      switch (currentMode) {
      
        //what we want the robot to do when its looking for a jar 
        case GREEN_WANDER:
          
          System.out.println("Robot 1 -- REACT -- GREEN_WANDER");
        
          if (turnCount > 0) {
            if (currentDirection == LEFT)
              leftSpeed -= 2;
            else
              rightSpeed -= 2;
            turnCount--;
            if (--turnCount == 0) 
              currentDirection = NONE;
          }
          else {
            if ((byte)(Math.random()*5) == 0) {// turn 20% of the time
              currentDirection = (byte)(Math.random()*2);
              turnCount = (byte)(Math.random()*51+25);
            }
          }
          break;
        
        //what we want the robot to do when it has detected a jar  
        case GREEN_HOME_IN:

          System.out.println("Robot 1 -- REACT -- GREEN_HOME_IN");
          
          
          if (jarDetectedCenter) {
            leftSpeed = SAFE_SPEED; //was SAFE_SPEED
            rightSpeed = SAFE_SPEED; //was SAFE_SPEED
          }
          
          else if (jarDetectedRight) {
          
            leftSpeed = (SAFE_SPEED); //was 6
            rightSpeed = (SAFE_SPEED / 2); //was 3
          }
          
          else if (jarDetectedLeft) {
            leftSpeed = (SAFE_SPEED / 2); //was 3
            rightSpeed = (SAFE_SPEED); //was 6
          }
        
          turnCount = 0;
          break;
        
        //what we want the robot to do when its about to hit a wall
        case GREEN_AVOID:
        
          System.out.println("Robot 1 -- REACT -- GREEN_AVOID");
          
          
          random = Math.random();
          System.out.println(random);
          
          if (random > 0.5) {
            leftSpeed = (SAFE_SPEED/2);
            rightSpeed = (-(SAFE_SPEED/2));
          } else {
            leftSpeed = (-(SAFE_SPEED/2));
            rightSpeed = (SAFE_SPEED/2);
          }
          
        
          turnCount = 0;
          
          break;
         
        //what we want the robot to do when it gets in front of a green jar 
        case GREEN_STOP_AND_PICK_UP:
        
           System.out.println("Robot 1 -- REACT -- GREEN_STOP_AND_PICK_UP");
         
           //stop the robot
           leftSpeed = 0;
           rightSpeed = 0;
           leftMotor.setVelocity(leftSpeed);
           rightMotor.setVelocity(rightSpeed);
           for (int i = 0; i < 100; i++) { robot.step(timeStep); } 
           
           //go forward a little
           leftSpeed=1;    
           rightSpeed=1;  
           leftMotor.setVelocity(leftSpeed);
           rightMotor.setVelocity(rightSpeed);
           for (int i = 0; i < 4; i++) { robot.step(timeStep); }  
           
           //stop again
           leftSpeed=0;
           rightSpeed=0;  
           leftMotor.setVelocity(leftSpeed);
           rightMotor.setVelocity(rightSpeed); 
           for (int i = 0; i < 3; i++) { robot.step(timeStep); }  
           
           
           //close the gripper
           openCloseGripper((float)0.01);
           for (int i = 0; i < 100; i++) { robot.step(timeStep); }
           
           //lift the object
           //liftLowerGripper((float)-0.025);
           //for (int i = 0; i < 100; i++) { robot.step(timeStep); }          
           
           break;
        
        case TURN_TO_WALL_ONE:

          System.out.println("Robot 1 -- REACT -- BOOK_IT_TO_WALL_ONE");
          
          if (compassValue < -90) {
            //CURVE LEFT
            leftSpeed = (-1 * 2);
            rightSpeed = (2);

          }

          if (compassValue > -90) {
            //CURVE RIGHT
            leftSpeed = (2);
            rightSpeed = (-1 * 2);
          }
          

          break;
        
        case BOOK_IT_TO_WALL_ONE:
        
          System.out.println("Robot 1 -- REACT -- BOOK_IT_TO_WALL_ONE");

          //full speed ahead!
          leftSpeed = SAFE_SPEED; //was FULL_SPEED
          rightSpeed = SAFE_SPEED;
          
          if (compassValue < -90) {
            //SPIN LEFT!
            leftSpeed = (SAFE_SPEED -1);
            rightSpeed = (SAFE_SPEED);
          }
          
          if (compassValue > -90) {
            //CURVE RIGHT
            leftSpeed = (SAFE_SPEED);
            rightSpeed = (SAFE_SPEED - 1);
          }

          break;
        
        case TURN_TO_WALL_TWO:
        
         System.out.println("Robot 1 -- REACT -- TURN_TO_WALL_TWO");
          
         if (compassValue < 0) {
            //CURVE LEFT
            leftSpeed = (-1 * 2);
            rightSpeed = (2);

          }

          if (compassValue > 0) {
            //SPIN RIGHT
            leftSpeed = (2);
            rightSpeed = (-1 * 2);
          }

          break;

        case BOOK_IT_TO_WALL_TWO:
        
          System.out.println("Robot 1 -- REACT -- BOOK_IT_TO_WALL_TWO");
          
          if (compassValue < 0) {
            //CURVE LEFT
            leftSpeed = (SAFE_SPEED -1);
            rightSpeed = (SAFE_SPEED);
          }
          
          if (compassValue > 0) {
            //CURVE RIGHT
            leftSpeed = (SAFE_SPEED);
            rightSpeed = (SAFE_SPEED - 1);
          }          

          break;

          case DROP_OFF_THEN_BACK_UP:

          System.out.println("Robot 1 -- REACT -- DROP_OFF_THEN_BACK_UP");
         
          //stop the robot.
          leftSpeed = 0;
          rightSpeed = 0;
          leftMotor.setVelocity(leftSpeed);
          rightMotor.setVelocity(rightSpeed);
          for (int i = 0; i < 50; i++) { robot.step(timeStep); }
         
          //drop the cup
          openCloseGripper((float)0.099);
          //liftLowerGripper((float)-0.01);
          for (int i = 0; i < 50; i++) { robot.step(timeStep); }
           
          droppedOffJarsCount++;
           
          //back up a little bit.
          leftSpeed = -5;
          rightSpeed = -6;
          leftMotor.setVelocity(leftSpeed);
          rightMotor.setVelocity(rightSpeed);
          for (int i = 0; i < 140; i++) { robot.step(timeStep); }
           
           
          //turn around
          leftSpeed = -5;
          rightSpeed = 5;
          leftMotor.setVelocity(leftSpeed);
          rightMotor.setVelocity(rightSpeed);
          for (int i = 0; i < 50; i++) { robot.step(timeStep); } //turn over 180 degrees
           
          leftSpeed = SAFE_SPEED;
          rightSpeed = SAFE_SPEED;
         
          break;
      
      }

      PREVIOUS_MODE = currentMode;

      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
      
      System.out.println("---------------");

      //exit the program, jars are all dropped off
      if (droppedOffJarsCount == 5) {
        System.out.println("Robot 1: End of program " + droppedOffJarsCount + " jars delivered");
        System.out.println("Victory Dance!");
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
        break;
        //System.exit(0); // exit program.
      }
      
    }
    
    System.exit(0);
    
  }
}