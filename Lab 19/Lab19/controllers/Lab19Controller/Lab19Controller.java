// Author: Mark Lanthier (SN:100000001)
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Display;
import java.awt.Point;

public class Lab19Controller {
  static int     WORLD_WIDTH    = 0; // Will be set when floor size is read in from main()
  static int     WORLD_HEIGHT   = 0; // Will be set when floor size is read in from main()
  
  static final double  MAX_SPEED      =   10;  // 19.1 is maximum but it seems that higher values can distort the sensor readings
  static final int     UPDATE_FREQUENCY = 10; // Send reading updates only every 10 iterations
  
  static final double  TOO_CLOSE_THRESHOLD = 0.03; // 3cm is too close to an obstacle
  
  // Various modes that the robot may be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    SPIN_RIGHT = 2;
  
 
  // Sensors
  static DistanceSensor  rangeSensors[];
  static DistanceSensor  ultrasonicSensor;
  static Compass         compass;
    

  // Read the compass
  private static double getCompassReadingInDegrees(Compass compass) {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI/2) / Math.PI * 180.0;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return bearing;
  }
  
  // Read the compass
  private static Point  getCurrentLocation(Field translationField) {
    // Get the actual robot locationd and direction
    double values[] = translationField.getSFVec3f();
    return new Point((int)(values[0]*100), (int)-(values[2]*100));
  }


  // Send a sensor reading update to the estimator
  static void SendReadingUpdateToEstimator(MapperApp mapper, Field translationField) {
    // Offsets for front-facing Ultrasonic sensor
    float xOffset = 5.643f;
    double d = ultrasonicSensor.getValue() * 100;
    if (d < 300) // Don't send invalid readings
      mapper.updateReadingInLocationEstimator(d + xOffset);
  }
  
  // Send a turn update to the estimator
  static void SendTurnUpdateToEstimator(MapperApp mapper, double degrees) {
    mapper.updateOrientationInLocationEstimator(degrees);
    
  }
  
  // Send a distance update to the estimator
  static void SendDistanceUpdateToEstimator(MapperApp mapper, double distanceTraveled) {
    mapper.updateDistanceInLocationEstimator(distanceTraveled);
  }
  
  public static void main(String[] args) {
    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Get the floor dimensions and set up the MapperApp
    Node floor = robot.getFromDef("floor");
    Field fSize = floor.getField("floorSize");

    double[] sizes = fSize.getSFVec2f();
    WORLD_WIDTH = (int)(sizes[0]*100);
    WORLD_HEIGHT = (int)(sizes[1]*100);
    MapperApp  mapper = new MapperApp(WORLD_WIDTH, WORLD_HEIGHT, 
                                      Integer.parseInt(floor.getField("name").getSFString()),
                                      robot.getDisplay("display"));
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    Field   translationField = robotNode.getField("translation");
    
    // Prepare the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // Prepare the IR sensors
    rangeSensors = new DistanceSensor[9];
    for (int i=0; i<9; i++) {
      rangeSensors[i] = robot.getDistanceSensor("ds"+i);
      rangeSensors[i].enable(timeStep);
    }
    
    // Prepare the US sensor
    ultrasonicSensor = robot.getDistanceSensor("us2");
    ultrasonicSensor.enable(timeStep);
    

    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(timeStep);


    // Initialize the logic variable for turning
    byte    lastTurn = STRAIGHT;
    byte    desiredTurn = STRAIGHT;
    byte    turnDecision;
    int     readingsUpdateCounter = 0;  // Keep track of number of iterations of straight motion to send updates in
    double  startAngle = getCompassReadingInDegrees(compass);  // Angle we started turning at
    Point   startPoint = getCurrentLocation(translationField); // Point we started going straight at
    double  leftSpeed, rightSpeed;

    while (robot.step(timeStep) != -1) {    
      // SENSE: Read the distance sensors
      boolean leftAheadObj  = rangeSensors[3].getValue() < TOO_CLOSE_THRESHOLD;
      boolean leftAngleObj  = rangeSensors[2].getValue() < TOO_CLOSE_THRESHOLD;
      boolean rightAheadObj = rangeSensors[4].getValue() < TOO_CLOSE_THRESHOLD;
      boolean rightAngleObj = rangeSensors[5].getValue() < TOO_CLOSE_THRESHOLD;
      boolean leftSideObj   = rangeSensors[1].getValue() < TOO_CLOSE_THRESHOLD;
      boolean rightSideObj  = rangeSensors[6].getValue() < TOO_CLOSE_THRESHOLD;
      
      // THINK: Check for obstacles and decide how we need to turn
      switch(desiredTurn) {
        case SPIN_LEFT: 
          if (!rightAheadObj && !rightAngleObj && !leftAheadObj && !leftAngleObj) {
            desiredTurn = STRAIGHT;
            // Just stopped turning ... so send angle update
            SendTurnUpdateToEstimator(mapper, getCompassReadingInDegrees(compass) - startAngle);
            startAngle = getCompassReadingInDegrees(compass);
            startPoint = getCurrentLocation(translationField);
          }
          break;
        case SPIN_RIGHT: 
          if (!leftAheadObj && !leftAngleObj && !rightAheadObj && !rightAngleObj) {
            desiredTurn = STRAIGHT;
            // Just stopped turning ... so send angle update
            SendTurnUpdateToEstimator(mapper, getCompassReadingInDegrees(compass) - startAngle);
            startAngle = getCompassReadingInDegrees(compass);
            startPoint = getCurrentLocation(translationField);
          }
          break;
        default: 
          if (rightAheadObj || leftAheadObj || rightAngleObj || leftAngleObj){
            if (rightSideObj)
              desiredTurn = SPIN_LEFT;
            else if (leftSideObj)
              desiredTurn = SPIN_RIGHT;
            else if ((int)(Math.random()*2) % 2 == 0)
              desiredTurn = SPIN_LEFT;
            else
              desiredTurn = SPIN_RIGHT;
            // Just stopped going straight ... so send distance update
            SendDistanceUpdateToEstimator(mapper, (int)(getCurrentLocation(translationField).distance(startPoint)));
            startPoint = getCurrentLocation(translationField);
            startAngle = getCompassReadingInDegrees(compass);
            SendReadingUpdateToEstimator(mapper, translationField);
            readingsUpdateCounter = 1; // Reset the straight movement iteration counter
          }
          else {
            readingsUpdateCounter++; // We are still going straight so update the iteration counter
            // Send a reading update if it is time to do so
            if (readingsUpdateCounter % UPDATE_FREQUENCY == 0) {
              SendDistanceUpdateToEstimator(mapper, (int)(getCurrentLocation(translationField).distance(startPoint)));
              startPoint = getCurrentLocation(translationField);
              startAngle = getCompassReadingInDegrees(compass);
              SendReadingUpdateToEstimator(mapper, translationField);
            }
          }
          break;
      }
      
      lastTurn = desiredTurn;

      // REACT: Move motors accordingly
      switch(desiredTurn) {
        case SPIN_LEFT:
          leftSpeed  = -0.5 * MAX_SPEED;
          rightSpeed = 0.5 * MAX_SPEED;
          break;
        case SPIN_RIGHT:
          leftSpeed  = 0.5 * MAX_SPEED;
          rightSpeed = -0.5 * MAX_SPEED;
          break;
        default:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
      }
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
}
