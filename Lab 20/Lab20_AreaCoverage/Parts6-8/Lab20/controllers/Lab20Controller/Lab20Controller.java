// Author: Mark Lanthier (SN:100000001)
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;

public class Lab20Controller {

  static final double  MAX_SPEED = 10;
  static int WORLD_WIDTH = 400;
  static int WORLD_HEIGHT = 300;
    
    
  static Compass    compass; // The compass sensor is needed to get the direction that the robot is facing
  static Motor      LeftMotor;
  static Motor      RightMotor;
  static int        TimeStep;
  static Supervisor robot;
  static Field      TranslationField;
  

  // This method moves forward from point (x1,y1) to point (x2, y2)
  private static void moveAhead(MapperApp mapper, double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    
    // Go forward
    LeftMotor.setVelocity(MAX_SPEED);
    RightMotor.setVelocity(MAX_SPEED);
    //System.out.println("Moving Forward from ("+x1+","+y1+") to ("+x2+","+y2+")");
    
    double desiredDistance = Math.sqrt((xDiff)*(xDiff) + (yDiff)*(yDiff));
    // Now wait until the final position has been reached
    while(robot.step(TimeStep) != -1) {
      double values[] = TranslationField.getSFVec3f();
      double x = (values[0]*100) + WORLD_WIDTH/2;
      double y = -(values[2]*100) + WORLD_HEIGHT/2;
      if (Math.sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1)) >= desiredDistance) 
        break;
      mapper.updatePosition(x, y);
    }
    // Stop the motors
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0); 
  }
	
  // This method spins from point (x1,y1) to point (x2, y2)
  private static void makeTurn(double x1, double y1, double x2, double y2) {
    double xDiff = x2 - x1;
    double yDiff = y2 - y1;
    int startAngle = getCompassReadingInDegrees(compass);
    //System.out.println("Start Angle is " + startAngle);

    int turn = (int)(Math.atan2(yDiff, xDiff) * 180 / Math.PI);
    turn = (turn - startAngle) % 360;
    if (turn < -180)
      turn += 360;
    else if (turn > 180)
      turn -= 360;
    //System.out.println("Turn Amount is " + turn);
    
    int finalAngle = (startAngle + turn +360)%360;

    if (turn == 0) 
      return;
      
    if (turn > 0) {// Turn left
      LeftMotor.setVelocity(-1);//-MAX_SPEED/2);
      RightMotor.setVelocity(1);//MAX_SPEED/2);
    }
    else { // turn right
      LeftMotor.setVelocity(1);//MAX_SPEED/2);
      RightMotor.setVelocity(-1);//-MAX_SPEED/2);   
    }
    
    // Now wait until the final angle has been reached
    while(robot.step(TimeStep) != -1) {
      int degrees = getCompassReadingInDegrees(compass);
      if ((degrees+360)%360 == finalAngle) 
        break;
    }
    
    // Stop the motors
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0); 
  }
  
  // Read the compass
  private static int getCompassReadingInDegrees(Compass compass) {
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI/2) / Math.PI * 180.0;
    if (bearing > 180)
      bearing = 360 - bearing;
    if (bearing < -180)
      bearing = 360 + bearing;
    return (int)(bearing);
  }


  public static void main(String[] args) {
    // Set up the robot
    robot = new Supervisor();
    TimeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");

    MapperApp   mapper = new MapperApp(WORLD_WIDTH, WORLD_HEIGHT, robot.getDisplay("display"));  
    AreaCoverage areaCoverage = mapper.getAreaCoverage(); 

    // Prepare the motors
    LeftMotor = robot.getMotor("left wheel motor");
    RightMotor = robot.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);
    RightMotor.setPosition(Double.POSITIVE_INFINITY);
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0); 
    
    // Prepare the Compass sensor
    compass = robot.getCompass("compass");
    compass.enable(TimeStep);
    
    
    // Get the actual robot location and direction
    double x = 15;
    double y = 150;
    double a = 90; 
    System.out.println("Starting At ("+x+","+y+") and "+a+" degrees");
    
    // Assume that there is no path to move along yet
    ArrayList<Point> path = null;
    areaCoverage.setStart((int)x, (int)y);
    areaCoverage.getVectorMap().loadMapFromWorld();
    areaCoverage.computeGridGraph();
    areaCoverage.computeSpanningTree();
    
    // Get the path to follow
    path = areaCoverage.getCoveragePath();
    
    // Run the robot
    while (robot.step(TimeStep) != -1) {
      // Check to make sure that the path has been obtained.  If so, start/continue moving
      if ((path != null) && (path.size() > 1)) {
        for (int i=0; i<path.size(); i++) {
          // Get the current location
          double values[] = TranslationField.getSFVec3f();
          x = (values[0]*100) + WORLD_WIDTH/2;
          y = -(values[2]*100) + WORLD_HEIGHT/2;
          makeTurn(x, y, path.get(i).x, path.get(i).y);
          moveAhead(mapper, x, y, path.get(i).x, path.get(i).y);
        }
        mapper.showFinalTree();
        robot.step(TimeStep);
        break;
      }
    }
  }
}
