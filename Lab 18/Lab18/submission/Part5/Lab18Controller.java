// Author: Mark Lanthier (SN:100000001)
import java.awt.Point; 
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;

public class Lab18Controller {

  static final double  MAX_SPEED      = 1;    // 19.1 is maximum but it seems that higher values can distort the sensor readings
  static int WORLD_WIDTH = 400;
  static int WORLD_HEIGHT = 300;
    
    
  static Compass    compass; // The compass sensor is needed to get the direction that the robot is facing
  static Motor      LeftMotor;
  static Motor      RightMotor;
  static int        TimeStep;
  static Supervisor robot;
  static Field      TranslationField;
  

  // This method moves forward from point (x1,y1) to point (x2, y2)
  private static void moveAhead(double x1, double y1, double x2, double y2) {
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
    //System.out.println("Final angle is " + finalAngle);

    if (turn > 0) {// Turn left
      LeftMotor.setVelocity(-MAX_SPEED);
      RightMotor.setVelocity(MAX_SPEED);
    }
    else { // turn right
      LeftMotor.setVelocity(MAX_SPEED);
      RightMotor.setVelocity(-MAX_SPEED);   
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
    // Load up the Test world
    VectorMap vm = new VectorMap(WORLD_WIDTH*2, WORLD_HEIGHT*2, 0.5);
    vm.loadTestWorld();   
    PathPlanner  planner = new PathPlanner(vm);
    planner.setStart(15, 150);
    planner.setEnd(360, 210);
    
    // Compute the path
    planner.growObstacles();
    planner.computeVisibilityGraph();
    System.out.println("Visiibility Graph computed with " + planner.getVisibilityGraph().getNodes().size() + 
                       " nodes and " + planner.getVisibilityGraph().getEdges().size() + " edges");
                       
    planner.computeShortestPath(); 
    ArrayList<Point> path = planner.getShortestPath();
    
    // Display the path
    System.out.println("Shortest Path has " + path.size() + " points");
    for (Point p: path) 
    	System.out.println(p);

    // Set up the robot
    robot = new Supervisor();
    TimeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // Code required for being able to get the robot's location
    Node    robotNode = robot.getSelf();
    TranslationField = robotNode.getField("translation");

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
    double values[] = TranslationField.getSFVec3f();
    double x = (values[0]*100) + WORLD_WIDTH/2;
    double y = -(values[2]*100) + WORLD_HEIGHT/2;
    double a = getCompassReadingInDegrees(compass);
    //System.out.println("Starting At ("+x+","+y+") and "+a+" degrees");
    
    // Run the robot
    while (robot.step(TimeStep) != -1) {
      // Check to make sure that the path has been obtained.  If so, start/continue moving
      if ((path != null) && (path.size() > 1)) {
        for (int i=1; i<path.size(); i++) {
          // Get the current location
          values = TranslationField.getSFVec3f();
          x = (values[0]*100) + WORLD_WIDTH/2;
          y = -(values[2]*100) + WORLD_HEIGHT/2;
          makeTurn(x, y, path.get(i).x, path.get(i).y);
          moveAhead(x, y, path.get(i).x, path.get(i).y);
        }
        path = null; // Reset for the next time
      }
    }
  }
}
