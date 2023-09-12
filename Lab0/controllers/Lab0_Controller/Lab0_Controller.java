// This code moves a robot around in a simple manner such that it avoids obstacles

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;

public class Lab0_Controller extends Robot {

  private final int timeStep = 32;
  private final double maxSpeed = 10.0;

  private DistanceSensor[] distanceSensors;
  private Motor[] motors;

  private double boundSpeed(double speed) {
    return Math.max(-maxSpeed, Math.min(maxSpeed, speed));
  }

  // COnstructor to initialize everything
  public Lab0_Controller() {
    motors = new Motor[] {getMotor("left wheel motor"),getMotor("right wheel motor")};
    motors[0].setPosition(Double.POSITIVE_INFINITY);
    motors[1].setPosition(Double.POSITIVE_INFINITY);
    motors[0].setVelocity(0.0);
    motors[1].setVelocity(0.0);
    distanceSensors = new DistanceSensor[] {getDistanceSensor("ds0"),getDistanceSensor("ds1")};
    for (int i=0; i<2; i++) 
      distanceSensors[i].enable(timeStep);
  }

  // This simulates the life of the robot
  public void run() {
    double   leftSpeed, rightSpeed; // motor speeds
    double   delta;                 // speed change based on obstacles
  
    // Loop forever ... until simulation is stopped
    while (step(timeStep) != -1) {
      // Read the sensors and determine the difference
      delta = distanceSensors[0].getValue() - distanceSensors[1].getValue();
      
      // Adjust the speeds accordingly
      leftSpeed = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
      rightSpeed = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
      motors[0].setVelocity(leftSpeed);
      motors[1].setVelocity(rightSpeed);
    }
  }

  // This is where the program starts
  public static void main(String[] args) {
    Lab0_Controller controller = new Lab0_Controller();
    controller.run();
  }
}
