import java.awt.*;

public class LidarProcessor {
  private static int VIEW_RANGE = 180; // 180 degree viewing angle
  
  private float[]    rangeData;        // The range data from 0 to 359 degrees around the center
  private Point      wayPointLeft;     // Used for debugging to show a goal for the robot
  private Point      wayPointRight;    // Used for debugging to show a goal for the robot
  private Point      centerPoint;      // The center of gravity of the computed points

    
  // Create a new lidar which faces the starting angle
  public LidarProcessor(int startingAngle) {
    rangeData = new float[360]; // one reading for each degree
    wayPointLeft = new Point(-1000, -1000);  // Off screen for now.
    wayPointRight = new Point(-1000, -1000); // Off screen for now.
    centerPoint = new Point(-1000, -1000); // Off screen for now.
  }

  // This function is ONLY used by the LidarDisplayApp for drawing purposes.
  // YOU SHOULD NOT CALL THIS function in your code
  public float[] getRangeData() { return rangeData; }

  // Reset the range data for another reading.
  public void resetRangeData() { 
    for (int a=0; a<360; a++)
      rangeData[a] = 0;
  }

  public Point getLeftWayPoint() { return wayPointLeft; }
  public Point getRightWayPoint() { return wayPointRight; }
  public Point getCenterPoint() { return centerPoint; }
  public void setLeftWayPoint(int x, int y) { wayPointLeft.x = x; wayPointLeft.y = y;}
  public void setRightWayPoint(int x, int y) { wayPointRight.x = x; wayPointRight.y = y;}
  public void setCenterPoint(int x, int y) { centerPoint.x = x; centerPoint.y = y;}

  // Add the given readings to the lidar data set
  public void applyReadings(float[] newRanges, int robotsAngle) {
    resetRangeData(); // DO NOT REMOVE THIS LINE
    
    // Lab 22: Slides 9 and 10
    for (int a = 0; a < newRanges.length; a++) {
      int angle = robotsAngle + 90 - a;

      // You will need to add code to ensure that this is always in the range from 0 to 359 (i.e., not negative nor above 359).
      if (angle < 0) {
        angle += 360;
      } else if (angle > 359) {
        angle -= 360;
      }

      rangeData[angle] = newRanges[a] * 100; // convert m to cm
    }

  }
}
