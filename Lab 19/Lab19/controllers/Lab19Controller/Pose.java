import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class Pose {
  public int      x;
  public int      y;
  public double   angle;     // in degrees
  
  public Pose(int ix, int iy, double a) {
    x = ix; y = iy;
    angle = a;
    //normalizeAngle();
  }
  
  public void drawWith(Display aPen, int height, int magnification) {
    // Draw the orientation
    /*aPen.setColor(0x000000); //  black
    aPen.drawLine(x*magnification, 
                  height - y*magnification, 
                  (int)(x*magnification+6*Math.cos(Math.toRadians(angle))), 
                  (height-(int)(y*magnification+6*Math.sin(Math.toRadians(angle)))));*/

    // Draw the pose with an intensity reflecting its certainty
    aPen.setColor(0xFF0000); //  red
    aPen.fillOval(x*magnification-2, height - y*magnification-2, 5, 5);
    //aPen.setColor(0x000000); //  black
    //aPen.drawOval(x*magnification-2, height - y*magnification-2, 5, 5);
  }
  
  public String toString() {
    return "("+x+","+y+","+angle+")";
  }
  
  public void normalizeAngle() {
    if (angle > 180)
      angle = 360 - angle;
    if (angle < -180)
      angle = 360 + angle;
  }
}
  