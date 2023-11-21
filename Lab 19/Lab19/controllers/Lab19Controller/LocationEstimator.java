import java.util.ArrayList;
import java.awt.Point;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.ImageRef;

public class LocationEstimator {
  public static final int NUM_SAMPLES = 100000; // Number of samples to start with
  public static final double PERCENT_TOLERANCE = 0.20; // 20% randomness error

  private VectorMap vectorMap; // a vector version of the map
  private int[] gridImage; // For display purposes only
  private ArrayList<Pose> currentEstimates;
  private int maxX, minX, maxY, minY;
  private int width, height;

  // A Location Estimator must have a map to start with
  public LocationEstimator(VectorMap vmp) {
    vectorMap = vmp;
    currentEstimates = new ArrayList<Pose>();
    calculateMinAndMax();
    width = vmp.getWidth();
    height = vmp.getHeight();
    gridImage = new int[width * height];
    System.out.println("Display Grid dimensions = (" + width + "," + height + ")");
  }

  public ArrayList<Pose> getPoseEstimates() {
    return currentEstimates;
  }

  // Find the minimum and maximum x and y of all obstacles
  public void calculateMinAndMax() {
    maxX = maxY = -1;
    minX = minY = 9999999;
    for (Obstacle ob : vectorMap.getObstacles()) {
      for (int i = 0; i < ob.numVertices(); i++) {
        Point p = ob.getVertex(i);
        if (p.x > maxX)
          maxX = p.x;
        if (p.x < minX)
          minX = p.x;
        if (p.y > maxY)
          maxY = p.y;
        if (p.y < minY)
          minY = p.y;
      }
    }
    // System.out.println("MIN("+minX+","+minY+") and MAX("+maxX+","+maxY+")");
  }

  // Return a pose that does not intersect any obstacles
  private boolean isValidPose(Pose p) {
    // If it is not inside an obstacle, then it is a good one
    for (Obstacle ob : vectorMap.getObstacles()) {
      Point pt = new Point(p.x, p.y);
      if (ob.contains(pt) || ob.pointOnBoundary(p.x, p.y))
        return false;
    }
    return true;
  }

  // Return a pose that does not intersect any obstacles
  private Pose getValidPose() {
    while (true) {
      Pose p = new Pose((int) (Math.random() * (maxX - minX) + minX), (int) (Math.random() * (maxY - minY) + minY),
          (int) (Math.random() * 360));
      if (isValidPose(p))
        return p;
    }
  }

  // Reset the pose estimates
  public void resetEstimates() {
    currentEstimates.clear();
    int sampleCount = 0;
    while (sampleCount < NUM_SAMPLES) {
      Pose p = getValidPose();
      currentEstimates.add(p);
      sampleCount++;
    }
  }

  // Return the intersection point of two lines (assumes that they do indeed
  // intersect)
  public Point intersectionPoint(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
    int d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (d == 0)
      return null;

    int xi = ((x3 - x4) * (x1 * y2 - y1 * x2) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
    int yi = ((y3 - y4) * (x1 * y2 - y1 * x2) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;

    Point p = new Point(xi, yi);
    if (xi < Math.min(x1, x2) || xi > Math.max(x1, x2))
      return null;
    if (xi < Math.min(x3, x4) || xi > Math.max(x3, x4))
      return null;

    return p;
  }

  // Determine if the given pose estimate is a good one based on the distance
  // reading d
  private boolean isGoodEstimate(Pose p, double d) {
    Point pPrime = new Point((int) (p.x + (1 + PERCENT_TOLERANCE) * d * Math.cos(Math.toRadians(p.angle))),
        (int) (p.y + (1 + PERCENT_TOLERANCE) * d * Math.sin(Math.toRadians(p.angle))));

    double minDist = Double.MAX_VALUE;
    for (Obstacle obj : vectorMap.getObstacles()) {
      for (int i = 0; i < obj.numVertices(); i++) {
        Point current = obj.getVertex(i);
        Point after = obj.getVertex((i + 1) % obj.numVertices());
        boolean linesIntersect = java.awt.geom.Line2D.Double.linesIntersect(current.x, current.y, after.x, after.y,
            pPrime.x, pPrime.y, p.x, p.y);
        if (linesIntersect) {
          Point q = intersectionPoint(current.x, current.y, after.x, after.y, pPrime.x, pPrime.y, p.x, p.y);
          if (q != null) {
            Point point = new Point(p.x, p.y);
            double qdist = calculateDistance(point, q);
            if (qdist < minDist) {
              minDist = qdist;
            }
          }
        }

      }
    }
    if (minDist == Double.MAX_VALUE) {
      return false;
    }

    return minDist > (1 - PERCENT_TOLERANCE) * d && minDist < (1 + PERCENT_TOLERANCE) * d;
    // REPLACE THIS LINE WITH YOUR CODE
  }

  private double calculateDistance(Point point1, Point point2) {
    double xDifference = point2.getX() - point1.getX();
    double yDifference = point2.getY() - point1.getY();

    return Math.sqrt(xDifference * xDifference + yDifference * yDifference);
  }

  // Re-estimate the position again based on the given sensor reading (in cm)
  public void estimateFromReading(double d) {
    ArrayList<Pose> goodEstimates = new ArrayList<>();
    for (Pose p : currentEstimates) {
      if (isGoodEstimate(p, d) && isValidPose(p)) {
        goodEstimates.add(p);
      }
    }
    if (goodEstimates.isEmpty()) {
      resetEstimates();
    } else {
      int c = NUM_SAMPLES / goodEstimates.size();
      currentEstimates.clear();
      for (Pose p : goodEstimates) {
        for (int i = 0; i < c; i++) {
          currentEstimates.add(new Pose(p.x, p.y, p.angle));
        }
      }
    }
    // REPLACE THsnIS LINE WITH YOUR CODE
  }

  // The robot has moved forward a bit, so update all the estimates, based on the
  // given forward movement (in pixels)
  public void updateLocation(double distance) {
    // WRITE YOUR CODE HERE
    for (Pose p : currentEstimates) {
      double randomPercent = PERCENT_TOLERANCE * (2 * Math.random() - 1);
      p.x += (int) (distance * (1 + randomPercent) * Math.cos(Math.toRadians(p.angle)));
      p.y += (int) (distance * (1 + randomPercent) * Math.sin(Math.toRadians(p.angle)));

    }
  }

  // The robot has turned, so update all the estimates based on the given change
  // in orientation (in degrees)
  public void updateOrientation(double degrees) {
    // WRITE YOUR CODE HERE
    for (Pose p : currentEstimates) {
      double randomPercent = PERCENT_TOLERANCE * (2 * Math.random() - 1);
      p.angle += (degrees * (1 + randomPercent));
    }

  }

  // Draw the location estimates
  public void draw(Display aPen, int magnification) {
    try {
      // Erase the old grid
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          gridImage[y * width + x] = 0x00000000;
        }
      }
      for (Pose p : currentEstimates) {
        if ((p.x < 0) || (p.x >= width) || (p.y < 0) || (p.y >= height))
          continue;
        gridImage[(height - 1 - p.y) * width + p.x] = 0xFFFFFFFF;
      }

      ImageRef imageOfGrid = aPen.imageNew(width, height, gridImage, Display.ARGB);
      aPen.imagePaste(imageOfGrid, 0, 0, false);
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}