// James Yap        [101276054]
// Christopher Shen [101149908]

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.filechooser.FileNameExtensionFilter;

public class VectorMap {

  public static int LINE_TOLERANCE;
  public static boolean ShowVector = true;

  public static final int VERTEX_RADIUS = 3;
  public static final int VERTEX_DIAMETER = VERTEX_RADIUS * 2;

  private static Color VertexColor = Color.red;
  private static Color EdgeColor = Color.blue;

  private int width;
  private int height;
  private ArrayList<Obstacle> allObstacles;
  private boolean reduced; // true only if the map has been reduced by applying the line-fitting

  // Create a vector map with the given with, height, resolution
  public VectorMap(int w, int h) {
    width = w;
    height = h;
    reduced = true;

    allObstacles = new ArrayList<Obstacle>();
  }

  // Add an obstacle to the map
  public void addObstacle(Obstacle ob) {
    allObstacles.add(ob);
  }

  // Clear the obstacles from the map to get reqady for a new trace
  public void clear() {
    allObstacles.clear();
    reduced = false;
  }

  // Return the distance of point i to line ab
  private double distanceToLine(Point a, Point b, Point i) {
    return Math.abs((b.x - a.x) * (a.y - i.y) - (a.x - i.x) * (b.y - a.y))
        / Math.sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  }

  // Apply the line tolerance to the obstacles so that there are a lot less
  // vertices
  public void applyLineTolerance() {
    ArrayList<Obstacle> newObstacles = new ArrayList<Obstacle>();

    // REMOVE THIS LINE FOR PART 3 AND REPLACE IT WITH YOUR CODE THAT
    // IMPLEMENTS THE INCREMENTAL LINE-FITTING ALGORITHM

    for (int currObstacleIndex = 0; currObstacleIndex < allObstacles.size(); currObstacleIndex++) {
      // Set polygon to be a new obstacle with no vertices
      Obstacle rawObstacle = allObstacles.get(currObstacleIndex);
      Obstacle newObstacle = new Obstacle();

      // First, make sure the point set has at least 3 points, otherwise quit
      if (rawObstacle.numVertices() < 3)
        continue;

      int a = 0;
      Point pa = rawObstacle.getVertex(a);
      newObstacle.addVertex((int) pa.getX(), (int) pa.getY());

      for (int b = 1; b < rawObstacle.numVertices(); b++) {
        Point pb = rawObstacle.getVertex(b);

        for (int i = a + 1; i < b; i++) {
          // for (int i = a + 1; i < b - 1; i++) {
          Point pi = rawObstacle.getVertex(i);
          if (distanceToLine(pa, pb, pi) > LINE_TOLERANCE) {
            Point pbm1 = rawObstacle.getVertex(b - 1);

            if (!pbm1.equals(newObstacle.getVertex(newObstacle.numVertices() - 1)))
              newObstacle.addVertex((int) pbm1.getX(), (int) pbm1.getY());

            a = b - 1;
          }

        }
      }

      if (newObstacle.numVertices() >= 3)
        newObstacles.add(newObstacle);
    }

    /*** MAKE SURE to use the distanceToLine() method that is provided above ***/

    // DO NOT CHANGE THESE LINES
    allObstacles = newObstacles;
    reduced = true;
  }

  // Draw the vector map
  public void draw(Graphics aPen, int magnification) {
    try {
      // Display the obstacles
      for (Obstacle ob : allObstacles) {
        ArrayList<Point> vertices = ob.getVertices();
        if (vertices.size() == 0)
          continue;

        // Draw the edges first
        for (int i = 0; i < vertices.size() - 1; i++) {
          aPen.setColor(EdgeColor);
          aPen.drawLine(MapperApp.MARGIN_X / 2 + vertices.get(i).x * magnification,
              MapperApp.MARGIN_Y / 2 + height * magnification - vertices.get(i).y * magnification,
              MapperApp.MARGIN_X / 2 + vertices.get(i + 1).x * magnification,
              MapperApp.MARGIN_Y / 2 + height * magnification - vertices.get(i + 1).y * magnification);
        }
        aPen.drawLine(MapperApp.MARGIN_X / 2 + vertices.get(0).x * magnification,
            MapperApp.MARGIN_Y / 2 + height * magnification - vertices.get(0).y * magnification,
            MapperApp.MARGIN_X / 2 + vertices.get(vertices.size() - 1).x * magnification,
            MapperApp.MARGIN_Y / 2 + height * magnification - vertices.get(vertices.size() - 1).y * magnification);

        // Now Draw the vertices
        for (int i = 0; i < vertices.size(); i++) {
          aPen.setColor(VertexColor);
          aPen.fillOval(MapperApp.MARGIN_X / 2 + vertices.get(i).x * magnification - VERTEX_RADIUS,
              MapperApp.MARGIN_Y / 2 + height * magnification - vertices.get(i).y * magnification - VERTEX_RADIUS,
              VERTEX_DIAMETER, VERTEX_DIAMETER);
          aPen.setColor(Color.black);
          aPen.drawOval(MapperApp.MARGIN_X / 2 + vertices.get(i).x * magnification - VERTEX_RADIUS,
              MapperApp.MARGIN_Y / 2 + height * magnification - vertices.get(i).y * magnification - VERTEX_RADIUS,
              VERTEX_DIAMETER, VERTEX_DIAMETER);
        }
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }

}