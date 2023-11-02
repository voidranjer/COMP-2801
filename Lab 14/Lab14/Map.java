import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.*;

public class Map {

  private static final float[] SIGMA_PROB = { 0.0214f, 0.1359f, 0.3413f, 0.3413f, 0.1359f, 0.0214f };
  private static final float DARKEN_FACTOR = 10;
  private static final float LIGHTEN_FACTOR = 20;
  private static final float ROBOT_RADIUS = 5.75f; // cm

  public static boolean ShowGrayscale = true;
  public static boolean ShowBinary = false;
  public static boolean ShowBorders = false;
  public static int BINARY_THRESHOLD;

  private static Color[] greyScalePalette = new Color[256];
  private float[][] tempGrid; // stores temporary sensor readings
  private float[][] occupancyGrid; // stores "occupied" ceryainty at each grid cell
  private byte[][] binaryGrid; // stores binary version of grid according to BINARY_THRESHOLD
  private VectorMap vectorMap; // a vector version of the map
  private int width;
  private int height;
  private float maxValue = 0.0f;
  private float minValue = 0.5f;

  // Create a map with the given with, height, resolution
  public Map(int w, int h) {
    width = w;
    height = h;
    tempGrid = new float[width][height];
    occupancyGrid = new float[width][height];
    binaryGrid = new byte[width][height];
    vectorMap = new VectorMap(w, h);

    // Create Grayscale Palette
    for (int i = 0; i <= 255; i++)
      greyScalePalette[i] = new Color(255 - i, 255 - i, 255 - i);
  }

  public int getWidth() {
    return width;
  }

  public int getHeight() {
    return height;
  }

  public VectorMap getVectorMap() {
    return vectorMap;
  }

  // Trace the border now
  private void traceWholeBorder(int xs, int ys, Obstacle obstacle) {
    int[] xOffset = { 1, 1, 0, -1, -1, -1, 0, 1 };
    int[] yOffset = { 0, 1, 1, 1, 0, -1, -1, -1 };

    int xc = xs, yc = ys; // The current point is the start point
    int d = 7;

    boolean notDone = true;
    while (notDone) {
      int dp;
      if (d % 2 == 0)
        dp = (d + 7) % 8;
      else
        dp = (d + 6) % 8;
      int i;
      for (i = 0; i < 8; i++) {
        if ((xc + xOffset[dp] >= 0) && (xc + xOffset[dp] < width) && (yc + yOffset[dp] >= 0)
            && (yc + yOffset[dp] < height)) {
          if (binaryGrid[xc + xOffset[dp]][yc + yOffset[dp]] != 0) {
            xc = xc + xOffset[dp];
            yc = yc + yOffset[dp];
            binaryGrid[xc][yc] = 2;

            obstacle.addVertex(xc, yc);

            d = dp;
            break;
          }
        }
        dp = (dp + 1) % 8;
      }
      if (i == 8) {
        notDone = false;
        binaryGrid[xc][yc] = 0; // Ignore single points
      }
      if ((xc == xs) && (yc == ys))
        notDone = false;
    }
  }

  // Find all the borders of the obstacles in the map
  // Assumes that binaryGrid has already been computed
  public void computeBorders() {
    vectorMap.clear(); // Leave this line

    for (int y = height - 1; y > 0; y--) {
      for (int x = 0; x < width; x++) {

        if ((binaryGrid[x][y] == 1) && ((x == 0) || (binaryGrid[x - 1][y] == 0))) {
          binaryGrid[x][y] = (byte) 2; // Mark it as a border

          Obstacle newObstacle = new Obstacle();
          newObstacle.addVertex(x, y);

          traceWholeBorder(x, y, newObstacle);

          if (newObstacle.numVertices() >= 3)
            vectorMap.addObstacle(newObstacle);
        }

      }
    }

    // Apply the line-fitting algorithm ... for Part 3
    vectorMap.applyLineTolerance();
  }

  // Compute the binary version of the grid
  public void computeBinaryGrid() {
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        if (occupancyGrid[x][y] > BINARY_THRESHOLD)
          binaryGrid[x][y] = (byte) 1;
        else
          binaryGrid[x][y] = (byte) 0;
      }
    }
  }

  // Open up a dialog box and get the name of a file and then load up the
  // occupancy grid from that file.
  public void readRestFromFile(DataInputStream theFile) {
    try {
      minValue = 255;
      maxValue = 0;
      for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
          occupancyGrid[x][y] = theFile.readFloat();
          if (occupancyGrid[x][y] > maxValue)
            maxValue = occupancyGrid[x][y];
          if (occupancyGrid[x][y] < minValue)
            minValue = occupancyGrid[x][y];
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
      JOptionPane.showMessageDialog(null, "Error reading from map file", "Error", JOptionPane.ERROR_MESSAGE);
    }
  }

  public void draw(Graphics aPen, int magnification) {
    try {
      // Display the occupancy grid
      for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
          // If binary is enabled, show as red and white
          if (ShowBinary) {
            if (binaryGrid[x][y] == 1)
              aPen.setColor(Color.red);
            else if (binaryGrid[x][y] == 0)
              aPen.setColor(Color.white);
          } else { // otherwise use grayscale
            if (ShowGrayscale && (!ShowBorders || (binaryGrid[x][y] != 2))) {
              double darkenFactor = 255;
              if (maxValue != minValue)
                darkenFactor /= (maxValue - minValue);
              short shade = (short) (occupancyGrid[x][y] * darkenFactor);
              if (shade < 0)
                shade = 0;
              if (shade > 255)
                shade = 255;
              aPen.setColor(greyScalePalette[shade]);
            } else
              aPen.setColor(Color.white);
          }
          // If ShowBorders is enabled, set color to blue
          if (ShowBorders && (binaryGrid[x][y] == 2))
            aPen.setColor(Color.blue);

          aPen.fillRect(MapperApp.MARGIN_X / 2 + x * magnification,
              MapperApp.MARGIN_Y / 2 + (height * magnification) - (y * magnification), magnification, magnification);
        }
      }

      // Display the vector map version if desired
      if (VectorMap.ShowVector) {
        vectorMap.draw(aPen, magnification);
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}
