// James Yap [101276054]
// Amin Kolahan [101122575]
// Room 39

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

  public static boolean ShowBinary = false;
  public static boolean ShowBorders = false;
  public static int BINARY_THRESHOLD;

  private static Color[] greyScalePalette = new Color[256];
  private float[][] tempGrid; // stores temporary sensor readings
  private float[][] occupancyGrid; // stores "occupied" ceryainty at each grid cell
  private byte[][] binaryGrid; // stores binary version of grid according to BINARY_THRESHOLD
  private int width;
  private int height;
  private double resolution;
  private float maxValue = 0.0f;
  private float minValue = 0.5f;

  // Create a mao with the given with, height, resolution and whether or not it is
  // binary (otherwise gray scale)
  public Map(int w, int h, double res) {
    width = w;
    height = h;
    resolution = res;
    tempGrid = new float[width][height];
    occupancyGrid = new float[width][height];
    binaryGrid = new byte[width][height];

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

  private boolean isInside(int x, int y) {
    return ((x >= 0) && (x < width) && (y >= 0) && (y < height));
  }

  // Tarce the border now
  private void traceWholeBorder(int xs, int ys) {
    int[] xOffset = { 1, 1, 0, -1, -1, -1, 0, 1 };
    int[] yOffset = { 0, 1, 1, 1, 0, -1, -1, -1 };

    int xc = xs;
    int yc = ys;
    int d = 7;

    while (true) {
      int dPrime;
      if (d % 2 == 0)
        dPrime = (d + 7) % 8;
      else
        dPrime = (d + 6) % 8;

      boolean found = false;

      for (int i = 0; i < 8; i++) {
        int tempX = xc + xOffset[dPrime];
        int tempY = yc + yOffset[dPrime];

        if (isInside(tempX, tempY)) {
          if (binaryGrid[tempX][tempY] != 0) {
            xc = tempX;
            yc = tempY;
            binaryGrid[xc][yc] = (byte) 2;
            d = dPrime;
            found = true;
            break;
          }
        }
        dPrime = (dPrime + 1) % 8;
      }
      if (!found) {
        binaryGrid[xc][yc] = (byte) 0;
        break;
      }
      if ((xc == xs) && (yc == ys))
        break;
    }
  }

  // Find all the borders of the obstacles in the map
  // Assumes that binaryGrid has already been computed
  public void computeBorders() {
    for (int y = height - 1; y >= 1; y--) {
      for (int x = 0; x < width; x++) {
        if (binaryGrid[x][y] == 1 && ((x == 0) || binaryGrid[x - 1][y] == 0)) {
          binaryGrid[x][y] = (byte) 2;
          traceWholeBorder(x, y);
        }
      }
    }
  }

  // Compute the binary version of the grid
  public void computeBinaryGrid() {
    for (int x = 0; x < width; x++)
      for (int y = 0; y < height; y++) {
        if (occupancyGrid[x][y] > BINARY_THRESHOLD)
          binaryGrid[x][y] = (byte) 1;
        else
          binaryGrid[x][y] = (byte) 0;
      }
  }

  // Returns true if the given point is within the occupancy grid boundaries and
  // false otherwise.
  boolean isInsideGrid(double x, double y) {
    return (((int) (x / resolution + width / 2) >= 0) && ((int) (y / resolution + height / 2) >= 0) &&
        ((int) (x / resolution + width / 2) < width) && ((int) (y / resolution + height / 2) < height));
  }

  // Set the value of the temp grid atthe given location to be the given amount
  void setTempGridValue(double x, double y, float amount) {
    tempGrid[(int) (x / resolution + width / 2)][(int) (y / resolution + height / 2)] = amount;
  }

  // Clear the area under the robot
  private void clearAroundRobot(double x, double y) {
    float inc = (float) Math.PI / 180.0f;
    for (float a = 0; a < Math.PI * 2; a += inc) {
      for (float d = 0; d < ROBOT_RADIUS; d += 0.5) {
        float px = (float) (x + d * Math.cos(a));
        float py = (float) (y + d * Math.sin(a));
        occupancyGrid[(int) (px / resolution + width / 2)][(int) (py / resolution + height / 2)] = 0;
      }
    }
  }

  // Transfer all the tempGrid changes to the occupancyGrid
  void updateGrid(double rx, double ry) {
    for (int x = 0; x < width; x++)
      for (int y = 0; y < height; y++) {
        occupancyGrid[x][y] += tempGrid[x][y];
        if (occupancyGrid[x][y] < 0)
          occupancyGrid[x][y] = 0;
        if (occupancyGrid[x][y] > 255)
          occupancyGrid[x][y] = 255;
        tempGrid[x][y] = 0;
        if (occupancyGrid[x][y] > maxValue)
          maxValue = occupancyGrid[x][y];
        if (occupancyGrid[x][y] < minValue)
          minValue = occupancyGrid[x][y];
      }
    clearAroundRobot(rx, ry);
  }

  // Apply the sensor model
  public void applySensorModelReading(double sensorX, double sensorY, int sensorAngle, double distance,
      double beamWidthInDegrees, double distanceErrorAsPercent) {
    for (double r = 0; r < distance * (1 + distanceErrorAsPercent); r += 0.25) {
      double xa = sensorX + r * Math.cos(Math.toRadians(sensorAngle + beamWidthInDegrees / 2));
      double ya = sensorY + r * Math.sin(Math.toRadians(sensorAngle + beamWidthInDegrees / 2));
      double xb = sensorX + r * Math.cos(Math.toRadians(sensorAngle - beamWidthInDegrees / 2));
      double yb = sensorY + r * Math.sin(Math.toRadians(sensorAngle - beamWidthInDegrees / 2));

      double w = beamWidthInDegrees / Math.sqrt((xa - xb) * (xa - xb) + (ya - yb) * (ya - yb)) / 2;

      for (double a = -beamWidthInDegrees / 2; a <= beamWidthInDegrees / 2; a += w) {
        double x = sensorX + (r * Math.cos(Math.toRadians(sensorAngle + a)));
        double y = sensorY + (r * Math.sin(Math.toRadians(sensorAngle + a)));

        int angleIndex = (int) (((a + beamWidthInDegrees / 2) / beamWidthInDegrees) * 5.99);
        int distIndex = (int) (((r - distance * (1 - distanceErrorAsPercent)) / (2 * distance * distanceErrorAsPercent))
            * 5.99);

        if (isInsideGrid(x, y)) {
          if (r >= distance * (1 - distanceErrorAsPercent)) {
            setTempGridValue(x, y, SIGMA_PROB[angleIndex] * SIGMA_PROB[distIndex] * DARKEN_FACTOR);
          } else
            setTempGridValue(x, y, -SIGMA_PROB[angleIndex] * LIGHTEN_FACTOR);
        }
      }
    }
  }

  // Open up a dialog box and get the name of a file and then load up the
  // occupancy grid from that file.
  public void readRestFromFile(DataInputStream theFile) {
    System.out.println("Trying to read from size " + width + "," + height);
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
          // If ShowBorders is enabled, set color to blue
          if (ShowBorders && (binaryGrid[x][y] == 2))
            aPen.setColor(Color.blue);

          // If binary is enabled, show as red and white
          if (ShowBinary) {
            if (binaryGrid[x][y] == 1)
              aPen.setColor(Color.red);
            else if (binaryGrid[x][y] == 0)
              aPen.setColor(Color.white);
          } else { // otherwise use grayscale
            if (!ShowBorders || (binaryGrid[x][y] != 2)) {
              double darkenFactor = 255;
              if (maxValue != minValue)
                darkenFactor /= (maxValue - minValue);
              short shade = (short) (occupancyGrid[x][y] * darkenFactor);
              if (shade < 0)
                shade = 0;
              if (shade > 255)
                shade = 255;
              aPen.setColor(greyScalePalette[shade]);
            }
          }
          aPen.fillRect(MapperApp.MARGIN_X / 2 + x * magnification,
              MapperApp.MARGIN_Y / 2 + (height * magnification) - (y * magnification), magnification, magnification);
        }
      }
    } catch (java.util.ConcurrentModificationException ex) {
      // Do nothing please
    }
  }
}
