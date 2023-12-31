package frc.robot.utils;

public class CTREConversion {
  // Positional conversions

  public static double falconToDegrees(int ticks, double gearRatio) {
    return (ticks * (360.0 / 2048)) / gearRatio;
  }

  public static int degreesToFalcon(double degrees, double gearRatio) {
    return (int) (degrees * (2048.0 / 360.0) * gearRatio);
  }
}
