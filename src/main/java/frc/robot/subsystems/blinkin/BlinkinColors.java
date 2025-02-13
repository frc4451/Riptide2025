package frc.robot.subsystems.blinkin;

/** Color Codes as defined here: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf */
public enum BlinkinColors {
  UNKNOWN(0.0),
  SOLID_HOT_PINK(0.57),
  SOLID_DARK_RED(0.59),
  SOLID_RED(0.61),
  SOLID_RED_ORANGE(0.63),
  SOLID_ORANGE(0.65),
  SOLID_GOLD(0.67),
  SOLID_YELLOW(0.69),
  SOLID_LAWN_GREEN(0.71),
  SOLID_LIME(0.73),
  SOLID_DARK_GREEN(0.75),
  SOLID_GREEN(0.77),
  SOLID_BLUE_GREEN(0.79),
  SOLID_AQUA(0.81),
  SOLID_SKY_BLUE(0.83),
  SOLID_DARK_BLUE(0.85),
  SOLID_BLUE(0.87),
  SOLID_BLUE_VIOLET(0.89),
  SOLID_VIOLET(0.91),
  SOLID_WHITE(0.93),
  SOLID_GRAY(0.95),
  SOLID_DARK_GRAY(0.97),
  SOLID_BLACK(0.99);

  private final double colorCode;

  private BlinkinColors(double colorCode) {
    this.colorCode = colorCode;
  }

  public double getColorCode() {
    return colorCode;
  }
}
