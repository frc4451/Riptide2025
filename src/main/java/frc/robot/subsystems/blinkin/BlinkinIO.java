package frc.robot.subsystems.blinkin;

import org.littletonrobotics.junction.AutoLog;

public interface BlinkinIO {
  @AutoLog
  public static class BlinkinIOInputs {
    public BlinkinColors color = BlinkinColors.UNKNOWN;
    public double colorCode = color.getColorCode();
    public double outputColorCode = color.getColorCode();
  }

  public default void updateInputs(BlinkinIOInputs inputs) {}

  /**
   * @see BlinkinColors
   */
  public default void setColor(BlinkinColors color) {}
}
