package frc.robot.subsystems.blinkin;

public class BlinkinIOSim implements BlinkinIO {
  private BlinkinColors color = BlinkinColors.UNKNOWN;

  public void updateInputs(BlinkinIOInputs inputs) {
    inputs.color = color;
    inputs.colorCode = color.getColorCode();
    inputs.outputColorCode = inputs.colorCode;
  }

  @Override
  public void setColor(BlinkinColors color) {
    this.color = color;
  }
}
