package frc.robot.subsystems.blinkin;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class BlinkinIOSpark implements BlinkinIO {
  private final Spark led = new Spark(0);
  private BlinkinColors color = BlinkinColors.UNKNOWN;

  public void updateInputs(BlinkinIOInputs inputs) {
    inputs.color = color;
    inputs.colorCode = color.getColorCode();
    inputs.outputColorCode = led.get();
  }

  @Override
  public void setColor(BlinkinColors color) {
    this.color = color;
    led.set(color.getColorCode());
  }
}
