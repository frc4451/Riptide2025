package frc.robot.subsystems.music;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Supplier;

public enum Instruments {
  FIRST(0, () -> new TalonFX(0));

  public final int trackNumber;
  public final Supplier<ParentDevice> deviceSource;

  private Instruments(int trackNumber, Supplier<ParentDevice> deviceSource) {
    this.trackNumber = trackNumber;
    this.deviceSource = deviceSource;
  }
}
