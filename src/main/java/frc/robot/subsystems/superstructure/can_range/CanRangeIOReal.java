package frc.robot.subsystems.superstructure.can_range;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;

public class CanRangeIOReal implements CanRangeIO {
  private final CANrange canRange;
  private final StatusSignal<Distance> distance;

  public CanRangeIOReal(int canID, boolean longRange) {
    canRange = new CANrange(0);
    distance = canRange.getDistance();

    BaseStatusSignal.setUpdateFrequencyForAll(100, distance);
    canRange.optimizeBusUtilization(0.0, 1.0);

    CANrangeConfiguration cfg = new CANrangeConfiguration();
    cfg.ToFParams.withUpdateMode(
        longRange ? UpdateModeValue.LongRangeUserFreq : UpdateModeValue.ShortRangeUserFreq);
    canRange.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(CanRangeIOInputs inputs) {
    inputs.connected = StatusSignal.refreshAll(distance).isOK();
    inputs.distanceMeters = distance.getValueAsDouble();
  }
}
