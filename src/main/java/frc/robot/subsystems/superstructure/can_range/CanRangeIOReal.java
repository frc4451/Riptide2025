package frc.robot.subsystems.superstructure.can_range;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class CanRangeIOReal implements CanRangeIO {
  private final CANrange canRange;
  private final StatusSignal<Distance> distance;
  private final StatusSignal<Boolean> isDetected;

  public CanRangeIOReal(int canId, boolean longRange) {
    canRange = new CANrange(canId, Constants.alternateCanBus);
    distance = canRange.getDistance();
    isDetected = canRange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(100, distance, isDetected);
    canRange.optimizeBusUtilization(0.0, 1.0);

    CANrangeConfiguration cfg = new CANrangeConfiguration();
    cfg.ProximityParams.ProximityThreshold = 0.2;
    cfg.ToFParams.withUpdateMode(
        longRange ? UpdateModeValue.LongRangeUserFreq : UpdateModeValue.ShortRangeUserFreq);
    canRange.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(CanRangeIOInputs inputs) {
    inputs.connected = StatusSignal.refreshAll(distance, isDetected).isOK();
    inputs.isDetected = isDetected.getValue();
    inputs.distanceMeters = distance.getValueAsDouble();
  }
}
