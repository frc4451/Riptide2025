package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class SingleRollerIOTalonFX implements SingleRollerIO {
  private final TalonFX talon;
  private final double reduction;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final VelocityVoltage velocityOut =
      new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public SingleRollerIOTalonFX(
      int canId, double reduction, double currentLimitAmps, boolean invert) {
    this.reduction = reduction;

    talon = new TalonFX(canId);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    voltage = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(currentLimitAmps);
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.phoenixUpdateFreqHz,
        position,
        velocity,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius);
    talon.optimizeBusUtilization(0.0, 1.0);

    talon.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius)
            .isOK();

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  public void runVelocity(double velocityRadPerSecond) {
    talon.setControl(velocityOut.withVelocity(Units.radiansToRotations(velocityRadPerSecond)));
  }

  @Override
  public void runPosition(double positionRad) {
    talon.setControl(positionOut.withPosition(Units.radiansToRotations(positionRad) * reduction));
  }

  @Override
  public void resetPosition(double positionRad) {
    talon.setPosition(Units.radiansToRotations(positionRad) * reduction);
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
