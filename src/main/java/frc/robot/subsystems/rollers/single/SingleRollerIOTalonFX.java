package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> velocitySetpointRotationsPerSec;

  private final MotionMagicVoltage mmVoltage;
  private final VoltageOut voltageOut;
  private final NeutralOut neutralOut = new NeutralOut();

  private double positionGoalRotations = 0;

  public SingleRollerIOTalonFX(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig) {
    this.reduction = reduction;

    talon = new TalonFX(canId, Constants.alternateCanBus);

    voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0).withEnableFOC(foc);
    mmVoltage = new MotionMagicVoltage(0.0).withUpdateFreqHz(0).withEnableFOC(foc);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    voltage = talon.getMotorVoltage();
    supplyCurrentAmps = talon.getSupplyCurrent();
    torqueCurrentAmps = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    positionSetpointRotations = talon.getClosedLoopReference();
    velocitySetpointRotationsPerSec = talon.getClosedLoopReferenceSlope();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(currentLimitAmps);
    cfg.Slot0 = gains;
    cfg.MotionMagic = mmConfig;
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.phoenixUpdateFreqHz,
        position,
        velocity,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius,
        positionSetpointRotations,
        velocitySetpointRotationsPerSec);
    talon.optimizeBusUtilization(0.0, 1.0);

    talon.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                supplyCurrentAmps,
                torqueCurrentAmps,
                tempCelsius,
                positionSetpointRotations,
                velocitySetpointRotationsPerSec)
            .isOK();

    inputs.positionRotations = position.getValueAsDouble() / reduction;
    inputs.velocityRotationsPerSec = velocity.getValueAsDouble() / reduction;

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();

    inputs.positionGoalRotations = positionGoalRotations / reduction;
    inputs.positionSetpointRotations = positionSetpointRotations.getValueAsDouble() / reduction;
    inputs.velocitySetpointRotationsPerSec =
        velocitySetpointRotationsPerSec.getValueAsDouble() / reduction;
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    talon.setControl(mmVoltage.withPosition(positionGoalRotations));
  }

  @Override
  public void resetPosition(double positionRad) {
    talon.setPosition(positionRad * reduction);
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }
}
