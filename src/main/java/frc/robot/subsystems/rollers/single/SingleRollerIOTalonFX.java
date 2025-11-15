package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class SingleRollerIOTalonFX implements SingleRollerIO {
  protected final TalonFX talon;
  protected final double reduction;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> velocitySetpointRotationsPerSec;

  protected final MotionMagicVoltage mmVoltage;
  protected final PositionVoltage positionVoltage;
  protected final VoltageOut voltageOut;
  protected final VelocityVoltage velocityVoltage;
  protected final NeutralOut neutralOut = new NeutralOut();

  protected double positionGoalRotations = 0;

  protected final Rotation2d minimumPosition;
  protected final Rotation2d maximumPosition;

  public SingleRollerIOTalonFX(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig) {
    this(
        canId,
        reduction,
        currentLimitAmps,
        invert,
        isBrakeMode,
        foc,
        gains,
        mmConfig,
        Rotation2d.fromRotations(Double.MIN_VALUE),
        Rotation2d.fromRotations(Double.MAX_VALUE));
  }

  public SingleRollerIOTalonFX(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      Rotation2d minimumPosition,
      Rotation2d maximumPosition) {
    this.reduction = reduction;

    talon = new TalonFX(canId, Constants.alternateCanBus);

    voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0).withEnableFOC(foc);
    mmVoltage = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(foc);
    velocityVoltage = new VelocityVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(foc);
    positionVoltage = new PositionVoltage(0.0).withUpdateFreqHz(0.0).withEnableFOC(foc);

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

    this.minimumPosition = minimumPosition.times(reduction);
    this.maximumPosition = maximumPosition.times(reduction);
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
  public void runVelocity(double velocityRotationsPerSec) {
    talon.setControl(velocityVoltage.withVelocity(velocityRotationsPerSec));
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    talon.setControl(
        mmVoltage.withPosition(
            MathUtil.clamp(
                positionGoalRotations,
                minimumPosition.getRotations(),
                maximumPosition.getRotations())));
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
