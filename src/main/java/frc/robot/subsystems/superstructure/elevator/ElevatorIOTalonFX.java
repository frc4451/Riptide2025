package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final double reduction;

  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderVoltage;
  private final StatusSignal<Current> leaderSupplyCurrentAmps;
  private final StatusSignal<Current> leaderTorqueCurrentAmps;
  private final StatusSignal<Temperature> leaderTempCelsius;

  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerSupplyCurrentAmps;
  private final StatusSignal<Current> followerTorqueCurrentAmps;
  private final StatusSignal<Temperature> followerTempCelsius;

  private final MotionMagicVoltage positionVoltageRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut =
      new VoltageOut(0.0).withEnableFOC(false).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final Follower followOut;

  private double positionGoalInches;
  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;

  public ElevatorIOTalonFX(
      int leaderCanId,
      int followerCanId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean invertFollower,
      boolean isBrakeMode) {

    this.reduction = reduction;

    leader = new TalonFX(leaderCanId, Constants.alternateCanBus);
    follower = new TalonFX(followerCanId, Constants.alternateCanBus);

    followOut = new Follower(leaderCanId, invertFollower);
    follower.setControl(followOut);

    leaderPosition = leader.getPosition();
    leaderVelocity = leader.getVelocity();
    leaderVoltage = leader.getMotorVoltage();
    leaderSupplyCurrentAmps = leader.getSupplyCurrent();
    leaderTorqueCurrentAmps = leader.getTorqueCurrent();
    leaderTempCelsius = leader.getDeviceTemp();

    followerPosition = follower.getPosition();
    followerVelocity = follower.getVelocity();
    followerVoltage = follower.getMotorVoltage();
    followerSupplyCurrentAmps = follower.getSupplyCurrent();
    followerTorqueCurrentAmps = follower.getTorqueCurrent();
    followerTempCelsius = follower.getDeviceTemp();

    positionGoalInches = 0.0;
    positionSetpointRotations = this.leader.getClosedLoopReference();
    positionErrorRotations = this.leader.getClosedLoopError();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(currentLimitAmps);
    // This might help us, idk
    cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    cfg.Slot0
        .withKG(ElevatorConstants.realGains.kG())
        .withKS(ElevatorConstants.realGains.kS())
        .withKV(ElevatorConstants.realGains.kV())
        .withKA(ElevatorConstants.realGains.kA())
        .withKP(ElevatorConstants.realGains.kP())
        .withKD(ElevatorConstants.realGains.kD());

    // (Motion) Magic, the Gathering
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(
            Units.inchesToMeters(ElevatorConstants.trapezoidConstraints.maxVelocity)
                / ElevatorConstants.pullyCircumference
                * ElevatorConstants.reduction
            )
        .withMotionMagicAcceleration(
            Units.inchesToMeters(ElevatorConstants.trapezoidConstraints.maxAcceleration)
                / ElevatorConstants.pullyCircumference
                * ElevatorConstants.reduction)
        ;
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        Constants.phoenixUpdateFreqHz,
        leaderPosition,
        leaderVelocity,
        leaderVoltage,
        leaderSupplyCurrentAmps,
        leaderTorqueCurrentAmps,
        leaderTempCelsius,
        followerPosition,
        followerVelocity,
        followerVoltage,
        followerSupplyCurrentAmps,
        followerTorqueCurrentAmps,
        followerTempCelsius);
    leader.optimizeBusUtilization(0.0, 1.0);
    follower.optimizeBusUtilization(0.0, 1.0);

    leader.getConfigurator().apply(cfg);
    follower.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderVoltage,
                leaderSupplyCurrentAmps,
                leaderTorqueCurrentAmps,
                leaderTempCelsius,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerSupplyCurrentAmps,
                followerTorqueCurrentAmps,
                followerTempCelsius)
            .isOK();

    inputs.leaderPositionRad =
        Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / reduction;
    inputs.leaderVelocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / reduction;

    inputs.leaderAppliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrentAmps.getValueAsDouble();
    inputs.leaderTemperatureCelsius = leaderTempCelsius.getValueAsDouble();

    inputs.followerPositionRad =
        Units.rotationsToRadians(followerPosition.getValueAsDouble()) / reduction;
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble()) / reduction;

    inputs.followerAppliedVoltage = followerVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrentAmps.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrentAmps.getValueAsDouble();
    inputs.followerTemperatureCelsius = followerTempCelsius.getValueAsDouble();

    inputs.positionInches =
        Units.metersToInches(
            (this.leaderPosition.getValueAsDouble() / ElevatorConstants.reduction)
                * ElevatorConstants.pullyCircumference);
    inputs.velocityInchesPerSec =
        Units.metersToInches(
            (this.leaderVelocity.getValueAsDouble() / ElevatorConstants.reduction)
                * ElevatorConstants.pullyCircumference);
    inputs.positionGoalInches = positionGoalInches;
    inputs.positionSetpointInches =
        Units.metersToInches(
            (this.positionSetpointRotations.getValueAsDouble() / ElevatorConstants.reduction)
                * ElevatorConstants.pullyCircumference);
    inputs.positionErrorInches =
        Units.metersToInches(
            (positionErrorRotations.getValueAsDouble() / ElevatorConstants.reduction)
                * ElevatorConstants.pullyCircumference);
  }

  @Override
  public void runVolts(double volts) {
    this.leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPosition(double positionInches) {
    this.leader.setPosition(
        Units.inchesToMeters(positionInches)
            / ElevatorConstants.pullyCircumference
            * ElevatorConstants.reduction);
  }

  @Override
  public void setPositionGoal(double positionInches) {
    positionGoalInches = positionInches;

    this.leader.setControl(
        positionVoltageRequest.withPosition(
            Units.inchesToMeters(positionInches)
                / ElevatorConstants.pullyCircumference
                * ElevatorConstants.reduction));
  }
}
