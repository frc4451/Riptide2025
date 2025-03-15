package frc.robot.subsystems.rollers.follow;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class FollowRollersIOTalonFX implements FollowRollersIO {
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

  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> velocitySetpointRotationsPerSec;

  private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0.0).withUpdateFreqHz(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final Follower followOut;

  private double positionGoalRotations = 0;

  public FollowRollersIOTalonFX(
      int leaderCanId,
      int followerCanId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean invertFollower,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig) {
    this.reduction = reduction;

    voltageOut.EnableFOC = foc;
    mmVoltage.EnableFOC = foc;

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

    positionSetpointRotations = leader.getClosedLoopReference();
    velocitySetpointRotationsPerSec = leader.getClosedLoopReferenceSlope();

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
        followerTempCelsius,
        positionSetpointRotations,
        velocitySetpointRotationsPerSec);
    leader.optimizeBusUtilization(0.0, 1.0);
    follower.optimizeBusUtilization(0.0, 1.0);

    leader.getConfigurator().apply(cfg);
    follower.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(FollowRollersMagicIOInputs inputs) {
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
                followerTempCelsius,
                positionSetpointRotations,
                velocitySetpointRotationsPerSec)
            .isOK();

    inputs.leaderPositionRotations = leaderPosition.getValueAsDouble() / reduction;
    inputs.leaderVelocityRotationsPerSec = leaderVelocity.getValueAsDouble() / reduction;

    inputs.leaderAppliedVoltage = leaderVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrentAmps.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrentAmps.getValueAsDouble();
    inputs.leaderTemperatureCelsius = leaderTempCelsius.getValueAsDouble();

    inputs.followerPositionRotations = followerPosition.getValueAsDouble() / reduction;
    inputs.followerVelocityRotationsPerSec = followerVelocity.getValueAsDouble() / reduction;

    inputs.followerAppliedVoltage = followerVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrentAmps.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrentAmps.getValueAsDouble();
    inputs.followerTemperatureCelsius = followerTempCelsius.getValueAsDouble();

    inputs.positionGoalRotations = positionGoalRotations / reduction;
    inputs.positionSetpointRotations = positionSetpointRotations.getValueAsDouble() / reduction;
    inputs.velocitySetpointRotationsPerSec =
        velocitySetpointRotationsPerSec.getValueAsDouble() / reduction;
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    leader.setControl(mmVoltage.withPosition(positionGoalRotations));
  }

  @Override
  public void resetPosition(double positionRotations) {
    leader.setPosition(positionRotations * reduction);
    follower.setPosition(positionRotations * reduction);
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
  }
}
