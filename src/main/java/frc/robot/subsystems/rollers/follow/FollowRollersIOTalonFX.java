package frc.robot.subsystems.rollers.follow;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final VelocityVoltage velocityOut =
      new VelocityVoltage(0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionVoltage positionOut =
      new PositionVoltage(0.0).withEnableFOC(true).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();

  private final Follower followOut;

  public FollowRollersIOTalonFX(
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

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
          cfg.MotorOutput
              .withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(currentLimitAmps);
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

  public void updateInputs(FollowRollersIOInputs inputs) {
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
  }

  public void runVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts));
  }

  public void runVelocity(double velocityRadPerSecond) {
    leader.setControl(velocityOut.withVelocity(Units.radiansToRotations(velocityRadPerSecond)));
  }

  public void runPosition(double positionRad, double feedforwardVolts) {
    leader.setControl(
        positionOut.withPosition(Units.radiansToRotations(positionRad) * reduction)
        // .withFeedForward(feedforwardVolts)
        );
  }

  @Override
  public void resetPosition(double positionRad) {
    leader.setPosition(Units.radiansToRotations(positionRad) * reduction);
    follower.setPosition(Units.radiansToRotations(positionRad) * reduction);
  }

  public void stop() {
    leader.setControl(neutralOut);
  }
}
