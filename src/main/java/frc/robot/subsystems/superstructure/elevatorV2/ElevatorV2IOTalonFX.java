package frc.robot.subsystems.superstructure.elevatorV2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.Gains;
import frc.robot.util.MotionMagicProps;

public class ElevatorV2IOTalonFX implements ElevatorV2IO {
  protected final TalonFX leader;
  private final TalonFX follower;

  private final double reduction;
  private final double inchesPerRad;
  private final boolean foc;

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

  private final VoltageOut voltageOut;
  private final MotionMagicVoltage motionMagicVoltage;
  private final NeutralOut neutralOut = new NeutralOut();

  private final Follower followOut;

  private double motorPositionGoalInches = 0.0;
  private final StatusSignal<Double> positionSetpointRotations;
  private final StatusSignal<Double> positionErrorRotations;

  public ElevatorV2IOTalonFX(
      int leaderCanId,
      int followerCanId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean invertFollower,
      boolean isBrakeMode,
      boolean foc,
      double inchesPerRad,
      Gains staticGains,
      MotionMagicProps motionMagicProps) {
    this.reduction = reduction;
    this.inchesPerRad = inchesPerRad;
    this.foc = foc;

    this.voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0).withEnableFOC(this.foc);
    this.motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(this.foc);

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
    positionErrorRotations = leader.getClosedLoopError();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
        cfg.MotorOutput
                .withInverted(invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        cfg.CurrentLimits
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(currentLimitAmps);

        cfg.withSlot0(
            new Slot0Configs()
                .withKV(staticGains.kV())
                .withKA(staticGains.kA())
                .withKG(staticGains.kG())
                .withKP(staticGains.kP())
                .withKD(staticGains.kD())
        );

        cfg.withMotionMagic(
            new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(motionMagicProps.cruiseVelocity())
                    .withMotionMagicAcceleration(motionMagicProps.acceleration())
                    .withMotionMagicJerk(motionMagicProps.jerk())
        );

        cfg.withFeedback(
          new FeedbackConfigs().withSensorToMechanismRatio(reduction)
        );
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
        positionErrorRotations);
    leader.optimizeBusUtilization(0.0, 1.0);
    follower.optimizeBusUtilization(0.0, 1.0);

    leader.getConfigurator().apply(cfg);
    follower.getConfigurator().apply(cfg);
  }

  @Override
  public void updateInputs(ElevatorV2IOInputs inputs) {
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

    inputs.positionGoalInches = this.getPositionGoalInches();
    inputs.positionSetpointInches =
        Units.rotationsToRadians(this.positionSetpointRotations.getValueAsDouble()) * inchesPerRad;
    inputs.positionErrorInches =
        Units.rotationsToRadians(this.positionErrorRotations.getValueAsDouble()) * inchesPerRad;
  }

  @Override
  public void runVolts(double volts) {
    leader.setControl(voltageOut.withOutput(volts).withEnableFOC(this.foc));
  }

  @Override
  public void resetPosition(double positionRad) {
    leader.setPosition(Units.radiansToRotations(positionRad) * reduction);
    follower.setPosition(Units.radiansToRotations(positionRad) * reduction);
  }

  @Override
  public void stop() {
    leader.setControl(neutralOut);
  }

  @Override
  public void setPosition(double positionRad) {
    leader.setPosition(Units.radiansToRotations(positionRad) * reduction);
  }

  @Override
  /**
   * Position for Motion Magic is the "goal" of the Trapezoid Profile. The Position needs to be in
   * mechanical rotations. So convert with the following:
   *
   * <p>positionInches / inchesPerRadian / (2*PI) * Reduction of motor
   */
  public void setPositionGoalInches(double positionInches) {
    this.motorPositionGoalInches = positionInches;
    double positionRad = positionInches / inchesPerRad;
    leader.setControl(
        motionMagicVoltage.withPosition(Units.radiansToRotations(positionRad) * reduction));
  }

  @Override
  public double getPositionInches() {
    return Units.rotationsToRadians(leader.getPosition().getValueAsDouble()) * inchesPerRad;
  }

  @Override
  public double getPositionGoalInches() {
    return this.motorPositionGoalInches;
  }
}
