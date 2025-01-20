package frc.robot.subsystems.rollers.single;

public class SingleRollerIOSpark implements SingleRollerIO {
  //   private final SparkMax spark;
  //   private final RelativeEncoder encoder;

  //   private final Debouncer connectedDebounce = new Debouncer(0.5);

  //   public SingleRollerIOSpark(int canId, double reduction) {
  //     spark = new SparkMax(canId, MotorType.kBrushless);
  //     encoder = spark.getEncoder();
  //     // spark.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //      // Configure drive motor
  //     var config = new SparkMaxConfig();
  //     config
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit()
  //         .voltageCompensation(12.0);
  //     config
  //         .encoder
  //         .positionConversionFactor()
  //         .velocityConversionFactor()
  //         .uvwMeasurementPeriod(10)
  //         .uvwAverageDepth(2);
  //     config
  //         .closedLoop
  //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //         .pidf(
  //             driveKp, 0.0,
  //             driveKd, 0.0);
  //     config
  //         .signals
  //         .primaryEncoderPositionPeriodMs(20)
  //         .primaryEncoderVelocityPeriodMs(20)
  //         .appliedOutputPeriodMs(20)
  //         .busVoltagePeriodMs(20)
  //         .outputCurrentPeriodMs(20);

  //     SparkUtil.tryUntilOk(
  //         spark,
  //         5,
  //         () ->
  //             spark.configure(
  //                 config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

  //     SparkUtil.tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
  //   }

  //   public void updateInputs(SingleRollerIOInputs inputs) {
  // /**
  //  *
  //     public boolean connected = false;

  //     public double positionRad = 0.0;
  //     public double velocityRadPerSec = 0.0;

  //     public double appliedVoltage = 0.0;
  //     public double supplyCurrentAmps = 0.0;
  //     public double temperatureCelsius = 0.0;
  //  *
  //  */

  //     inputs.connected = connectedDebounce.calculate(!SparkUtil.sparkStickyFault);

  //     SparkUtil.ifOk(spark, encoder::getPosition, (value) -> inputs.positionRad = value);
  //     SparkUtil.ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);

  //     SparkUtil.ifOk(spark, new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
  // (value) -> inputs. = value);

  //   }

  //   /** Run roller at set voltage */
  //   public void runVolts(double volts) {}

  //   /** Run roller at set position */
  //   public void runPosition(double positionRad) {}

  //   /** Stop roller */
  //   public void stop() {
  //     runVolts(0);
  //   }
}
