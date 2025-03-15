package frc.robot.subsystems.superstructure.elevatorV2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.util.Gains;

public class ElevatorV2IOProfiledSim implements ElevatorV2IO {
  private final ElevatorSim elevatorSim;
  private final ProfiledPIDController controller;
  private final ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  private final double inchesPerRad;

  public ElevatorV2IOProfiledSim(
      DCMotor motorSystem,
      double reduction,
      double inchesPerRad,
      double carriageMassKg,
      double minHeightMeters,
      double maxHeightMeters,
      boolean simulateGravity,
      double startingHeight,
      Gains gains,
      TrapezoidProfile.Constraints constraints) {
    elevatorSim =
        new ElevatorSim(
            motorSystem,
            reduction,
            carriageMassKg,
            Units.inchesToMeters(inchesPerRad),
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeight);

    controller =
        new ProfiledPIDController(
            gains.kP(),
            0,
            gains.kD(),
            new Constraints(constraints.maxVelocity, constraints.maxAcceleration));

    feedforward = new ElevatorFeedforward(gains.kS(), gains.kG(), gains.kV());

    this.appliedVolts = 0.0;
    this.isClosedLoop = true;

    this.inchesPerRad = inchesPerRad;
  }

  @Override
  public void updateInputs(ElevatorV2IOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    }

    if (isClosedLoop) {
      appliedVolts =
          controller.calculate(Units.metersToInches(elevatorSim.getPositionMeters()))
              + feedforward.calculate(controller.getSetpoint().position);
    }

    inputs.connected = true;

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(Constants.loopPeriodSecs);

    inputs.leaderPositionRad =
        Units.metersToInches(elevatorSim.getPositionMeters()) / this.inchesPerRad;
    inputs.leaderAppliedVoltage = appliedVolts;
    inputs.leaderSupplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.leaderTorqueCurrentAmps = elevatorSim.getCurrentDrawAmps();

    inputs.followerPositionRad =
        Units.metersToInches(elevatorSim.getPositionMeters()) / this.inchesPerRad;
    inputs.followerAppliedVoltage = appliedVolts;
    inputs.followerSupplyCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.followerTorqueCurrentAmps = elevatorSim.getCurrentDrawAmps();

    inputs.positionGoalInches = controller.getGoal().position;
    inputs.positionSetpointInches = controller.getSetpoint().position;
    inputs.positionErrorInches = controller.getPositionError();
  }

  @Override
  public void runVolts(double volts) {
    this.isClosedLoop = false;
    this.appliedVolts = volts;
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }

  @Override
  public void resetPosition(double positionRad) {
    setPosition(positionRad);
  }

  @Override
  public void setPosition(double positionRad) {
    this.elevatorSim.setState(Units.inchesToMeters(positionRad * inchesPerRad), 0);
  }

  @Override
  public void setPositionGoalInches(double positionInches) {
    this.isClosedLoop = true;
    this.controller.setGoal(positionInches);
  }

  @Override
  public double getPositionInches() {
    return Units.metersToInches(this.elevatorSim.getPositionMeters());
  }

  @Override
  public double getPositionGoalInches() {
    return this.controller.getGoal().position;
  }
}
