package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotorSim leader;
  private final DCMotorSim follower;

  private final boolean invertFollower;

  private final ElevatorSim sim;
  private final ProfiledPIDController controller;
  private final ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  public ElevatorIOSim(
      DCMotor leaderModel,
      DCMotor followerModel,
      double reduction,
      double moi,
      boolean invertFollower) {
    leader =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leaderModel, moi, reduction), leaderModel);
    follower =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(followerModel, moi, reduction), followerModel);

    this.invertFollower = invertFollower;

    this.sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                ElevatorConstants.elevatorSimMotorConfig,
                ElevatorConstants.massKg,
                Units.inchesToMeters(ElevatorConstants.inchesPerRad),
                ElevatorConstants.reduction),
            ElevatorConstants.elevatorSimMotorConfig,
            Units.inchesToMeters(ElevatorConstants.elevatorConstraints.minHeightInches()),
            Units.inchesToMeters(ElevatorConstants.elevatorConstraints.maxHeightInches()),
            true,
            Units.inchesToMeters(ElevatorConstants.startHeightInches));

    this.controller =
        new ProfiledPIDController(
            ElevatorConstants.kP, 0, ElevatorConstants.kD, ElevatorConstants.trapezoidConstraints);

    this.feedforward =
        new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

    this.isClosedLoop = true;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    }

    inputs.connected = true;
    leader.update(Constants.loopPeriodSecs);

    inputs.leaderAppliedVoltage = appliedVolts;
    inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.leaderTorqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.leaderTemperatureCelsius = 0.0;

    inputs.followerAppliedVoltage = appliedVolts;
    inputs.followerSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.followerTorqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.followerTemperatureCelsius = 0.0;

    if (this.isClosedLoop) {
      appliedVolts =
          controller.calculate(Units.metersToInches(sim.getPositionMeters()))
              + feedforward.calculate(controller.getSetpoint().position);
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.positionInches = Units.metersToInches(sim.getPositionMeters());
    // inputs.velocityInchesPerSec = sim.getVelocityMetersPerSecond() * 39.3701;
    inputs.velocityInchesPerSec = Units.metersToInches(sim.getVelocityMetersPerSecond());

    inputs.positionGoalInches = controller.getGoal().position;
    inputs.positionSetpointInches = controller.getSetpoint().position;
    inputs.positionErrorInches = controller.getPositionError();
  }

  @Override
  public void runVolts(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double positionInches) {
    isClosedLoop = true;
    controller.reset(positionInches);
  }

  @Override
  public void setPositionGoal(double positionInches) {
    isClosedLoop = true;
    controller.setGoal(positionInches);
  }
}
