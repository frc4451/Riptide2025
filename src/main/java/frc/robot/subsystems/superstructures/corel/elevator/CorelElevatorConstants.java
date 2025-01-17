package frc.robot.subsystems.superstructures.corel.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.rollers.elevators.ElevatorConstraints;

public class CorelElevatorConstants {
  public static final PIDConstants pidConstants = new PIDConstants(1.0);
  public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1);
  public static final double reduction = 18.0 / 12.0;
  public static final double moi = 0.001;

  public static final double inchesPerRad = 3.0; // equal to radius

  public static final ElevatorConstraints elevatorConstraints =
      new ElevatorConstraints(0, CorelElevatorSetpoint.L4.setpointInches);

  public static final TrapezoidProfile.Constraints trapezoidConstraints =
      new TrapezoidProfile.Constraints(1.0 * inchesPerRad, 1.0 * inchesPerRad);
}
