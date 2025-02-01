package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveCommandConstants {
  public static ProfiledPIDController makeAngleController() {
    ProfiledPIDController angleController =
        new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return angleController;
  }

  public static PIDController makeTranslationController() {
    return new PIDController(5.0, 0.0, 0.0);
  }
}
