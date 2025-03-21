package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoConstants;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.util.CommandCustomXboxController;

public class TeleopCommandFactory {
   public static Command autoAlignClosestReefPole(Drive drive, SuperStructure superStructure, CommandCustomXboxController driverController, CommandCustomXboxController operatorController) {
        return DrivePerpendicularToPoseCommand.rumbleWithinThreshold(
            drive,
            AutoConstants.useConstrainedPoseForReef,
            () -> FieldUtils.getClosestReef().rightPole.getPose(),
            () -> -driverController.getLeftYSquared(),
            () ->
                superStructure.isL4Coral()
                    ? AutoConstants.l4MinDistanceMeters
                    : AutoConstants.l2MinDistanceMeters,
            () ->
                superStructure.isL4Coral()
                    ? AutoConstants.l4MaxDistanceMeters
                    : AutoConstants.l2MaxDistanceMeters,
            Commands.parallel(
                driverController.rumbleSeconds(1, 0.25),
                operatorController.rumbleSeconds(1, 0.25)));
   }
}
