// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.Autos;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DrivePerpendicularToPoseCommand;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.quest.QuestIO;
import frc.robot.subsystems.quest.QuestIOReal;
import frc.robot.subsystems.quest.QuestSubsystem;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.CommandCustomXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Vision vision = new Vision();
  private final SuperStructure superStructure = new SuperStructure();

  public final QuestSubsystem quest;

  // Controller
  public final CommandCustomXboxController driverController = new CommandCustomXboxController(0);
  private final CommandCustomXboxController operatorController = new CommandCustomXboxController(1);

  // Dashboard inputs
  private final AutoChooser autoChooser;
  private final Autos autos;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new BobotState();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        quest = new QuestSubsystem(new QuestIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        quest = new QuestSubsystem(new QuestIO() {});
        break;

      case REPLAY:
      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        quest = new QuestSubsystem(new QuestIO() {});
        break;
    }

    autoChooser = new AutoChooser();
    autos = new Autos(drive, quest);

    configureAutos();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureAutos() {
    // Set up SysId routines
    autoChooser.addCmd(
        "Drive Wheel Radius Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addCmd(
        "Drive Simple FF Characterization", () -> DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Forward)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Quasistatic Reverse)",
        () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Forward)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd(
        "Drive SysId (Dynamic Reverse)", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addRoutine("2 Meters", autos::twoMeters);
    autoChooser.addRoutine("3 Meters", autos::threeMeters);
    autoChooser.addRoutine("5 Meters", autos::fiveMeters);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftYSquared(),
            () -> -driverController.getLeftXSquared(),
            () -> -driverController.getRightXSquared()));

    configureAlignmentBindings();
    configureSuperBindings();
  }

  private void configureAlignmentBindings() {
    // Coral
    driverController
        .a()
        .and(driverController.leftBumper().negate())
        .and(driverController.rightBumper().negate())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftYSquared(),
                () -> -driverController.getLeftXSquared(),
                () -> BobotState.getRotationToClosestReef()));

    driverController
        .a()
        .and(driverController.leftBumper())
        .whileTrue(
            DrivePerpendicularToPoseCommand.withJoystickRumble(
                drive,
                () -> FieldUtils.getClosestReef().leftPole.getPose(),
                () -> -driverController.getLeftYSquared(),
                Commands.parallel(
                    driverController.rumbleOnOff(1, 0.25, 0.25, 2),
                    operatorController.rumbleOnOff(1, 0.25, 0.25, 2))));

    driverController
        .a()
        .and(driverController.rightBumper())
        .whileTrue(
            DrivePerpendicularToPoseCommand.withJoystickRumble(
                drive,
                () -> FieldUtils.getClosestReef().rightPole.getPose(),
                () -> -driverController.getLeftYSquared(),
                Commands.parallel(
                    driverController.rumbleOnOff(1, 0.25, 0.25, 2),
                    operatorController.rumbleOnOff(1, 0.25, 0.25, 2))));

    // Human Player Stations
    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftYSquared(),
                () -> -driverController.getLeftXSquared(),
                () -> BobotState.getRotationToClosestHPS()));

    // Barge
    driverController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftYSquared(),
                () -> -driverController.getLeftXSquared(),
                () -> BobotState.getRotationToClosestBarge()));

    superStructure
        .isCoralIntaked()
        .onTrue(
            Commands.parallel(
                driverController.rumbleSeconds(1.0, 0.5),
                operatorController.rumbleSeconds(1.0, 0.5)));
  }

  private void configureSuperBindings() {
    operatorController
        .rightBumper()
        .onTrue(superStructure.setModeCommand(SuperStructureModes.TUCKED));
    operatorController.a().onTrue(superStructure.setModeCommand(SuperStructureModes.L1));
    operatorController.x().onTrue(superStructure.setModeCommand(SuperStructureModes.L2));
    operatorController.b().onTrue(superStructure.setModeCommand(SuperStructureModes.L3));
    operatorController.y().onTrue(superStructure.setModeCommand(SuperStructureModes.L4));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
