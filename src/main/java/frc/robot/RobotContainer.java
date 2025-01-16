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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.dashboard.ReefTreeSelector;
import frc.robot.dashboard.ReefTreeSelector.ReefTree;
import frc.robot.state_machines.BobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.rollers.elevators.ElevatorIO;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.superstructures.corel.CorelSuperstructure;
import frc.robot.subsystems.superstructures.corel.conveyor.CorelConveyor;
import frc.robot.subsystems.superstructures.corel.conveyor.CorelConveyorIOSim;
import frc.robot.subsystems.superstructures.corel.elevator.CorelElevator;
import frc.robot.subsystems.superstructures.corel.elevator.CorelElevatorIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final PivotSubsystem pivotSubsystem;
  // private final ElevatorSubsystem elevatorSubsystem;
  private final CorelSuperstructure corelSuperstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final CommandJoystick flightJoystick = new CommandJoystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedDashboardChooser<Integer> faceChooser;

  private final LoggedDashboardChooser<ReefTree> reefChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // this is a no-op but is required for it to be registered as a VirtualSubsystem
    new BobotState();

    // Superstructure subsystems are only used in the RobotContainer constructor
    // and then passed to their respective Superstructure constructors.
    CorelElevator corelElevator = null;
    CorelConveyor corelConveyor = null;

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
        // pivotSubsystem = new PivotSubsystem(new PivotIO() {});
        // elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
        corelElevator = new CorelElevator(new ElevatorIO() {});
        corelConveyor = new CorelConveyor(new SingleRollerIO() {});
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
        // pivotSubsystem = new PivotSubsystem(new PivotIOSim());
        // elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
        corelElevator = new CorelElevator(new CorelElevatorIOSim());
        corelConveyor = new CorelConveyor(new CorelConveyorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // pivotSubsystem = new PivotSubsystem(new PivotIO() {});
        // elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
        corelElevator = new CorelElevator(new ElevatorIO() {});
        corelConveyor = new CorelConveyor(new SingleRollerIO() {});
        break;
    }

    // Assuming that the superstructure dependencies have been created, create the
    // superstructure.
    corelSuperstructure = new CorelSuperstructure(corelElevator, corelConveyor);

    faceChooser = new LoggedDashboardChooser<>("Face Selected");
    faceChooser.addOption("1", 1);
    faceChooser.addOption("2", 2);
    faceChooser.addOption("3", 3);

    reefChooser =
        new LoggedDashboardChooser<ReefTree>("Reef Selector", new SendableChooser<ReefTree>());
    reefChooser.addOption("A", ReefTreeSelector.A);
    reefChooser.addOption("B", ReefTreeSelector.B);
    reefChooser.addOption("C", ReefTreeSelector.C);
    reefChooser.addOption("D", ReefTreeSelector.D);
    reefChooser.addOption("E", ReefTreeSelector.E);
    reefChooser.addOption("F", ReefTreeSelector.F);
    reefChooser.addOption("G", ReefTreeSelector.G);
    reefChooser.addOption("H", ReefTreeSelector.H);
    reefChooser.addOption("I", ReefTreeSelector.I);
    reefChooser.addOption("J", ReefTreeSelector.J);
    reefChooser.addOption("K", ReefTreeSelector.K);
    reefChooser.addOption("L", ReefTreeSelector.L);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
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
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // elevatorSubsystem.setDefaultCommand(
    // // TODO
    // );

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    operatorController.povUp().onTrue(this.corelSuperstructure.raiseElevatorToL4());
    operatorController.povLeft().onTrue(this.corelSuperstructure.raiseElevatorToL3());
    operatorController.povRight().onTrue(this.corelSuperstructure.raiseElevatorToL2());
    operatorController.povDown().onTrue(this.corelSuperstructure.raiseElevatorToL1());

    operatorController
        .a()
        .and(this.corelSuperstructure.isElevatorMoving().negate())
        .whileTrue(this.corelSuperstructure.shootCorel());

    flightJoystick
        .button(12)
        .and(this.corelSuperstructure.isElevatorMoving().negate())
        .whileTrue(this.corelSuperstructure.shootCorel());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
