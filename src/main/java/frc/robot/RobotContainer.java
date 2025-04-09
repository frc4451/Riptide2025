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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auto.Autos;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.AlignRoutines;
import frc.robot.commands.DriveCommands;
import frc.robot.field.Barge;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.field.ReefFaces;
import frc.robot.subsystems.blinkin.Blinkin;
import frc.robot.subsystems.blinkin.BlinkinIO;
import frc.robot.subsystems.blinkin.BlinkinIOSim;
import frc.robot.subsystems.blinkin.BlinkinState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.quest.QuestIO;
import frc.robot.subsystems.quest.QuestIOReal;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.shooter.ShooterModes;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.CommandCustomXboxController;
import frc.robot.util.PoseUtils;
import org.littletonrobotics.junction.Logger;

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
  private final Climber climber = new Climber();

  public final Blinkin blinkin;

  public final Quest quest;

  // Controller
  public final CommandCustomXboxController driverController = new CommandCustomXboxController(0);
  private final CommandCustomXboxController operatorController = new CommandCustomXboxController(1);
  private final CommandCustomXboxController resetController = new CommandCustomXboxController(4);

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
        quest = new Quest(new QuestIOReal());

        // not attached to the robot yet
        blinkin = new Blinkin(new BlinkinIO() {});
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
        quest = new Quest(new QuestIO() {});
        blinkin = new Blinkin(new BlinkinIOSim());

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
        quest = new Quest(new QuestIO() {});
        blinkin = new Blinkin(new BlinkinIO() {});
        break;
    }

    autoChooser = new AutoChooser();
    autos = new Autos(drive, superStructure);

    configureAutos();
    configureHumanPlayerStation();

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureAutos() {
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
    autoChooser.addCmd("Quest Offset Calibration", () -> quest.calibrateCommand(drive));

    // autoChooser.addRoutine("2 Meters", autos::twoMeters);
    // autoChooser.addRoutine("3 Meters", autos::threeMeters);
    autoChooser.addRoutine("5 Meters", autos::fiveMeters);
    autoChooser.addRoutine("Allred L2", autos::allredL2);
    autoChooser.addRoutine("AllRight L4", autos::allRightL4);
    autoChooser.addRoutine("AllLeft L4", autos::allLeftL4);
    autoChooser.addRoutine("Ethan", autos::ethan);
    autoChooser.addRoutine("Callahan", autos::callahan);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureRotationModes();
    configurePoleBindings();
    configureCageBindings();
    configureSuperBindings();

    // if (Constants.currentMode == Constants.Mode.SIM) {
    debugSetup();
    // }
  }

  private void configureRotationModes() {
    // Default, auto-align to closest tracker
    drive.setDefaultCommand(
        DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftYSquared(),
                () -> -driverController.getLeftXSquared(),
                () -> BobotState.getCurrentAlignmentTracker().getRotationTarget())
            .unless(DriverStation::isAutonomous));

    // No auto-align, manual
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -driverController.getLeftYSquared(),
    //         () -> -driverController.getLeftXSquared(),
    //         () -> -driverController.getRightXSquared()));

    // Normal field-relative drive when overridden via a button
    driverController
        .leftTrigger()
        .or(BobotState.autoAlignEnabled().negate())
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftYSquared(),
                () -> -driverController.getLeftXSquared(),
                () -> -driverController.getRightXSquared()));
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -driverController.getLeftYSquared(),
    //         () -> -driverController.getLeftXSquared(),
    //         () -> -driverController.getRightXSquared()));

    // // Barge
    // driverController
    //     .x()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftYSquared(),
    //             () -> -driverController.getLeftXSquared(),
    //             () -> BobotState.getRotationToClosestBarge()));
  }

  /** All callbacks & binds related to the Coral Human Player Stations */
  private void configureHumanPlayerStation() {
    /* Auto Intake */
    {
      // This will automatically run the intake when the robot is close
      // to the human player station, taking some mental load off the drive team.
      BobotState.humanPlayerShouldThrow()
          .and(superStructure.isCoralIntaked().negate())
          .onTrue(superStructure.setShooterModeCommand(ShooterModes.INTAKE))
          .onFalse(superStructure.setShooterModeCommand(ShooterModes.NONE));

      // Automatically stop intake & signal to drive team once the coral is in the robot.
      // This will mean the drive team's coral pick up loop will look like:
      // 1) Get to the human player station
      //    - The intake will automatically turn on
      // 2) Wait for the human player to put in the coral
      // 3) The drive team will be signaled by rumble & lights that they have a coral
      //    - The intake will automatically turn off
      // 4) Drive away
      superStructure
          .isCoralIntaked()
          .onTrue(
              Commands.parallel(
                  superStructure.setShooterModeCommand(ShooterModes.NONE),
                  driverController.rumbleSeconds(1.0, 0.5),
                  operatorController.rumbleSeconds(1.0, 0.5)));
    }

    /*
     * Lights: Please sort the distance triggers closest to farthest,
     * this will make it easier to see the priority of the states at a glance
     */
    {
      blinkin.addConditionalState(
          BobotState.humanPlayerShouldThrow(), BlinkinState.HUMAN_PLAYER_SHOULD_THROW);

      blinkin.addConditionalState(superStructure.isCoralIntaked(), BlinkinState.CORAL_IN);
    }
  }

  private final Command alignmentRumble =
      Commands.deferredProxy(
          () ->
              Commands.parallel(
                  driverController.rumbleOnOff(1, 0.25, 0.25, 2),
                  operatorController.rumbleOnOff(1, 0.25, 0.25, 2)));

  // Three big booms
  private final Command climbRumble =
      Commands.deferredProxy(
          () ->
              Commands.parallel(
                  driverController.rumbleOnOff(1, 0.50, 0.25, 3),
                  operatorController.rumbleOnOff(1, 0.50, 0.25, 3)));

  private void configurePoleBindings() {
    // -- Coral --
    driverController
        .leftBumper()
        .and(() -> !BobotState.climbMode)
        .and(driverController.a().negate())
        .and(driverController.b().negate())
        .whileTrue(
            AlignRoutines.alignToPose(
                    drive,
                    () -> FieldUtils.getClosestReef().leftPole.getPose(),
                    () -> -driverController.getLeftYSquared())
                .withJoystickRumble(superStructure::getRumbleDistance, alignmentRumble));

    driverController
        .leftBumper()
        .and(() -> !BobotState.climbMode)
        .and(driverController.a())
        .whileTrue(
            AlignRoutines.positionToPole(
                    drive,
                    () -> FieldUtils.getClosestReef().leftPole,
                    superStructure::getReefOffset)
                .withJoystickRumble(alignmentRumble));

    // driverController
    //     .leftBumper()
    //     .and(driverController.b())
    //     .whileTrue(
    //         AlignRoutines.positionToPoleAndScore(
    //             drive,
    //             superStructure,
    //             () -> FieldUtils.getClosestReef().leftPole,
    //             superStructure::getReefOffset));

    driverController
        .rightBumper()
        .and(() -> !BobotState.climbMode)
        .and(driverController.a().negate())
        .and(driverController.b().negate())
        .whileTrue(
            AlignRoutines.alignToPose(
                    drive,
                    () -> FieldUtils.getClosestReef().rightPole.getPose(),
                    () -> -driverController.getLeftYSquared())
                .withJoystickRumble(superStructure::getRumbleDistance, alignmentRumble));

    driverController
        .rightBumper()
        .and(() -> !BobotState.climbMode)
        .and(driverController.a())
        .whileTrue(
            AlignRoutines.positionToPole(
                    drive,
                    () -> FieldUtils.getClosestReef().rightPole,
                    superStructure::getReefOffset)
                .withJoystickRumble(alignmentRumble));

    // driverController
    //     .rightBumper()
    //     .and(driverController.b())
    //     .whileTrue(
    //         AlignRoutines.positionToPoleAndScore(
    //             drive,
    //             superStructure,
    //             () -> FieldUtils.getClosestReef().rightPole,
    //             superStructure::getReefOffset));

    // -- Algae --
    driverController
        .rightTrigger()
        .and(() -> !BobotState.climbMode)
        .and(driverController.a().negate())
        .whileTrue(
            AlignRoutines.alignToPose(
                drive,
                () -> FieldUtils.getClosestReef().tag.pose().toPose2d(),
                () -> -driverController.getLeftYSquared()));

    driverController
        .rightTrigger()
        .and(() -> !BobotState.climbMode)
        .and(driverController.a())
        .and(superStructure::shouldGrabAlgae)
        .whileTrue(
            AlignRoutines.positionToPole(
                drive,
                () -> FieldUtils.getClosestReef().center,
                () -> FieldConstants.eventConstants.algaeOffset));

    // -- Human Player Station --
    driverController
        .a()
        .and(() -> !BobotState.climbMode)
        .and(driverController.leftBumper().negate())
        .and(driverController.rightBumper().negate())
        .and(driverController.rightTrigger().negate())
        .whileTrue(AlignRoutines.positionToHPSCenter(drive, () -> FieldUtils.getClosestHPS()));
  }

  public void configureCageBindings() {
    // Toggle Climber Mode
    driverController
        .back()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(() -> BobotState.climbMode = !BobotState.climbMode), climbRumble));

    driverController
        .x()
        .and(() -> BobotState.climbMode)
        .onTrue(
            Commands.parallel(
                climber.toggleExtend(), climber.deployTrayServo(), climber.deployHookServo()));

    // testing only
    // driverController.x().and(() -> BobotState.climbMode).onTrue(climber.toggleExtend());
    // driverController.y().and(() -> BobotState.climbMode).onTrue(climber.deployTrayServo());

    driverController
        .rightY()
        .and(() -> BobotState.climbMode)
        .whileTrue(climber.manualCommand(() -> driverController.getRightY() * 10.0, true));

    resetController
        .rightY()
        .and(() -> !DriverStation.isFMSAttached())
        .whileTrue(climber.manualCommand(() -> resetController.getRightY() * 10.0, false));

    driverController
        .leftBumper()
        .and(() -> BobotState.climbMode)
        .and(driverController.a().negate())
        .whileTrue(
            AlignRoutines.alignToPose(
                drive,
                () -> Barge.get().left.transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
                () -> driverController.getLeftYSquared()));

    driverController
        .leftBumper()
        .and(() -> BobotState.climbMode)
        .and(driverController.a())
        .whileTrue(AlignRoutines.positionToPose(drive, () -> Barge.get().left));

    driverController
        .rightBumper()
        .and(() -> BobotState.climbMode)
        .and(driverController.a().negate())
        .whileTrue(
            AlignRoutines.alignToPose(
                drive,
                () -> Barge.get().right.transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
                () -> driverController.getLeftYSquared()));

    driverController
        .rightBumper()
        .and(() -> BobotState.climbMode)
        .and(driverController.a())
        .whileTrue(AlignRoutines.positionToPose(drive, () -> Barge.get().right));

    driverController
        .rightTrigger()
        .and(() -> BobotState.climbMode)
        .and(driverController.a().negate())
        .whileTrue(
            AlignRoutines.alignToPose(
                drive,
                () -> Barge.get().center.transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
                () -> driverController.getLeftYSquared()));

    driverController
        .rightTrigger()
        .and(() -> BobotState.climbMode)
        .and(driverController.a())
        .whileTrue(AlignRoutines.positionToPose(drive, () -> Barge.get().center));
  }

  private void configureSuperBindings() {
    // -- Coral --
    operatorController
        .leftTrigger()
        .onTrue(superStructure.setShooterModeCommand(ShooterModes.INTAKE))
        .onFalse(superStructure.setShooterModeCommand(ShooterModes.NONE));

    operatorController
        .rightTrigger()
        .onTrue(superStructure.setShooterModeCommand(ShooterModes.SHOOT))
        .onFalse(superStructure.setShooterModeCommand(ShooterModes.NONE));

    operatorController.b().onTrue(superStructure.setModeCommand(SuperStructureModes.TUCKED));
    operatorController.a().onTrue(superStructure.setModeCommand(SuperStructureModes.L2Coral));
    operatorController.x().onTrue(superStructure.setModeCommand(SuperStructureModes.L3Coral));
    operatorController.y().onTrue(superStructure.setModeCommand(SuperStructureModes.L4Coral));

    // -- Algae --
    operatorController
        .leftBumper()
        .onTrue(superStructure.setShooterModeCommand(ShooterModes.ALGAE_INTAKING));

    operatorController
        .rightBumper()
        .onTrue(superStructure.setShooterModeCommand(ShooterModes.ALGAE_SHOOT))
        .onFalse(superStructure.setShooterModeCommand(ShooterModes.NONE));

    // spotless: off
    operatorController
        .povRight()
        .onTrue(superStructure.setModeCommand(SuperStructureModes.FLOOR_ALGAE));
    operatorController
        .povLeft()
        .onTrue(
            Commands.deferredProxy(
                () ->
                    superStructure.setModeCommand(
                        FieldUtils.getClosestReef().isL2Algae
                            ? SuperStructureModes.L2Algae
                            : SuperStructureModes.L3Algae)));
    operatorController.povUp().onTrue(superStructure.setModeCommand(SuperStructureModes.TUCKED_L4));
    operatorController.back().whileTrue(superStructure.scoreAlgae());
    // spotless: on
  }

  private void debugSetup() {
    String logRoot = "Debug/ChoreoWaypoints";

    // Calculating Reef Offsets for Choreo
    for (ReefFaces face : ReefFaces.values()) {
      Logger.recordOutput(
          logRoot + "/Faces/" + face.name() + "/Left/L2",
          PoseUtils.plusRotation(
              face.blue.leftPole.getPerpendicularOffsetPose(
                  FieldConstants.eventConstants.l2ReefOffset),
              Rotation2d.kPi));
      Logger.recordOutput(
          logRoot + "/Faces/" + face.name() + "/Left/L4",
          PoseUtils.plusRotation(
              face.blue.leftPole.getPerpendicularOffsetPose(
                  FieldConstants.eventConstants.l4ReefOffset),
              Rotation2d.kPi));
      Logger.recordOutput(
          logRoot + "/Faces/" + face.name() + "/Right/L2",
          PoseUtils.plusRotation(
              face.blue.rightPole.getPerpendicularOffsetPose(
                  FieldConstants.eventConstants.l2ReefOffset),
              Rotation2d.kPi));
      Logger.recordOutput(
          logRoot + "/Faces/" + face.name() + "/Right/L4",
          PoseUtils.plusRotation(
              face.blue.rightPole.getPerpendicularOffsetPose(
                  FieldConstants.eventConstants.l4ReefOffset),
              Rotation2d.kPi));
    }

    Logger.recordOutput(
        logRoot + "/HPS/Left",
        PoseUtils.getPerpendicularOffsetPose(
            FieldConstants.blueHPSDriverLeft.pose().toPose2d(),
            FieldConstants.eventConstants.l2ReefOffset));
    Logger.recordOutput(
        logRoot + "/HPS/Right",
        PoseUtils.getPerpendicularOffsetPose(
            FieldConstants.blueHPSDriverRight.pose().toPose2d(),
            FieldConstants.eventConstants.l2ReefOffset));
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
