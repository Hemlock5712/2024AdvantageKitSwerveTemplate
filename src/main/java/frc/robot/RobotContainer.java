// Copyright 2021-2024 FRC 6328
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
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ColorSensorTester;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.ColorSensor.ColorSensor;
import frc.robot.subsystems.ColorSensor.ColorSensorIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonVisionSIM;
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
  private final AprilTagVision aprilTagVision;
  private final ShooterSubsystem shooter;
  // private final Flywheel flywheel;

  private final Intake intake;

  // Controller
  private final XboxController controller = new XboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardNumber flywheelSpeedInput =
  //     new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));

        aprilTagVision = new AprilTagVision(new AprilTagVisionIOLimelight("limelight"));

        shooter = new ShooterSubsystem(new ShooterIOSparkMax());
        // flywheel = new Flywheel(new FlywheelIOTalonFX());

        intake = new Intake(new IntakeIOTalonFX());
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
        aprilTagVision =
            new AprilTagVision(
                new AprilTagVisionIOPhotonVisionSIM(
                    "photonCamera1",
                    new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
                    drive::getPose));
        // flywheel = new Flywheel(new FlywheelIOSim());
        shooter = new ShooterSubsystem(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOTalonFX());

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
        aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
        // flywheel = new Flywheel(new FlywheelIO() {});
        shooter = new ShooterSubsystem(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOTalonFX());

        break;
    }

    // Set up named commands for PathPlanner
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //         () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
    // flywheel).withTimeout(5.0));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("S Curve", AutoBuilder.buildAuto("Example Auto"));
    autoChooser.addOption(
        "color sensor", new ColorSensorTester(new ColorSensor(new ColorSensorIOReal())));
    // Set up FF characterization routines
    autoChooser.addDefaultOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runCharacterizationVolts,
    // flywheel::getCharacterizationVelocity));

    // Configure the button bindings
    aprilTagVision.setDataInterfaces(drive::addVisionData);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    //    controller
    //        .b() // Button B to reset rotation part of robot pose
    //        .onTrue(
    //            Commands.runOnce(
    //                    () ->
    //                        drive.setPose(
    //                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                    drive)
    //                .ignoringDisable(true));
    controller
        .start()
        .onTrue(
            Commands.runOnce(() -> drive.setAutoStart(aprilTagVision.getRobotPose()), drive)
                .ignoringDisable(true));
    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
    controller
        .b()
        .whileTrue(Commands.startEnd(() -> shooter.runVolts(12.0 * .99), shooter::stop, shooter));
//    new JoystickButton(controller, XboxController.Button.kX.value)
//            .onTrue(Commands.run(drive::stopWithX, drive));
//    new JoystickButton(controller, XboxController.Button.kB.value)
//            .onTrue(Commands.runOnce(
//                            () ->
//                                    drive.setPose(
//                                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
//                            drive)
//                    .ignoringDisable(true));
//    new JoystickButton(controller, XboxController.Button.kStart.value)
//            .onTrue(Commands.runOnce(() -> drive.setAutoStart(aprilTagVision.getRobotPose()), drive)
//                    .ignoringDisable(true));
//    new JoystickButton(controller, XboxController.Button.kA.value)
//            .whileTrue(Commands.startEnd( () -> intake.setVoltage(IntakeConstants.INTAKE_VOLTAGE),
//                            intake::stop, intake));
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
