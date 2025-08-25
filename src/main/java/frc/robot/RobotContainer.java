// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.teleop.NullCommand;
import frc.robot.commands.teleop.SwerveCommands.StrafeLeftCommand;
import frc.robot.commands.teleop.SwerveCommands.StrafeRightCommand;
import frc.robot.commands.teleop.visions.AlignToReef;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.vision.CameraInterface;
import frc.robot.subsystems.vision.PhotonSim;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.ReefSide;
import swervelib.SwerveInputStream;
import frc.robot.commands.teleop.visions.AlignToReef;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // TODO: Initialize your DriveSubsystem here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(
      m_driverController,
      new File(Filesystem.getDeployDirectory(), "swerve"));

  public final PhotonSubsystem m_leftCamera = new PhotonSubsystem(PhotonConstants.leftCamProp);
  public final PhotonSubsystem m_rightCamera = new PhotonSubsystem(PhotonConstants.rightCamProp);
  public final PhotonSubsystem m_middleCamera = new PhotonSubsystem(PhotonConstants.middleCamProp);

  public PhotonSim m_cameraSim;
  public static final CameraInterface m_cameraSubsystem = new CameraInterface(VisionConstants.kCameraName);

  // private final CameraInterface leftCamera = new CameraInterface("quandale",
  // 0);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular
   * velocity.
   */
  public SwerveInputStream driveAngularVelocity = SwerveInputStream
      .of(m_swerveSubsystem.getSwerveDrive(), () -> m_driverController.getLeftY() * -1,
          () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
      .allianceRelativeControl(true);


  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
  .withControllerHeadingAxis(m_driverController::getRightX, m_driverController::getRightY)
  .headingWhile(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream
      .of(m_swerveSubsystem.getSwerveDrive(), () -> -m_driverController.getLeftY(),
          () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8)
      .allianceRelativeControl(true);
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  public SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
          () -> Math.sin(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2),
          () -> Math.cos(m_driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Robot.isSimulation()) {
      m_cameraSim = new PhotonSim(m_swerveSubsystem, m_leftCamera, m_middleCamera, m_rightCamera);
    }

    // Configure the trigger bindings
    configureBindings();
    m_swerveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
    new EventTrigger("autoalignleft")
        .whileTrue(new AlignToReef(m_cameraSubsystem, ReefSide.RIGHT, m_swerveSubsystem).withTimeout(2));
    new EventTrigger("autoalignright")
        .whileTrue(new AlignToReef(m_cameraSubsystem, ReefSide.LEFT, m_swerveSubsystem).withTimeout(2));
  }

  Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
  Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_driverController.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              m_driverController.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));
  Command driveFieldOrientedDirectAngleSim = m_swerveSubsystem.driveFieldOriented(driveDirectAngleSim);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Swerve drive controls
    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngleKeyboard = m_swerveSubsystem
        .driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = m_swerveSubsystem
        .driveFieldOriented(driveAngularVelocityKeyboard);

    // Default to field-centric swerve drive
    // m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // Zero gyro
    m_driverController.start().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));

    // Strafe controls
    // m_driverController.leftTrigger(0.05).whileTrue(new SequentialCommandGroup(new
    // CheckForAprilTag(0), new AlignToReef(this, 0)));
    // m_driverController.rightTrigger(0.05).whileTrue(new
    // SequentialCommandGroup(new CheckForAprilTag(1), new AlignToReef(this, 1)));

    m_driverController.a().whileTrue(new AlignToReef(m_cameraSubsystem, ReefSide.RIGHT, m_swerveSubsystem)); // left
    m_driverController.b().whileTrue(new AlignToReef(m_cameraSubsystem, ReefSide.LEFT, m_swerveSubsystem)); // right

    // m_driverController.leftTrigger(0.05).whileTrue(new StrafeLeftCommand());
    // m_driverController.rightTrigger(0.05).whileTrue(new StrafeRightCommand());

    // ======= Test bindings =======

    // Micro elevator adjustment
    // m_driverController.povUp().whileTrue(new
    // RelativeMoveElevatorCommand(0.00635));
    // m_driverController.povDown().whileTrue(new
    // RelativeMoveElevatorCommand(-0.00635));

    // Snap to angle
    // m_driverController.a().onTrue(drivebase.snapToAngle(90, 1));
    // m_driverController.b().onTrue(drivebase.snapToAngle(180, 1));
    // m_driverController.y().onTrue(drivebase.snapToAngle(0, 1));
    // m_driverController.x().onTrue(drivebase.snapToAngle(270, 1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
