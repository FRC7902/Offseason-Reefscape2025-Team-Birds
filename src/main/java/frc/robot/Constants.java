// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.vision.CameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.15;
  }

  public static class SwerveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(15);
  }

  public static class DriveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(15);
  }

  public static class PhotonConstants {
    public static final String leftCamName = "left";
    public static final Transform3d leftCamToRobotTsf = new Transform3d(0.207, 0.150, 0.567,
        new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(-4.333)));
    public static final CameraProperties leftCamProp = new CameraProperties(leftCamName, leftCamToRobotTsf, 640, 480,
        Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    public static final String rightCamName = "right";
    public static final Transform3d rightCamToRobotTsf = new Transform3d(0.207, -0.150, 0.567,
        new Rotation3d(Math.toRadians(0), Math.toRadians(30), Math.toRadians(4.333)));
    public static final CameraProperties rightCamProp = new CameraProperties(rightCamName, rightCamToRobotTsf, 640,
        480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);

    public static final String middleCamName = "middle";
    public static final Transform3d middleCamToRobotTsf = new Transform3d(0, 0, 0.35,
        new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0)));
    public static final CameraProperties middleCamProp = new CameraProperties(middleCamName, middleCamToRobotTsf, 640,
        480, Rotation2d.fromDegrees(100), 30, 0.25, 0.08);
  }

  public static class LimelightConstants {
    public static final String leftCamName = "left";
    public static double kStdDevs = 0.800000;
  }

  public static class VisionConstants {
    // Camera Offset

    // y distance = 299.8 mm (right left)
    // x dist = 269.87 mm (forward back)
    // ground = 272.94 mm

    public static final int[] acceptedTagIDs = new int[] { 2, 3, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

    public static final double kGroundToCamera = 0.27294; // meters
    public static final double kFowardToCamera = 0.26987; // meters
    public static final double kSidewaysToCamera = 0.2998; // meters
    public static final double kCameraRotation = 25.3; // degrees
    public static final String kCameraName = "limelight";

    public static double kLocalizationDisLim = 2;

    public static double kStdDevs = 0.800000;

    // //April Tag Offset
    // public static final double kGroundToAprilTagDistance = 0.171; //in meters
    // public static final double kAprilTagAreaLimit = 4.5;
    public static final double kAprilTagOffset = 0;

    // Reef Offset
    public static double leftReefToAprilTagOffset = -0.165000;
    public static double rightReefToAprilTagOffset = 0.210000;

    // PID Y Controller Constants
    public static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(60, 40);
    public static double yControllerTolerance = 0;
    public static double kPY = 0.07;
    public static double kIY = 0;
    public static double kDY = 0;

    public static double kPY2 = 6.0;
    public static double kIY2 = 0;
    public static double kDY2 = 0.5;

    public static int kSecondPIDControllerStartingPoint = 13; // to change

    // Contains the stored position of each April Tag on the field. This varies between seasons.
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  }

}
