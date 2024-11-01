// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.function.Supplier;

/** This class represents the implementation of AprilTagVisionIO using Limelight camera. */
public class AprilTagVisionIOLimelight implements AprilTagVisionIO {

  private String limelightName;
  private Supplier<Rotation2d> driveHeadingSupplier;
  private Supplier<Double> gyroRateDegrees;
  private ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

  /**
   * Constructs a new AprilTagVisionIOLimelight instance.
   *
   * @param identifier The identifier of the Limelight camera.
   */
  public AprilTagVisionIOLimelight(
      String limelightName,
      Supplier<Rotation2d> driveHeadingSupplier,
      Supplier<Double> gyroRateDegrees) {
    this.limelightName = limelightName;
    this.driveHeadingSupplier = driveHeadingSupplier;
    this.gyroRateDegrees = gyroRateDegrees;
  }

  /**
   * Updates the inputs for AprilTag vision.
   *
   * @param inputs The AprilTagVisionIOInputs object containing the inputs.
   */
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    poseEstimates.clear();
    LimelightHelpers.SetRobotOrientation(
        limelightName, driveHeadingSupplier.get().getDegrees(), 0, 0, 0, 0, 0);

    PoseEstimate mt =
        DriverStation.isEnabled()
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    if (Math.abs(gyroRateDegrees.get()) > 80 || mt == null || mt.tagCount == 0) {
      inputs.poseEstimates = poseEstimates;
      return;
    }

    // Creates a new PoseEstimate object and adds it to the poseEstimates ArrayList
    poseEstimates.add(mt);

    // Assigns the poseEstimates ArrayList to the inputs.poseEstimates variable
    inputs.poseEstimates = poseEstimates;
  }
}
