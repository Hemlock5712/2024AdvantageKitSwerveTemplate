// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class AprilTagVisionIOPhotonVisionSIM implements AprilTagVisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  private ArrayList<PoseEstimate> poseEstimates = new ArrayList<>();

  private double lastEstTimestamp = 0;
  private final Supplier<Pose2d> poseSupplier;

  /**
   * Constructs a new AprilTagVisionIOPhotonVisionSIM instance.
   *
   * @param identifier The identifier of the PhotonCamera.
   * @param robotToCamera The transform from the robot's coordinate system to the camera's
   *     coordinate system.
   * @param poseSupplier The supplier of the robot's pose.
   */
  public AprilTagVisionIOPhotonVisionSIM(
      String identifier, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    camera = new PhotonCamera(identifier);
    photonEstimator =
        new PhotonPoseEstimator(
            FieldConstants.aprilTags,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    // Create the vision system simulation which handles cameras and targets on the
    // field.
    visionSim = new VisionSystemSim("main");
    // Add all the AprilTags inside the tag layout as visible targets to this
    // simulated field.
    visionSim.addAprilTags(FieldConstants.aprilTags);
    // Create simulated camera properties. These can be set to mimic your actual
    // camera.
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);
    // Create a PhotonCameraSim which will update the linked PhotonCamera's values
    // with visible
    // targets.
    cameraSim = new PhotonCameraSim(camera, cameraProp);
    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableDrawWireframe(true);
    this.poseSupplier = poseSupplier;
  }

  /**
   * Updates the inputs for AprilTag vision.
   *
   * @param inputs The AprilTagVisionIOInputs object containing the inputs.
   */
  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    poseEstimates.clear();
    PhotonPipelineResult results = cameraSim.getCamera().getLatestResult();
    double timestamp = results.getTimestampSeconds();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    if (!results.targets.isEmpty() && allianceOptional.isPresent()) {
      double latencyMS = results.getLatencyMillis();
      Pose3d poseEstimation;
      Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
      if (estimatedPose.isEmpty()) {
        return;
      }
      poseEstimation = estimatedPose.get().estimatedPose;
      double averageTagDistance = 0.0;
      timestamp -= (latencyMS / 1e3);
      int[] tagIDs = new int[results.targets.size()];
      for (int i = 0; i < results.targets.size(); i++) {
        tagIDs[i] = results.targets.get(i).getFiducialId();
        var tagPose = photonEstimator.getFieldTags().getTagPose(tagIDs[i]);
        if (tagPose.isEmpty()) {
          continue;
        }
        averageTagDistance +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(poseEstimation.getTranslation().toTranslation2d());
      }
      averageTagDistance /= tagIDs.length;
      RawFiducial[] rawFiducials =
          Arrays.stream(tagIDs)
              .mapToObj(id -> new RawFiducial((int) id, 0, 0, 0, 0, 0, 0))
              .toArray(RawFiducial[]::new);
      PoseEstimate poseEstimate =
          new PoseEstimate(
              poseEstimation.toPose2d(),
              timestamp,
              0,
              tagIDs.length,
              0,
              averageTagDistance,
              0,
              rawFiducials,
              DriverStation.isEnabled());
      poseEstimates.add(poseEstimate);
    }
    inputs.poseEstimates = poseEstimates;
  }

  /** Updates the PhotonPoseEstimator and returns the estimated global pose. */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    visionEst.ifPresentOrElse(
        est ->
            getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
        });
    if (newResult) lastEstTimestamp = latestTimestamp;
    return visionEst;
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!RobotBase.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
