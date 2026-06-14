package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
  private Drive m_driveSubsystem;
  /** Creates a new VisionSubsystem. */
  private final Field2d field2d = new Field2d();


  private final String leftCameraName = "leftCamera";
  private final String rightCameraName = "rightCamera";

  private final PhotonCamera leftCamera;
  private final PhotonCamera rightCamera;

  AprilTagFieldLayout fieldLayout;
  public static Pose2d aprilTag24Pose2d;
  

  private Transform3d leftCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(14), Units.inchesToMeters(11), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(15)));

  private Transform3d rightCameraTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(14), Units.inchesToMeters(-11), Units.inchesToMeters(0)),
          new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(-15)));


  public PhotonVisionSubsystem(Drive driveSubsystem) {

    leftCamera = new PhotonCamera(leftCameraName);
    rightCamera = new PhotonCamera(rightCameraName);

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    aprilTag24Pose2d = fieldLayout.getTagPose(24).get().toPose2d();
  }

  @Override
  public void periodic() {
    updateCameras();
  }

  public void updateCameras() {
    updateCamera(leftCamera, leftCameraTransform);
    updateCamera(rightCamera, rightCameraTransform);
  }

  private Pose2d updateCamera(PhotonCamera camera, Transform3d cameraTransform) {

    Pose2d latest = null;
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (var result : results) {
      if (result.hasTargets()) {
        PhotonTrackedTarget tar = result.getBestTarget();

        if (tar != null) {
          Transform3d c2t = tar.getBestCameraToTarget();

          Optional<Pose3d> tagPose = fieldLayout.getTagPose(tar.getFiducialId());

          if (tagPose.isEmpty() || tar.getPoseAmbiguity() > 0.15) {
            continue;
          }

          // Get current robot pose by comparing camera pose to tag pose and transforming with
          // camera transform.
          Pose2d p =
              PhotonUtils.estimateFieldToRobotAprilTag(
                      c2t, tagPose.get(), cameraTransform.inverse())
                  .toPose2d();

          // Get distance from robot to april tag
          Pose2d tag = tagPose.get().toPose2d();
          double distance = PhotonUtils.getDistanceToPose(p, tag);

          // Standard deviation of the measurement, based on distance. Tuned to be about 0.1 at 3
          // meters, and 0.5 at 10 meters.
          // smallert numbers mean more trust in the measurement, and larger numbers mean less
          // trust. This is used in the Kalman filter to weight the vision measurement against the
          // odometry measurement.
          double std = distance * distance * .07;

          // Update drive subsystem with vision measurement, including standard deviation for Kalman
          // filter.
          m_driveSubsystem.addVisionMeasurement(
              p, result.getTimestampSeconds(), VecBuilder.fill(std, std, std));
        }
      }
    }
    return latest;
  }
}
