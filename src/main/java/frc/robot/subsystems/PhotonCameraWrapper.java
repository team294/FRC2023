package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

import java.io.IOException;
import java.util.Optional;

import javax.lang.model.util.Elements.Origin;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Field field;
  private FileLog log;
  private boolean hasInit = false;
  private Alliance currAlliance = Alliance.Invalid;

  public PhotonCameraWrapper(Field field, FileLog log) {
    this.log = log;
    this.field = field;
  }

  public void init() {
    log.writeLog(true, "PhotonCameraWrapper", "Init", "Starting");

    currAlliance = field.getAlliance();

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(VisionConstants.cameraName);
    }


    //  aprilTagFieldLayout = field.getAprilTagFieldLayout();

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      updateAlliance();
      log.writeLog(true, "PhotonCameraWrapper", "Init", "Loaded april tags from file");
    } catch (IOException e) {
      log.writeLog(true, "PhotonCameraWrapper", "Init", "Error loading april tags from file");
      e.printStackTrace();
    }

    // Create pose estimator
    photonPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        photonCamera,
        VisionConstants.robotToCam);

    hasInit = true;

    log.writeLog(true, "PhotonCameraWrapper", "Init", "Done");
  }

  public boolean hasInit() {
    return hasInit;
  }
/**
 * updates the current alliance and apriltag field layout
 */
public void updateAlliance() {
  if (field.getAlliance() != currAlliance) {
    currAlliance = field.getAlliance();
    switch (currAlliance) {
      case Blue:
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        break;
      case Red:
        aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        break;
      default:
        log.writeLog(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance invalid");
        break;
    }
    log.writeLog(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance changed", currAlliance);
    aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }
}

public void periodic() {
  updateAlliance();
}

  /**
  * @param estimatedRobotPose The current best guess at robot pose
  * @return A pair of the fused camera observations to a single Pose2d on the
  *         field, and the time
  *         of the observation. Assumes a planar field and the robot is always
  *         firmly on the ground
  */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var newPoseOptional = photonPoseEstimator.update();
    if (newPoseOptional.isPresent()) {
      // log.writeLog(true, "PhotonCameraWrapper", "getEstimatedGlobalPose", "PreviousPose", "X",prevEstimatedRobotPose.getX(),"Y",prevEstimatedRobotPose.getY());
      EstimatedRobotPose newPose = newPoseOptional.get();
      // log.writeLog(true, "PhotonCameraWrapper", "getEstimatedGlobalPose", "NewPose", "X",newPose.estimatedPose.getX(),"Y",newPose.estimatedPose.getY());
    }
    return newPoseOptional;
  }
}