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
    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Starting");

    currAlliance = field.getAlliance();

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(VisionConstants.cameraName);
    }


  //  aprilTagFieldLayout = field.getAprilTagFieldLayout();

  try {
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    updateAlliance();
    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Loaded april tags from file");
  } catch (IOException e) {
    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Error loading april tags from file");
    e.printStackTrace();
  }

  log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Loaded april tags from field");
  log.writeLogEcho(true, "PhotonCameraWrapper", "Init", 
    "AT8 x",aprilTagFieldLayout.getTagPose(8).get().getX(),
    "AT8 y",aprilTagFieldLayout.getTagPose(8).get().getY(),
    "AT8 rot",aprilTagFieldLayout.getTagPose(8).get().getRotation());

    // Create pose estimator
    photonPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        photonCamera,
        VisionConstants.robotToCam);

    hasInit = true;

    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Done");
  }

  public boolean hasInit() {
    return hasInit;
  }

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
        log.writeLog(true, "PhotonCameraWrapper", "UpdateAlliance", "Alliance Invalid");
        break;
    }
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
      log.writeLog(true, "PhotonCameraWrapper", "getEstimatedGlobalPose", "PreviousPose", "X",prevEstimatedRobotPose.getX(),"Y",prevEstimatedRobotPose.getY());
      EstimatedRobotPose newPose = newPoseOptional.get();
      log.writeLog(true, "PhotonCameraWrapper", "getEstimatedGlobalPose", "NewPose", "X",newPose.estimatedPose.getX(),"Y",newPose.estimatedPose.getY());
    }
    return newPoseOptional;
  }
}