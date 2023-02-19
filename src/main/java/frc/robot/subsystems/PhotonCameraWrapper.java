/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
        public PhotonCamera photonCamera;
        public PhotonPoseEstimator photonPoseEstimator;

        public PhotonCameraWrapper() {
                // Set up a test arena of two apriltags at the center of each driver station set
                final AprilTag tag03 = new AprilTag(8, new Pose3d(new Pose2d(1.02743, 1.1, new Rotation2d(0))));
                final AprilTag tag02 = new AprilTag(7, new Pose3d(new Pose2d(1.02743, 2.76, new Rotation2d(0))));
                final AprilTag tag01 = new AprilTag(6, new Pose3d(new Pose2d(1.02743, 4.42, new Rotation2d(0)))); 
                ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
                atList.add(tag03);
                atList.add(tag02);
                atList.add(tag01);
                
                
                // ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
                // atList.add(VisionConstants.tag01);
                // atList.add(VisionConstants.tag02);
                // atList.add(VisionConstants.tag03);
                // atList.add(VisionConstants.tag04);
                // atList.add(VisionConstants.tag05);
                // atList.add(VisionConstants.tag06);
                // atList.add(VisionConstants.tag07);
                // atList.add(VisionConstants.tag08);

                AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

                // TODO DOES NOT WORK PROPERLY IF ON RED ALLIANCE

                // Forward Camera
                photonCamera = new PhotonCamera(VisionConstants.cameraName); 

                // Create pose estimator
                photonPoseEstimator = new PhotonPoseEstimator(
                        aprilTagFieldLayout, 
                        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, 
                        photonCamera, 
                        VisionConstants.robotToCam);
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
                return photonPoseEstimator.update();
        }
}