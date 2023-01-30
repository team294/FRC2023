// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Field {
    //Robot probably 31" with bumpers
    private final Pose2d BlueCommunityColumn1Initial = new Pose2d(1.64465, 0.512826, new Rotation2d(Math.PI)); //180 degrees
    private final Pose2d BlueCommunityColumn1Final = new Pose2d(1.16205, 0.512826, new Rotation2d(Math.PI)); //2' different between initial and final
    private final Pose2d BlueCommunityColumn2Initial = new Pose2d(1.33985, 1.071372, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn2Final = new Pose2d(1.16205, 1.071372, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn3Initial = new Pose2d(1.33985, 1.372997, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn3Final = new Pose2d(1.16205, 1.372997, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn4Initial = new Pose2d(1.33985, 1.976247, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn4Final = new Pose2d(1.16205, 1.976247, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn5Initial = new Pose2d(1.33985, 2.747772, new Rotation2d(Math.PI));//Double Check numbers, seems wrong
    private final Pose2d BlueCommunityColumn5Final = new Pose2d(1.16205, 2.747772, new Rotation2d(Math.PI));
}
