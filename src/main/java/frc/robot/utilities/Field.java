// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Manipulator;

/** Add your docs here. */
public class Field {
    //Robot probably 31" with bumpers

    //Community -> Loading
    private final Pose2d[] BlueCommunityColumnInitial = {
        new Pose2d(2.0871258795062873, 0.512826, new Rotation2d(Math.PI)), //54.25+(31/2)*sqrt(2)+6 inches to meters for x value
        new Pose2d(2.0871258795062873, 1.071626, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 1.630426, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 2.189226, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 2.748026, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 3.306826, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 3.865626, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 4.424426, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 4.983226, new Rotation2d(Math.PI)) 
    };

    private final Pose2d[] BlueCommunityColumnFinal = {
        new Pose2d(1.77165, 0.512826, new Rotation2d(Math.PI)), //54.25+(31/2) inches to meters for x value
        new Pose2d(1.77165, 1.071626, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 1.630426, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 2.189226, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 2.748026, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 3.306826, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 3.865626, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 4.424426, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 4.983226, new Rotation2d(Math.PI)) 
    };
    //Loading -> Community
    private final Pose2d[] RedCommunityColumnInitial = {
        new Pose2d(2.0871258795062873, 3.020568, new Rotation2d(0)), //118.92 inches
        new Pose2d(2.0871258795062873, 3.579368, new Rotation2d(0)), //140.92 inches
        new Pose2d(2.0871258795062873, 4.138168, new Rotation2d(0)), //162.92 inches
        new Pose2d(2.0871258795062873, 4.696968, new Rotation2d(0)), //184.92 inches
        new Pose2d(2.0871258795062873, 5.255768, new Rotation2d(0)), //206.92 inches
        new Pose2d(2.0871258795062873, 5.814568, new Rotation2d(0)), //228.92 inches
        new Pose2d(2.0871258795062873, 6.373368, new Rotation2d(0)), //250.92 inches
        new Pose2d(2.0871258795062873, 6.932168, new Rotation2d(0)), //272.92 inches
        new Pose2d(2.0871258795062873, 7.490968, new Rotation2d(0))  //294.92 inches
    };

    private final Pose2d[] RedCommunityColumnFinal = {
        new Pose2d(1.77165, 3.020568, new Rotation2d(0)), 
        new Pose2d(1.77165, 3.579368, new Rotation2d(0)), 
        new Pose2d(1.77165, 4.138168, new Rotation2d(0)), 
        new Pose2d(1.77165, 4.696968, new Rotation2d(0)), 
        new Pose2d(1.77165, 5.255768, new Rotation2d(0)), 
        new Pose2d(1.77165, 5.814568, new Rotation2d(0)), 
        new Pose2d(1.77165, 6.373368, new Rotation2d(0)), 
        new Pose2d(1.77165, 6.932168, new Rotation2d(0)), 
        new Pose2d(1.77165, 7.490968, new Rotation2d(0)) 
    };

    //Bottom/Top refers to height relative to y-axis
    private final Pose2d[] BlueStationInitial = {
        new Pose2d(2.148713, 2.130489, new Rotation2d(0)), //Community bottom
        new Pose2d(2.148713, 2.748026, new Rotation2d(0)), //Same y as column 5
        new Pose2d(2.148713, 3.365563, new Rotation2d(0)), //Community top
        new Pose2d(4.855718, 2.130489, new Rotation2d(Math.PI)), //Field bottom
        new Pose2d(4.855718, 2.748026, new Rotation2d(Math.PI)),
        new Pose2d(4.855718, 3.365563, new Rotation2d(Math.PI))  //Field top
    };
    
    private final Pose2d[] BlueStationFinal = {
        new Pose2d(3.8354, 2.130489, new Rotation2d(0)), //Always faces away from communities, this could cause issues
        new Pose2d(3.8354, 2.748026, new Rotation2d(0)),
        new Pose2d(3.8354, 3.365563, new Rotation2d(0))  //"Highest" position (on y-axis)
    };

    private final Pose2d[] RedStationInitial = {
        new Pose2d(2.148713, 4.646867, new Rotation2d(0)), //Community bottom
        new Pose2d(2.148713, 5.264404, new Rotation2d(0)), //Same y as column 5
        new Pose2d(2.148713, 5.881941, new Rotation2d(0)), //Community top
        new Pose2d(4.855718, 4.646867, new Rotation2d(Math.PI)), //Field bottom
        new Pose2d(4.855718, 5.264404, new Rotation2d(Math.PI)),
        new Pose2d(4.855718, 5.881941, new Rotation2d(Math.PI))  //Field top
    };
    
    private final Pose2d[] RedStationFinal = {
        new Pose2d(3.8354, 4.646867, new Rotation2d(0)), //Always faces away from communities, this could cause issues
        new Pose2d(3.8354, 5.264404, new Rotation2d(0)), //Values found by adding loading zone width (99.07 inches) to Blue values
        new Pose2d(3.8354, 5.881941, new Rotation2d(0))  //"Highest" position (on y-axis)
    };

    private final Pose2d[] AprilTagsBlue = {

        new Pose2d(15.51356, 1.071626, new Rotation2d(0)),
        new Pose2d(15.51356, 2.748026, new Rotation2d(0)),
        new Pose2d(15.51356, 4.424426, new Rotation2d(0)),
        new Pose2d(16.17878, 6.749796, new Rotation2d(0)),
        new Pose2d(0.36195, 6.749796, new Rotation2d(Math.PI)),
        new Pose2d(1.02743, 4.424426, new Rotation2d(Math.PI)),
        new Pose2d(1.02743, 2.748026, new Rotation2d(Math.PI)),
        new Pose2d(1.02743, 1.071626, new Rotation2d(Math.PI))
    };
    
    private final Pose2d[] AprilTagsRed = {
        new Pose2d(1.02743, 6.932168, new Rotation2d(Math.PI)),
        new Pose2d(1.02743, 5.255768, new Rotation2d(Math.PI)),
        new Pose2d(1.02743, 3.579368, new Rotation2d(Math.PI)),
        new Pose2d(0.36195, 1.253998, new Rotation2d(Math.PI)),
        new Pose2d(16.17878, 1.253998, new Rotation2d(0)),
        new Pose2d(15.51356, 3.579368, new Rotation2d(0)),
        new Pose2d(15.51356, 5.255768, new Rotation2d(0)),
        new Pose2d(15.51356, 6.932168, new Rotation2d(0))
    };

    private AllianceSelection alliance;

    public Field(AllianceSelection alliance){
        this.alliance = alliance;
    }

    /**
	 * Gets the initial column position
	 * 
	 * @param column The column that will be returned (1-9)
	 */
    public Pose2d getInitialColumn(int column) {
        if(column < 10 && column > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueCommunityColumnInitial[column-1];
            }
            else {
                return RedCommunityColumnInitial[column-1];
            }
        } else {
            return null;
        }
    }

    /**
	 * Gets the final column position
	 * 
	 * @param column The column that will be returned (1-9)
	 */
    public Pose2d getFinalColumn(int column) {
        if(column < 10 && column > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueCommunityColumnFinal[column-1];
            }
            else {
                return RedCommunityColumnFinal[column-1];
            }
        } else {
            return null;
        }
    }

    /**
	 * Gets the position to approach the station from
	 * 
	 * @param position 1-3 Lowest-Highest Communtiy Side | 4-6 Lowest-Highest Field Side
	 */
    public Pose2d getStationInitial(int position){
        if(position < 7 && position > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueStationInitial[position-1];
            }
            else {
                return RedStationInitial[position-1];
            }
        } else {
            return null;
        }
    }

    /**
	 * Gets the center positions on the station
	 * 
	 * @param position 1-3 Lowest-Hightest (Y-Axis)
	 */
    public Pose2d getStationCenter(int position){
        if(position < 4 && position > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueStationFinal[position-1];
            }
            else {
                return RedStationFinal[position-1];
            }
        } else {
            return null;
        }
    }

    /**
	 * Gets the position of a specified April Tag (1-8)
	 * 
	 * @param position
	 */
    public Pose2d getAprilTag(int ID){
        if(ID < 9 && ID > 0) {
            if(alliance.getAlliance() == Alliance.Blue) {
                return AprilTagsBlue[ID-1];
            } else {
                return AprilTagsRed[ID-1];
            }
        } else {
            return null;
        }
    }

    public Pose2d getClosestGoal(DriveTrain drivetrain, Manipulator manipulator){
        Pose2d closestGoal;
        double robotY = drivetrain.getPose().getY();
        if(alliance.getAlliance() == Alliance.Blue){//Alliance Blue
            closestGoal = BlueCommunityColumnFinal[0];
            if(manipulator.getPistonExtended()){//Carrying Cone
                for(int i = 0; i < 9; i++){
                    if(i == 2 || i == 5 || i == 8){
                        continue;
                    }
                    if(Math.abs(robotY - BlueCommunityColumnFinal[i].getY()) < Math.abs(robotY - closestGoal.getY())){
                        closestGoal = BlueCommunityColumnFinal[i];
                    }
                }
            } else {
                for(int i = 0; i < 9; i++){
                    if(i != 2 && i != 5 && i != 8){
                        continue;
                    }
                    if(Math.abs(robotY - BlueCommunityColumnFinal[i].getY()) < Math.abs(robotY - closestGoal.getY())){
                        closestGoal = BlueCommunityColumnFinal[i];
                    }
                }
            }
        } else {
            closestGoal = RedCommunityColumnFinal[0];
            if(manipulator.getPistonExtended()){//Carrying Cone
                for(int i = 0; i < 9; i++){
                    if(i == 2 || i == 5 || i == 8){
                        continue;
                    }
                    if(Math.abs(robotY - RedCommunityColumnFinal[i].getY()) < Math.abs(robotY - closestGoal.getY())){
                        closestGoal = RedCommunityColumnFinal[i];
                    }
                }
            } else {
                for(int i = 0; i < 9; i++){
                    if(i != 2 && i != 5 && i != 8){
                        continue;
                    }
                    if(Math.abs(robotY - RedCommunityColumnFinal[i].getY()) < Math.abs(robotY - closestGoal.getY())){
                        closestGoal = RedCommunityColumnFinal[i];
                    }
                }
            }
        }
        return closestGoal;
    }
}
