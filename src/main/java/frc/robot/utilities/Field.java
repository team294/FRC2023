// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class Field {
    //Robot probably 31" with bumpers
    private final Pose2d BlueCommunityColumn1Initial = new Pose2d(1.64465, 0.512826, new Rotation2d(Math.PI)); //180 degrees
    private final Pose2d BlueCommunityColumn1Final = new Pose2d(1.16205, 0.512826, new Rotation2d(Math.PI)); //2' different between initial and final
    private final Pose2d BlueCommunityColumn2Initial = new Pose2d(1.33985, 1.071372, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn2Final = new Pose2d(1.16205, 1.071372, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn3Initial = new Pose2d(1.33985, 1.629918, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn3Final = new Pose2d(1.16205, 1.629918, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn4Initial = new Pose2d(1.33985, 2.189226, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn4Final = new Pose2d(1.16205, 2.189226, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn5Initial = new Pose2d(1.33985, 2.747772, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn5Final = new Pose2d(1.16205, 2.747772, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn6Initial = new Pose2d(1.33985, 3.306318, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn6Final = new Pose2d(1.16205, 3.306318, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn7Initial = new Pose2d(1.33985, 3.865626, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn7Final = new Pose2d(1.16205, 3.865626, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn8Initial = new Pose2d(1.33985, 4.424172, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn8Final = new Pose2d(1.16205, 4.424172, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn9Initial = new Pose2d(1.33985, 4.982718, new Rotation2d(Math.PI));
    private final Pose2d BlueCommunityColumn9Final = new Pose2d(1.16205, 4.982718, new Rotation2d(Math.PI));

    private final Pose2d RedCommunityColumn1Initial = new Pose2d(15.2019, 0.512826, new Rotation2d(0));
    private final Pose2d RedCommunityColumn1Final = new Pose2d(15.3797, 0.512826, new Rotation2d(0));
    private final Pose2d RedCommunityColumn2Initial = new Pose2d(15.2019, 1.071372, new Rotation2d(0));
    private final Pose2d RedCommunityColumn2Final = new Pose2d(15.3797, 1.071372, new Rotation2d(0));
    private final Pose2d RedCommunityColumn3Initial = new Pose2d(15.2019, 1.629918, new Rotation2d(0));
    private final Pose2d RedCommunityColumn3Final = new Pose2d(15.3797, 1.629918, new Rotation2d(0));
    private final Pose2d RedCommunityColumn4Initial = new Pose2d(15.2019, 2.189226, new Rotation2d(0));
    private final Pose2d RedCommunityColumn4Final = new Pose2d(15.3797, 2.189226, new Rotation2d(0));
    private final Pose2d RedCommunityColumn5Initial = new Pose2d(15.2019, 2.747772, new Rotation2d(0));
    private final Pose2d RedCommunityColumn5Final = new Pose2d(15.3797, 2.747772, new Rotation2d(0));
    private final Pose2d RedCommunityColumn6Initial = new Pose2d(15.2019, 3.306318, new Rotation2d(0));
    private final Pose2d RedCommunityColumn6Final = new Pose2d(15.3797, 3.306318, new Rotation2d(0));
    private final Pose2d RedCommunityColumn7Initial = new Pose2d(15.2019, 3.865626, new Rotation2d(0));
    private final Pose2d RedCommunityColumn7Final = new Pose2d(15.3797, 3.865626, new Rotation2d(0));
    private final Pose2d RedCommunityColumn8Initial = new Pose2d(15.2019, 4.424172, new Rotation2d(0));
    private final Pose2d RedCommunityColumn8Final = new Pose2d(15.3797, 4.424172, new Rotation2d(0));
    private final Pose2d RedCommunityColumn9Initial = new Pose2d(15.2019, 4.982718, new Rotation2d(0));
    private final Pose2d RedCommunityColumn9Final = new Pose2d(15.3797, 4.982718, new Rotation2d(0));

    private final Pose2d BlueChargeHubTopComm = new Pose2d(2.148713, 3.3653095, new Rotation2d(0));
    private final Pose2d BlueChargeHubMidComm = new Pose2d(2.148713, 2.747772, new Rotation2d(0));
    private final Pose2d BlueChargeHubBotComm = new Pose2d(2.148713, 2.1302345, new Rotation2d(0));
    private final Pose2d BlueChargeHubTopOpen = new Pose2d(4.855718, 3.3653095, new Rotation2d(Math.PI));
    private final Pose2d BlueChargeHubMidOpen = new Pose2d(4.855718, 2.747772, new Rotation2d(Math.PI));
    private final Pose2d BlueChargeHubBotOpen = new Pose2d(4.855718, 2.1302345, new Rotation2d(Math.PI));
    private final Pose2d BlueChargeHubTop = new Pose2d(3.8354, 3.3653095, new Rotation2d(0));
    private final Pose2d BlueChargeHubMid = new Pose2d(3.8354, 2.747772, new Rotation2d(0));
    private final Pose2d BlueChargeHubBot = new Pose2d(3.8354, 2.1302345, new Rotation2d(0));

    private final Pose2d RedChargeHubTopComm = new Pose2d(14.393037, 3.3653095, new Rotation2d(Math.PI));
    private final Pose2d RedChargeHubMidComm = new Pose2d(14.393037, 2.747772, new Rotation2d(Math.PI));
    private final Pose2d RedChargeHubBotComm = new Pose2d(14.393037, 2.1302345, new Rotation2d(Math.PI));
    private final Pose2d RedChargeHubTopOpen = new Pose2d(11.686032, 3.3653095, new Rotation2d(0));
    private final Pose2d RedChargeHubMidOpen = new Pose2d(11.686032, 2.747772, new Rotation2d(0));
    private final Pose2d RedChargeHubBotOpen = new Pose2d(11.686032, 2.1302345, new Rotation2d(0));
    private final Pose2d RedChargeHubTop = new Pose2d(12.70635, 3.3653095, new Rotation2d(0));
    private final Pose2d RedChargeHubMid = new Pose2d(12.70635, 2.747772, new Rotation2d(0));
    private final Pose2d RedChargeHubBot = new Pose2d(12.70635, 2.1302345, new Rotation2d(0));

    private final Pose2d AprilTag1 = new Pose2d(15.513558, 1.071626, new Rotation2d(0));//Red April Tags
    private final Pose2d AprilTag2 = new Pose2d(15.513558, 2.748026, new Rotation2d(0));
    private final Pose2d AprilTag3 = new Pose2d(15.513558, 4.424426, new Rotation2d(0));
    private final Pose2d AprilTag4 = new Pose2d(16.178784, 6.749796, new Rotation2d(0));

    private final Pose2d AprilTag5 = new Pose2d(0.36195, 6.749796, new Rotation2d(Math.PI));//Blue April Tags
    private final Pose2d AprilTag6 = new Pose2d(1.02743, 4.424426, new Rotation2d(Math.PI));
    private final Pose2d AprilTag7 = new Pose2d(1.02743, 2.748026, new Rotation2d(Math.PI));
    private final Pose2d AprilTag8 = new Pose2d(1.02743, 1.071626, new Rotation2d(Math.PI));

    /**
	 * Gets the initial column position
	 * 
	 * @param column The column that will be returned
	 */
    public Pose2d getInitialColumn(int column) {
        if(true) {
            switch(column){
                case 1:
                    return BlueCommunityColumn1Initial;
                case 2:
                    return BlueCommunityColumn2Initial;
                case 3:
                    return BlueCommunityColumn3Initial;
                case 4:
                    return BlueCommunityColumn4Initial;
                case 5:
                    return BlueCommunityColumn5Initial;
                case 6:
                    return BlueCommunityColumn6Initial;
                case 7:
                    return BlueCommunityColumn7Initial;
                case 8:
                    return BlueCommunityColumn8Initial;
                case 9:
                    return BlueCommunityColumn9Initial;
                default:
                    return null;
            }
        }
        else {
            switch(column){
                case 1:
                    return RedCommunityColumn1Initial;
                case 2:
                    return RedCommunityColumn2Initial;
                case 3:
                    return RedCommunityColumn3Initial;
                case 4:
                    return RedCommunityColumn4Initial;
                case 5:
                    return RedCommunityColumn5Initial;
                case 6:
                    return RedCommunityColumn6Initial;
                case 7:
                    return RedCommunityColumn7Initial;
                case 8:
                    return RedCommunityColumn8Initial;
                case 9:
                    return RedCommunityColumn9Initial;
                default:
                    return null;
            }
        }
    }

    /**
	 * Gets the final column position
	 * 
	 * @param column The column that will be returned
	 */
    public Pose2d getFinalColumn(int column) {
        if(true) {
            switch(column){
                case 1:
                    return BlueCommunityColumn1Final;
                case 2:
                    return BlueCommunityColumn2Final;
                case 3:
                    return BlueCommunityColumn3Final;
                case 4:
                    return BlueCommunityColumn4Final;
                case 5:
                    return BlueCommunityColumn5Final;
                case 6:
                    return BlueCommunityColumn6Final;
                case 7:
                    return BlueCommunityColumn7Final;
                case 8:
                    return BlueCommunityColumn8Final;
                case 9:
                    return BlueCommunityColumn9Final;
                default:
                    return null;
            }
        }
        else {
            switch(column){
                case 1:
                    return RedCommunityColumn1Final;
                case 2:
                    return RedCommunityColumn2Final;
                case 3:
                    return RedCommunityColumn3Final;
                case 4:
                    return RedCommunityColumn4Final;
                case 5:
                    return RedCommunityColumn5Final;
                case 6:
                    return RedCommunityColumn6Final;
                case 7:
                    return RedCommunityColumn7Final;
                case 8:
                    return RedCommunityColumn8Final;
                case 9:
                    return RedCommunityColumn9Final;
                default:
                    return null;
            }
        }
    }

    /**
	 * Gets the position to approach the station from inside the community
	 * 
	 * @param position 1-3 Lowest-Hightest
	 */
    public Pose2d getChargepadCommunity(int position){
        if(true) {
            switch(position){
                case 1:
                    return BlueChargeHubBotComm;
                case 2:
                    return BlueChargeHubMidComm;
                case 3:
                    return BlueChargeHubTopComm;
                default:
                    return null;
            }
        } else {
            switch(position){
                case 1:
                    return RedChargeHubBotComm;
                case 2:
                    return RedChargeHubMidComm;
                case 3:
                    return RedChargeHubTopComm;
                default:
                    return null;
            }
        }
    }

    /**
	 * Gets the position to approach the station from outside the community
	 * 
	 * @param position 1-3 Lowest-Hightest
	 */
    public Pose2d getChargepadField(int position){
        if(true) {
            switch(position){
                case 1:
                    return BlueChargeHubBotOpen;
                case 2:
                    return BlueChargeHubMidOpen;
                case 3:
                    return BlueChargeHubTopOpen;
                default:
                    return null;
            }
        } else {
            switch(position){
                case 1:
                    return RedChargeHubBotOpen;
                case 2:
                    return RedChargeHubMidOpen;
                case 3:
                    return RedChargeHubTopOpen;
                default:
                    return null;
            }
        }
    }

    /**
	 * Gets the center positions on the station
	 * 
	 * @param position 1-3 Lowest-Hightest
	 */
    public Pose2d getChargepadCenter(int position){
        if(true) {
            switch(position){
                case 1:
                    return BlueChargeHubBot;
                case 2:
                    return BlueChargeHubMid;
                case 3:
                    return BlueChargeHubTop;
                default:
                    return null;
            }
        } else {
            switch(position){
                case 1:
                    return RedChargeHubBot;
                case 2:
                    return RedChargeHubMid;
                case 3:
                    return RedChargeHubTop;
                default:
                    return null;
            }
        }
    }

    /**
	 * Gets the position of a specified April Tag
	 * 
	 * @param position 1-3 Lowest-Highest Community | 4 Loading Station
	 */
    public Pose2d getAprilTag(int ID){
        if(true) {
            switch(ID){
                case 1:
                    return AprilTag1;
                case 2:
                    return AprilTag2;
                case 3:
                    return AprilTag3;
                case 4:
                    return AprilTag4;
                default:
                    return null;
            }
        } else {
            switch(ID){
                case 1:
                    return AprilTag5;
                case 2:
                    return AprilTag6;
                case 3:
                    return AprilTag7;
                case 4:
                    return AprilTag8;
                default:
                    return null;
            }
        }
    }

    public Pose2d getClosestGoal(DriveTrain drivetrain){
        return null;
    }
}
