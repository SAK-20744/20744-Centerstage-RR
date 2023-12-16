package org.firstinspires.ftc.teamcode.common.commandbase;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

public class driveFollowTrajectory extends SequentialCommandGroup {
    public driveFollowTrajectory(SampleMecanumDrive dt, Trajectory myTraj) {
        super(
                new InstantCommand( () ->  dt.followTrajectory(myTraj))
        );
    }
}