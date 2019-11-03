package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;

public class Runner_Mecanum_Drive extends MecanumDrive {
    Mecanum_Drive drive;
    Two_Wheel_Localizer localizer;
    TrajectoryFollower follower;
    DriveConstraints constraints;

    public Runner_Mecanum_Drive(HardwareMap h){
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH);
        setDrive(h);
        setLocalizer(h);
        constraints = new MecanumConstraints(DriveConstants.constraints, DriveConstants.TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(DriveConstants.TRANSLATIONAL_PID_X, DriveConstants.TRANSLATIONAL_PID_Y, DriveConstants.HEADING_PID);
    }

    public void setDrive(HardwareMap h){
        drive = new Mecanum_Drive(h);
    }

    public void setLocalizer(HardwareMap h){
        localizer = new Two_Wheel_Localizer(new Dead_Wheel(new MA3_Encoder(0, 0)), new Dead_Wheel(new MA3_Encoder(2, 0)), h.get(ExpansionHubEx.class, "Expansion Hub 2"));
    }

    @Override
    public void setMotorPowers(double p1, double p2, double p3, double p4){
        drive.getMotors().get(0).setPower(p1);
        drive.getMotors().get(1).setPower(p2);
        drive.getMotors().get(2).setPower(p3);
        drive.getMotors().get(3).setPower(p4);
    }

    @Override
    public ArrayList<Double> getWheelPositions(){
        ArrayList<Double> pos = new ArrayList<Double>();
        pos.add((double)drive.getMotors().get(0).getCurrentPosition());
        pos.add((double)drive.getMotors().get(1).getCurrentPosition());
        pos.add((double)drive.getMotors().get(2).getCurrentPosition());
        pos.add((double)drive.getMotors().get(3).getCurrentPosition());
        return pos;
    }

    public double getRawExternalHeading(){
        return localizer.getPrev_heading();
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getEstimatedPose(), constraints);
    }

    public void followTrajectory(Trajectory t){
        follower.followTrajectory(t);
    }

    public void update(RevBulkData data, long dt){
        localizer.update(data, dt);
        setDriveSignal(follower.update(getEstimatedPose()));
        drive.write();
    }

    public Pose2d getEstimatedPose(){
        return localizer.getPos();
    }
}
