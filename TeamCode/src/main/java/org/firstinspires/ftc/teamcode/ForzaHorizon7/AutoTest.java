package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class AutoTest extends LinearOpMode {

    SampleMecanumDrive drive;

    public static double FRONTAL_MULTIPLIER = 1; // DIPSHIT
    public static double LATERAL_MULTIPLIER = 1;

    @Override
    public void runOpMode(){

        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory drum_fata = drive.trajectoryBuilder(new Pose2d())
                .forward(67)
                .build();
        Trajectory drum_dreapta = drive.trajectoryBuilder(drum_fata.end())
                .strafeRight(24)
                .build();

        waitForStart();

        drive.followTrajectory(drum_fata);
        drive.followTrajectory(drum_dreapta);

    }

    //functie care returneaza distanta corecta cu tot cu multiplier ca face undershoot
    //si ampulea trebuie facuta si pentru frontal si lateral separat // have fun at strafes DIPSHIT

    double repair_frontal (double distance){ return distance*FRONTAL_MULTIPLIER; }

    double repair_lateral (double distance){ return distance*LATERAL_MULTIPLIER; }

}
