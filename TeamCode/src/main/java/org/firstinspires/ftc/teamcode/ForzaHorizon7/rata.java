package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class rata extends LinearOpMode {

    @Override
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d pose = new Pose2d(-3, 57.2, 0);
        ElapsedTime time = new ElapsedTime();

        double timp_ridicare = variabile.timp_ridicare_sus;

        drive.setPoseEstimate(pose);

        waitForStart();

        TrajectorySequence start2rata = drive.trajectorySequenceBuilder( pose )
                .lineTo( variabile.rata_vector )
                .addDisplacementMarker( ()->{
                    drive.betie_rata(-1);
                })
                .build();
        drive.followTrajectorySequence(start2rata);

        time.reset();
        while(time.seconds()<1.2){}

        TrajectorySequence rata2hub = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker( ()->{
                    drive.stop_rata();
                })
                .addDisplacementMarker( ()->{
                    drive.scula_power(variabile.scula_power);
                })
                .lineTo(variabile.parcare_rataVector)
                .splineToSplineHeading(variabile.hub_rata, Math.toRadians(-90))
                .addTemporalMarker( timp_ridicare-.5, ()->{
                    drive.arunca();
                })
                .addTemporalMarker(timp_ridicare, ()->{
                    drive.scula_power(0-variabile.scula_power);
                    drive.retrage_cuva();
                })
                .addTemporalMarker( timp_ridicare+variabile.timp_coborare_sus, ()->{
                    drive.scula_power(0);
                })
                .lineTo(variabile.parcare_ratePunctSafe)
                .lineToConstantHeading(variabile.parcare_rataVector)
                .build();

        drive.followTrajectorySequence(rata2hub);

    }

}
