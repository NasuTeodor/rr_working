package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class rataOPUS extends LinearOpMode {

    @Override
    public void runOpMode(){

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d pose = new Pose2d(-3, -57.2, 0);
        ElapsedTime time = new ElapsedTime();

        double timp_ridicare = variabile.timp_ridicare_sus;

        drive.setPoseEstimate(pose);

        waitForStart();

        TrajectorySequence start2rata = drive.trajectorySequenceBuilder( pose )
                .splineToLinearHeading( variabile.rata_aproapeOPUS, Math.toRadians(40), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .splineToLinearHeading( variabile.rataOPUS, Math.toRadians(-70), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectorySequence(start2rata);

        drive.betie_rata(-1);
        time.reset();
        while(time.seconds()<2.8){}

        TrajectorySequence rata2hub = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker( ()->{
                    drive.stop_rata();
                })
                .addDisplacementMarker( ()->{
                    drive.scula_power(variabile.scula_power);
                })
                .lineTo(variabile.parcare_rataVectorOPUS)
                .splineToSplineHeading(variabile.hub_rataOPUS, Math.toRadians(90))
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
                .lineTo(variabile.parcare_ratePunctSafeOPUS)
                .lineToConstantHeading(variabile.parcare_rataVectorOPUS)
                .build();

        drive.followTrajectorySequence(rata2hub);

    }

}
