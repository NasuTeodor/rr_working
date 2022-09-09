package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class AutoTestOPUS extends LinearOpMode {

    SampleMecanumDrive drive;

    private ElapsedTime time = new ElapsedTime();

    public static double FRONTAL_MULTIPLIER = 0.951254565; // DIPSHIT
    public static double LATERAL_MULTIPLIER = 1;

    @Override
    public void runOpMode(){

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose = new Pose2d(0,0,0);

        OpenCvCamera camera; // = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        NordStream2PipeLine pipeline = new NordStream2PipeLine();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        //wrong foloseste while pipeline.gasesteMarker
        int pozitie = pipeline.gasesteMarker();
        double timp_ridicare;
        if(pozitie == 1)
            timp_ridicare = variabile.timp_ridicare_jos;
        else if(pozitie == 2)
            timp_ridicare = variabile.timp_ridicare_centru;
        else timp_ridicare = variabile.timp_ridicare_sus;

        telemetry.addData("pozitie:", String.valueOf(pozitie));
        telemetry.update();

        waitForStart();

        drive.setPoseEstimate(pose);

//        while(pipeline.gasesteMarker() == 0){}

        TrajectorySequence start2hub = drive.trajectorySequenceBuilder( pose )
                .addDisplacementMarker( () ->{
                    drive.scula_power(1);
                })
                .lineTo( variabile.hub_vectorOPUS )
                .addTemporalMarker( timp_ridicare-1, ()->{
                    drive.scula_power(0);
                    drive.arunca();
                })
                .addTemporalMarker( timp_ridicare, ()->{
                    drive.retrage_cuva();
                    drive.scula_power(-1);
                })
                .addTemporalMarker( timp_ridicare + variabile.timp_coborare_sus, ()->{
                    drive.scula_power(0);
                })
                .lineToLinearHeading( variabile.start_spreHouseOPUS)
                .addDisplacementMarker( ()->{
                    drive.absoarbe();
                })
                .lineTo(variabile.depozit_penultimOPUS,
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectorySequence(start2hub);

        time.reset();
        while(time.seconds() <= 2 && drive.sensor_distanta()>=9){
            drive.putere(-.1, -.1, -.1, -.1);
        }

        TrajectorySequence depozit2start = drive.trajectorySequenceBuilder( drive.getPoseEstimate() )
                .addDisplacementMarker( ()->{
                    drive.scuipa();
                })
                .splineToConstantHeading(variabile.depozit_penultimOPUS, Math.toRadians(-90))
                .splineToConstantHeading(variabile.start_spreHouse_realinieVectorOPUS, Math.toRadians(-90))
                .addTemporalMarker( 1.7, ()->{
                    drive.stai_absorbtie();
                })
                .build();

        drive.followTrajectorySequence(depozit2start);

        TrajectorySequence start2hub2 = drive.trajectorySequenceBuilder( drive.getPoseEstimate())
                .addDisplacementMarker( () ->{
                    drive.scula_power(1);
                })
                .lineToLinearHeading( variabile.hubOPUS)
                .addTemporalMarker( timp_ridicare-1, ()->{
                    drive.scula_power(0);
                    drive.arunca();
                })
                .addTemporalMarker( timp_ridicare, ()->{
                    drive.retrage_cuva();
                    drive.scula_power(-1);
                })
                .addTemporalMarker( timp_ridicare + variabile.timp_coborare_sus, ()->{
                    drive.scula_power(0);
                })
                .lineToLinearHeading( variabile.start_spreHouseOPUS)
                .addDisplacementMarker( ()->{
                    drive.absoarbe();
                })
                .lineTo(variabile.depozitOPUS)
                .build();

        drive.followTrajectorySequence(start2hub2);

        time.reset();
        while(time.seconds() <= 2 && drive.sensor_distanta()>=9){
            drive.putere(-.1, -.1, -.1, -.1);
        }

        TrajectorySequence depozit2start2 = drive.trajectorySequenceBuilder( drive.getPoseEstimate() )
                .addDisplacementMarker( ()->{
                    drive.scuipa();
                })
                .lineTo(variabile.depozitOPUS)
//                .strafeTo(variabile.depozit_penultim)
                .splineToLinearHeading(variabile.start_spreHouse_realinieOPUS, Math.toRadians(-90))
                .addTemporalMarker( 1.7, ()->{
                    drive.stai_absorbtie();
                })
                .build();

        drive.followTrajectorySequence(depozit2start2);

        TrajectorySequence start2hub3 = drive.trajectorySequenceBuilder( drive.getPoseEstimate() )
                .addDisplacementMarker( () ->{
                    drive.scula_power(1);
                })
                .lineToLinearHeading( variabile.hubOPUS )
                .addTemporalMarker( timp_ridicare-1, ()->{
                    drive.scula_power(0);
                    drive.arunca();
                })
                .addTemporalMarker( timp_ridicare, ()->{
                    drive.retrage_cuva();
                    drive.scula_power(-1);
                })
                .addTemporalMarker( timp_ridicare + variabile.timp_coborare_sus, ()->{
                    drive.scula_power(0);
                })
                .lineToLinearHeading( variabile.start_spreHouseOPUS)
                .lineTo(variabile.depozit_penultimOPUS)
                .strafeTo( variabile.depozit_finalOPUS)
                .build();

        drive.followTrajectorySequence(start2hub3);

    }

    //functie care returneaza distanta corecta cu tot cu multiplier ca face undershoot
    //si trebuie facuta si pentru frontal si lateral separat // have fun at DIPSHIT

    double repair_frontal (double distance){ return distance*FRONTAL_MULTIPLIER; }

    public double repair_lateral (double distance){ return distance*LATERAL_MULTIPLIER; }

}
