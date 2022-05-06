package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class cuviosSensor extends LinearOpMode {

    public DistanceSensor distanceSensor;
    
    @Override
    public void runOpMode(){

        waitForStart();

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");
        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("aici", String.valueOf(distance));
            telemetry.update();
        }
    }

}
