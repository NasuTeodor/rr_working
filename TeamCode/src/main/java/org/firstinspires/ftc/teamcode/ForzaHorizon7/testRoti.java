package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class testRoti extends LinearOpMode {

    DcMotor lf,lr,rf,rr;

    public void runOpMode(){

        waitForStart();

        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        while(opModeIsActive()){

            if(gamepad1.dpad_up)
                lf.setPower(1);
            else
                lf.setPower(0);
            if(gamepad1.dpad_right)
                rf.setPower(1);
            else
                rf.setPower(0);
            if(gamepad1.dpad_left)
                lr.setPower(1);
            else
                lr.setPower(0);
            if(gamepad1.dpad_down)
                rr.setPower(1);
            else
                rr.setPower(0);

        }

    }

}
