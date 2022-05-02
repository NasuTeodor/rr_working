package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
@Disabled
public class Driftus extends LinearOpMode {

    DcMotor fl,fr,rl,rr;

    double SPEED = 0.5;
    double SPEED_REDUCTION = 0.5;
    double REAL_SPEED;
    Boolean driftMode = false;

    @Override
    public void runOpMode(){

        double drive,turn;

        initMotoare();

        telemetry.addData(">Waiting", " for start");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a)
                SPEED = 0.7;
            if(gamepad1.b)
                SPEED = 0.4;

            if(gamepad1.y)
                driftMode = !driftMode;

            if(!driftMode)
                REAL_SPEED = SPEED * SPEED_REDUCTION;
            else
                REAL_SPEED = SPEED;

            while(gamepad1.dpad_up)
                putere(REAL_SPEED, REAL_SPEED, REAL_SPEED, REAL_SPEED);
            while(gamepad1.dpad_down)
                putere(-REAL_SPEED, -REAL_SPEED, -REAL_SPEED, -REAL_SPEED);
            while(gamepad1.dpad_right)
                putere(REAL_SPEED, -REAL_SPEED, -REAL_SPEED, REAL_SPEED);
            while(gamepad1.dpad_left)
                putere(-REAL_SPEED, REAL_SPEED, REAL_SPEED, -REAL_SPEED);
            stopMotor();

            while(gamepad1.left_bumper)
                putere(-REAL_SPEED, REAL_SPEED, -REAL_SPEED, REAL_SPEED);
            while(gamepad1.right_bumper)
                putere(REAL_SPEED, -REAL_SPEED, REAL_SPEED, -REAL_SPEED);
            stopMotor();

            if(!driftMode) {
                drive = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_y;

                while (drive != 0 || turn != 0) {

                    drive = gamepad1.right_stick_x;
                    turn = gamepad1.left_stick_y;

                    double leftPower = Range.clip( drive + turn, -1, 1);
                    double rightPower = Range.clip( drive - turn, -1, 1);

                    leftPower *= SPEED_REDUCTION;
                    rightPower *= SPEED_REDUCTION;

                    putere(leftPower, rightPower, leftPower, rightPower);

                }
                stopMotor();
            } else {

                //add drifting
                double front = gamepad1.right_trigger;
                double back = gamepad1.left_trigger;
                turn = gamepad1.left_stick_x;

                if( back == 0 && front != 0 )
                    while(back == 0 && front != 0){

                        front = gamepad1.right_trigger;
                        back = gamepad1.left_trigger;
                        turn = gamepad1.left_stick_x;

                        double leftTurn = 0, rightTurn = 0;
                        if(turn > 0 )
                            rightTurn = -turn;
                        else
                            leftTurn = turn;

                        if(turn > 2)
                            leftTurn = -rightTurn;
                        else if( turn < 2)
                            rightTurn = -leftTurn;

                        putere(rightTurn, leftTurn, front, front);

                    }
                else if( back != 0 && front != 0 )
                    while( back != 0 && front != 0 ){

                        front = gamepad1.right_trigger;
                        back = -gamepad1.left_trigger;

                        putere(front, front, back, back);

                }
                stopMotor();

            }


        }

    }

    public void initMotoare(){

        fl = hardwareMap.get(DcMotor.class, "lf");
        fr = hardwareMap.get(DcMotor.class, "rf");
        rl = hardwareMap.get(DcMotor.class, "lr");
        rr = hardwareMap.get(DcMotor.class, "rr");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        rl.setDirection(DcMotor.Direction.REVERSE);

    }

    public void putere(double flp, double frp, double rlp, double rrp){
        fl.setPower(flp);
        fr.setPower(frp);
        rl.setPower(rlp);
        rr.setPower(rrp);
    }

    public void stopMotor(){ putere(0,0,0,0); }

}
