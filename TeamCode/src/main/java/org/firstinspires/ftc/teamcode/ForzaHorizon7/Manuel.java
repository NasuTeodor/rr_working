package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class Manuel extends LinearOpMode {

    DcMotor lf,lr,rf,rr;
    CRServo roata,mana,absorbtie;
    Servo cuva;
    DcMotor bascula,scula_m;

    public static double default_cuva = .35; //.25
    public static double drop_cuva = .6;
    public static double absorbtie_power = 1;
    public static double roata_power = -1;
    public static double scula_power = .6;


    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //init_motoare();
        init_aux();
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            while(gamepad1.right_bumper)
                bascula.setPower(.8);
            while(gamepad1.left_bumper)
                bascula.setPower(-.8);
            bascula.setPower(0);

            if (gamepad1.a)
                cuva.setPosition(default_cuva);
            if (gamepad1.b)
                cuva.setPosition(drop_cuva);

            if(gamepad1.x)
                absorbtie.setPower(absorbtie_power);
            else if(gamepad1.y)
                absorbtie.setPower(-absorbtie_power);
            else absorbtie.setPower(0);

            while(gamepad1.dpad_left)
                roata.setPower(roata_power);
            while(gamepad1.dpad_right)
                roata.setPower(-roata_power);
            roata.setPower(0);

            while(gamepad1.dpad_up)
                scula_m.setPower(scula_power);
            while(gamepad1.dpad_down)
                scula_m.setPower(-scula_power);
            scula_m.setPower(0);

            if(gamepad1.left_trigger!=0)
                mana.setPower(1);
            else if(gamepad1.right_trigger!=0)
                mana.setPower(-1);
            else mana.setPower(0);

//            while(gamepad1.right_trigger!=0){
//                while(gamepad1.right_trigger!=0)
//                mana.setPower(1);
//                mana.setPower(0);
//            }
//            while(gamepad1.left_trigger!=0){
//                //while(gamepad1.left_trigger!=0)
//                    mana.setPower(-1);
//            //mana.setPower(0);
//            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x,
                            -gamepad2.right_stick_x
                    )
            );

            drive.update();

            while (gamepad2.left_trigger!=0)
                drive.rotate(gamepad2.left_trigger, -1);
            while (gamepad2.right_trigger!=0)
                drive.rotate(gamepad2.right_trigger, 1);
            //drive.stop_motor();

        }

    }

    void init_motoare(){

        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    void init_aux(){

        absorbtie = hardwareMap.get(CRServo.class, "absortie");
        roata = hardwareMap.get(CRServo.class, "rata");
        mana = hardwareMap.get(CRServo.class, "gheara");
        cuva = hardwareMap.get(Servo.class, "cuva");
        scula_m = hardwareMap.get(DcMotor.class, "brat_marker");
        bascula = hardwareMap.get(DcMotor.class, "brat");

        scula_m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bascula.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bascula.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}
