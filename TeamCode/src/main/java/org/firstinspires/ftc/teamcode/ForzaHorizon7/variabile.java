package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class variabile {

    public static double default_cuva = .55; //.35 //.25
    public static double drop_cuva = .15; //.6
    public static double absorbtie_power = 1;
    public static double roata_power = -1;
    public static double scula_power = .6;

    public static double timp_ridicare_sus = 2.1;
    public static double timp_ridicare_centru = 1;
    public static double timp_ridicare_jos = .5;

    public static double timp_coborare_sus = .725;
    public static double timp_coborare_centru = .725/2;
    public static double timp_coborare_jos = .725/3;

//    public static TrajectoryVelocityConstraint depozit_velConstrain = .5;

    public static Pose2d hub = new Pose2d( 22, 23, Math.toRadians(0)); //y 25
    public static Vector2d hub_vector = new Vector2d( 22, 25);
    public static Pose2d start = new Pose2d( 0, 0, Math.toRadians(0));
    public static Pose2d start_spreHouse = new Pose2d( -6, 0, Math.toRadians(90));
    public static Pose2d start_spreHouse_realinie = new Pose2d( -8, 0, Math.toRadians(90));
    public static Vector2d start_spreHouse_realinieVector = new Vector2d( -8, 0);
    public static Vector2d depozit = new Vector2d( -7,-40 );
    public static Vector2d depozit_realinie = new Vector2d( -8, -37);
    public static Vector2d depozit_penultim = new Vector2d( -7, -28);
    public static Vector2d depozit_final = new Vector2d( 20, -30);
    public static Vector2d depozit_iesire = new Vector2d( -4, -20);
    public static Pose2d rata = new Pose2d(0,0, Math.toRadians(0));
}
