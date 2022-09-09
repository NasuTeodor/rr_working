package org.firstinspires.ftc.teamcode.ForzaHorizon7;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

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

    //ador milisescundele
    //cine citeste asta are nevoie de ajutor

    //   _     _
    //  / \   / \
    // (  *   *  )   -"da beau samniuta"
    //  \-------/
    //   / | | \
    //  / |   | \
    //  _|     |_

//    public static TrajectoryVelocityConstraint depozit_velConstrain = .5;

    public static Pose2d hub = new Pose2d( 22, 25, Math.toRadians(0)); //y 25
    public static Vector2d hub_vector = new Vector2d( 23, 25);
    public static Pose2d hub_rata = new Pose2d( 22+ 11.5, 49, Math.toRadians(-90));
    public static Vector2d hub_rataVector = new Vector2d( 22+9, 23-6);
    public static Pose2d start = new Pose2d( 0, 0, Math.toRadians(0));
    public static Pose2d start_spreHouse = new Pose2d( -6, 5, Math.toRadians(90));
    public static Pose2d start_spreHouse_realinie = new Pose2d( -8, 5, Math.toRadians(90));
    public static Vector2d start_spreHouse_realinieVector = new Vector2d( -8, 0);
    public static Vector2d depozit = new Vector2d( -7,-41 );
    public static Vector2d depozit_realinie = new Vector2d( -8, -37);
    public static Vector2d depozit_penultim = new Vector2d( -7, -28);
    public static Vector2d depozit_final = new Vector2d( 20, -30);
    public static Vector2d depozit_iesire = new Vector2d( -4, -20);
    public static Pose2d rata = new Pose2d(2,57.2+23.9, Math.toRadians(0));
    public static Vector2d rata_vector = new Vector2d( 3, 57.2+23.6);

    public static Vector2d parcare_ratePunctSafe = new Vector2d(35, 57.2+28);
    public static Pose2d parcare_rata = new Pose2d( 12, 57.2+33.6, Math.toRadians(-90));
    public static Vector2d parcare_rataVector = new Vector2d(20, 57.2+20);


    //cum inca nu am murit de foame
    //cine spala canile in care pui acelasi lucru
    //reteta cafea thobor foame + cafea + ingredient special

    //CEALALTA
    public static Pose2d hubOPUS = new Pose2d( 22, -25, Math.toRadians(0)); //y 25
    public static Vector2d hub_vectorOPUS = new Vector2d( 30, -25);
    public static Pose2d hub_rataOPUS = new Pose2d( 22+ 15.5, -48, Math.toRadians(90));
    public static Vector2d hub_rataVectorOPUS = new Vector2d( 22+9, -23-6);
    public static Pose2d startOPUS = new Pose2d( 0, 0, Math.toRadians(0));
    public static Pose2d start_spreHouseOPUS = new Pose2d( -6, -5, Math.toRadians(-90));
    public static Pose2d start_spreHouse_realinieOPUS = new Pose2d( -8, -5, Math.toRadians(-90));
    public static Vector2d start_spreHouse_realinieVectorOPUS = new Vector2d( -8, 0);
    public static Vector2d depozitOPUS = new Vector2d( -7,41 );
    public static Vector2d depozit_realinieOPUS = new Vector2d( -8, 37);
    public static Vector2d depozit_penultimOPUS = new Vector2d( -7, 28);
    public static Vector2d depozit_finalOPUS = new Vector2d( 20, 30);
    public static Vector2d depozit_iesireOPUS = new Vector2d( -4, 20);
    public static Pose2d rataOPUS = new Pose2d(5,-57.2-22, Math.toRadians(60));
    public static Pose2d rata_aproapeOPUS = new Pose2d(5, -57.2-18, Math.toRadians(40));
    public static Vector2d rata_vectorOPUS = new Vector2d( 3, -57.2-23.6);

    public static String viata = "ma cam indoiesc";
    //tot trec astia pe hol si trebuie sa scriu aici

    public static Vector2d parcare_ratePunctSafeOPUS = new Vector2d(35, -57.2-15);
    public static Pose2d parcare_rataOPUS = new Pose2d( 12, -57.2-33.6, Math.toRadians(90));
    public static Vector2d parcare_rataVectorOPUS = new Vector2d(22.5, -57.2-18);
}
