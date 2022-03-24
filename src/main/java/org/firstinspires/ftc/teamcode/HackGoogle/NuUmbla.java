package org.firstinspires.ftc.teamcode.HackGoogle;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;

@Config
public class NuUmbla {
    public static long secunde = 1300;

    public static Pose2d startposRosuStanga = new Pose2d(-64.00, -45.00, Math.toRadians(5));
    public static Pose2d startposRosuDreapta = new Pose2d(-64.00, -60.00, Math.toRadians(30));
    public static Pose2d startposRosuDreaptaCelalalt = new Pose2d(-64.00, -50.00, Math.toRadians(-5));
    public static Pose2d startposRosuStangaCelalalt = new Pose2d(-64.00, -35.00, Math.toRadians(-30));

    public static Pose2d startposAlbastruDreapta = new Pose2d(-64.00, 30.00, Math.toRadians(-5));
    public static Pose2d startposAlbastruDreaptaCelalalt = new Pose2d(-64.00, 20.00, Math.toRadians(30));
    public static Pose2d startposAlbastruStanga = new Pose2d(-64.00, 35.00, Math.toRadians(5));
    public static Pose2d startposAlbastruStangaCelalalt = new Pose2d(-64.00, 55.00, Math.toRadians(-30));

//    public static Pose2d ARUNCARE = new Pose2d(-12, -16, Math.toRadians(0));
    public static Pose2d ARUNCARE_NORMALA = new Pose2d(-8, -39, Math.toRadians(5));
//    public static Pose2d PARK = new Pose2d(10, -10);
    public static Pose2d PARK_NORMAL = new Pose2d(10, -45, Math.toRadians(0.0));

    // PENTRU A
    public static Pose2d LA_A = new Pose2d(-10, -63.00, Math.toRadians(180));
    public static Pose2d LA_A_Alb_ST = new Pose2d(10, 23.00, Math.toRadians(-90));

    //    PENTRU B
    public static Pose2d LA_B = new Pose2d(15.00, -55.00, Math.toRadians(210.0));
    public static Pose2d LA_B_Alb_ST = new Pose2d(12, 10.00, Math.toRadians(180));
    public static Pose2d LA_B_Alb_ST_DR = new Pose2d(12, 35.00, Math.toRadians(180));


    // PENTRU C
    public static Pose2d LA_C = new Pose2d(37.00, -67.00, Math.toRadians(180.0));
    public static Pose2d LA_C_Alb_ST = new Pose2d(50, 10.00, Math.toRadians(-90));
    public static Pose2d LA_C_Alb_ST_DR = new Pose2d(50, 40.00, Math.toRadians(-90));

    /*
               ######                ######
             ###     ####        ####     ###
            ##          ###    ###          ##
            ##             ####             ##
            ##             ####             ##
            ##           ##    ##           ##
            ##         ###      ###         ##
             ##  ########################  ##
          ######    ###            ###    ######
      ###     ##    ##              ##    ##     ###
   ###         ## ###      ####      ### ##         ###
  ##           ####      ########      ####           ##
 ##             ###     ##########     ###             ##
  ##           ####      ########      ####           ##
   ###         ## ###      ####      ### ##         ###
      ###     ##    ##              ##    ##     ###
          ######    ###            ###    ######
             ##  ########################  ##
            ##         ###      ###         ##
            ##           ##    ##           ##
            ##             ####             ##
            ##             ####             ##
            ##          ###    ###          ##
             ###     ####        ####     ###
               ######                ######
     
     */
}
