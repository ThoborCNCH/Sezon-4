package org.firstinspires.ftc.teamcode.HackGoogle;

import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.DirLF;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.DirLR;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.DirRF;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.DirRR;
//import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.pozitie_gheara_closed;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.pozitie_inceput_deschis;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.pozitie_inceput_inchis;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putereAbsorbtie;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putereAruncare;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.puterePowerShot;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putereWooble;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putere_dpad;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putere_rotire;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.tragaciDeschis;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.tragaciInchis;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp Bun",group = "nationala")

public class teleOp extends LinearOpMode {
    private boolean dumnezeu = true;
    public Servo stanga;
    public Servo dreapta;
//    public double tragaciPus = 0;
    public double organizatorPus = 0;
    public double fatetePuse = 0;
    public double ghearaPusa = 0;
    double buttonReleased = 1;
    double buttonReleased2 = 1;
    double buttonReleased3 = 1;
    double buttonReleased5 = 1;
    double epureFata = 1;
    public static DcMotor leftFront;
    public static DcMotor leftRear;
    public static DcMotor rightRear;
    public static DcMotor rightFront;
    private Servo inceput;

    public DcMotor absorbtie;
    public DcMotor aruncare;
    private Servo tragaci;
    public CRServo wooblemoto;



    double v1, v2, v3, v4;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        leftRear = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        rightRear = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        rightFront = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        inceput = hardwareMap.get(Servo.class, "inceput");

        absorbtie = hardwareMap.get(DcMotorEx.class, "absorbtie");
        aruncare = hardwareMap.get(DcMotorEx.class, "aruncare");

        aruncare.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aruncare.setDirection(DcMotorEx.Direction.REVERSE);
        aruncare.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        tragaci = hardwareMap.get(Servo.class, "tragaci");


        stanga = hardwareMap.get(Servo.class, "stanga");
        dreapta = hardwareMap.get(Servo.class, "dreapta");

        wooblemoto = hardwareMap.get(CRServo.class, "wooblemoto");


        if (DirLR == 1) leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        else leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        if (DirRR == 1) rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        else rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        if (DirLF == 1) leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        else leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        if (DirRF == 1) rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        else rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AsteaptaStart();
        PozitiiDeBaza();

        while (opModeIsActive() && !isStopRequested()) {

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = (gamepad1.right_stick_x);
            v1 = r * Math.cos(robotAngle) + rightX;
            v2 = r * Math.sin(robotAngle) - rightX;
            v3 = r * Math.sin(robotAngle) + rightX;
            v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);

            while (gamepad1.dpad_down) {
                v1 = (-putere_dpad);
                v2 = (-putere_dpad);
                v3 = (-putere_dpad);
                v4 = (-putere_dpad);
//                      this.setMotorPowers(v1 * full_power, v3 * full_power, v4 * full_power, v2 * full_power);
                setMotorPowersFull(v1, v3, v4, v2);
            }
            while (gamepad1.dpad_right) {
                v1 = (+putere_dpad);
                v2 = (-putere_dpad);
                v3 = (-putere_dpad);
                v4 = (putere_dpad);
                this.setMotorPowersFull(v1, v3, v4, v2);
            }
            while (gamepad1.dpad_up) {
                v1 = (putere_dpad);
                v2 = (putere_dpad);
                v3 = (putere_dpad);
                v4 = (putere_dpad);
                this.setMotorPowersFull(v1, v3, v4, v2);
            }
            while (gamepad1.dpad_left) {
                v1 = (-putere_dpad);
                v2 = (putere_dpad);
                v3 = (putere_dpad);
                v4 = (-putere_dpad);
                this.setMotorPowersFull(v1, v3, v4, v2);
            }

            while (gamepad1.left_trigger != 0) {
                this.setMotorPowersFull(-putere_rotire, -putere_rotire, putere_rotire, putere_rotire);
            }

            while (gamepad1.right_trigger != 0) {
                this.setMotorPowersFull(putere_rotire, putere_rotire, -putere_rotire, -putere_rotire);
            }

            if (gamepad1.left_bumper) {
                this.Absoarbe(-putereAbsorbtie);

            } else if (gamepad2.a) {
                this.Absoarbe(putereAbsorbtie);
            } else {
                this.Absoarbe(0);
            }
            if (gamepad1.right_bumper) {
                this.Arunca(putereAruncare);
            } else if (gamepad1.y) {
                this.Arunca(puterePowerShot);
            } else {
                this.Arunca(0);
            }

            if (gamepad2.dpad_up) {
                this.RidicaWooble(putereWooble);

            } else if (gamepad2.dpad_down) {
                this.RidicaWooble(-putereWooble);

            } else {
                this.RidicaWooble(0);

            }
            //----------[tragaci]---------------------------------------------------------------------
            if (gamepad2.dpad_right && buttonReleased5 == 1) {
                buttonReleased5 = 0;
                if (ghearaPusa == 0) {
                    ghearaPusa = 1;
                    this.Gheara();
//                            drive.WoobleGheara(pozitie_gheara_closed);
                    telemetry.addLine("Gheara s-a strans");
                } else {
                    ghearaPusa = 0;
                    this.Gheara();
//                            drive.WoobleGheara(pozitie_gheara_open);
                    telemetry.addLine("Gheara s-a deschis");
                }
            }
            if (!gamepad2.dpad_right) buttonReleased5 = 1;


            //----------[tragaci]---------------------------------------------------------------------
            if (gamepad2.right_bumper && buttonReleased == 1) {
//                        buttonReleased = 0;
//                        if (tragaciPus == 0) {
//                            tragaciPus = 1;
                this.TragaciPozitie(tragaciDeschis);
                sleep(400);
                this.TragaciPozitie(tragaciInchis);
                telemetry.addLine("Tragaci s-a strans");
            }
            if (!gamepad2.right_bumper) buttonReleased = 1;

            if (gamepad2.left_bumper && buttonReleased2 == 1) {
                buttonReleased2 = 0;
                if (organizatorPus == 0) {
                    organizatorPus = 1;
                    this.InceputPozitie(pozitie_inceput_deschis);
                } else {
                    organizatorPus = 0;
                    this.InceputPozitie(pozitie_inceput_inchis);
                }
            }
            if (!gamepad2.left_bumper) buttonReleased2 = 1;

            //----------[fatete]---------------------------------------------------------------------
            if (gamepad1.a && buttonReleased3 == 1) {
                buttonReleased3 = 0;
                if (fatetePuse == 0) {
                    fatetePuse = 1;
//                            drive.fatete(putere_fata_dr, putere_fata_st);
                    telemetry.addLine("Fatete s-au deschis");
                } else {
                    fatetePuse = 0;
//                            drive.fatete(0, 0);
                    telemetry.addLine("Fatete s-au inchis");
                }
            }
            if (!gamepad1.a) buttonReleased3 = 1;

            if (gamepad2.b && epureFata == 1) {
                epureFata = 0;
                if (fatetePuse == 0) {
                    fatetePuse = 1;
//                            drive.fatete(-putere_fata_dr, -putere_fata_st);
                    telemetry.addLine("Fatete s-au deschis");
                } else {
                    fatetePuse = 0;
//                            drive.fatete(0, 0);
                    telemetry.addLine("Fatete s-au inchis");
                }
            }
            if (!gamepad2.b) epureFata = 1;


        }


    }


    public void Gheara(boolean ok) {
        if (dumnezeu && !ok) {
            stanga.setPosition(0);
            dreapta.setPosition(0);
            dumnezeu = false;
        } else {
            stanga.setPosition(1);
            dreapta.setPosition(1);
            dumnezeu = true;
        }
    }

    public void PozitiiDeBaza() {
        TragaciPozitie(tragaciInchis);
        InceputPozitie(pozitie_inceput_deschis);
    }


    public void Gheara() {
        Gheara(false);
    }

    public void Absoarbe(double putereabsorbtie) {
        absorbtie.setPower(putereabsorbtie);
    }

    public void Arunca(double puterearuncare) {
        aruncare.setPower(puterearuncare);
    }


    public void setMotorPowersFull(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    public void RidicaWooble(double putereridicare) {
        wooblemoto.setPower(putereridicare);
    }

    public void InceputPozitie(double pozitieinceput) {
        inceput.setPosition(pozitieinceput);
    }

    public void TragaciPozitie(double poztragaci) {
        tragaci.setPosition(poztragaci);
    }

    public void AsteaptaStart() // pentru eroarea cu Motorola
    {
        waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }


}
