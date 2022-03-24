package org.firstinspires.ftc.teamcode.HackGoogle;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.full_power;
import static org.firstinspires.ftc.teamcode.drive.ThoborVARS.putere_dpad;
@Disabled
@TeleOp(name = "TIGAIE BMW", group = "idk")

public class Teleop_BMW extends LinearOpMode {
    public DcMotor lf, rf, lr, rr;
    public CRServo brat, gheara;
    double v1, v2, v3, v4;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);


        brat = hardwareMap.crservo.get("brat");
        gheara = hardwareMap.crservo.get("gheara");
        AsteaptaStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            while (gamepad1.dpad_down) {
                v1 = (-putere_dpad);
                v2 = (-putere_dpad);
                v3 = (-putere_dpad);
                v4 = (-putere_dpad);
//                      drive.setMotorPowers(v1 * full_power, v3 * full_power, v4 * full_power, v2 * full_power);
                setMotorPowersFull(v1, v3, v4, v2);
            }
            while (gamepad1.dpad_right) {
                v1 = (+putere_dpad);
                v2 = (-putere_dpad);
                v3 = (-putere_dpad);
                v4 = (putere_dpad);
                setMotorPowersFull(v1, v3, v4, v2);
            }
            while (gamepad1.dpad_up) {
                v1 = (putere_dpad);
                v2 = (putere_dpad);
                v3 = (putere_dpad);
                v4 = (putere_dpad);
                setMotorPowersFull(v1, v3, v4, v2);
            }
            while (gamepad1.dpad_left) {
                v1 = (-putere_dpad);
                v2 = (putere_dpad);
                v3 = (putere_dpad);
                v4 = (-putere_dpad);
                setMotorPowersFull(v1, v3, v4, v2);
            }
            setMotorPowersFull(0,0,0,0);

            while (gamepad2.dpad_up) {
                brat.setPower(1);
            }
            while (gamepad2.dpad_down) {
                brat.setPower(-1);
            }
            brat.setPower(0);
            while (gamepad2.dpad_left ) {
                gheara.setPower(1);
            }
            while (gamepad2.dpad_right ) {
                gheara.setPower(-1);
            }
            gheara.setPower(0);

        }
    }

    private void setMotorPowersFull(double v, double v1, double v2, double v3) {
        lf.setPower(v);
        lr.setPower(v1);
        rr.setPower(v2);
        rf.setPower(v3);
    }

    private void AsteaptaStart() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();
        }
    }
}
