package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp(name = "Turret Encoder Test", group = "Tests")
@Configurable
public class TurretEncoderTest extends LinearOpMode {
    public static double MANUAL_POWER = 0.25;

    private DcMotorEx turret;
    private DcMotorEx turretEncoder;
    private long lastEncoderTime = 0;
    private double lastEncoderDeg = 0.0;
    private double encoderVelDeg = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        turretEncoder = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_ENCODER);

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        lastEncoderTime = System.nanoTime();
        lastEncoderDeg = rawEncoderAngleDeg();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                Shooter.TURRET_ENCODER_ZERO_DEG = rawEncoderAngleDeg();
            }
            if (gamepad1.x) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad1.b) {
                turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Shooter.TURRET_ENCODER_ZERO_DEG = 0.0;
                lastEncoderTime = System.nanoTime();
                lastEncoderDeg = rawEncoderAngleDeg();
                encoderVelDeg = 0.0;
            }

            double power = 0.0;
            if (gamepad1.left_bumper) {
                power = MANUAL_POWER;
            } else if (gamepad1.right_bumper) {
                power = -MANUAL_POWER;
            }
            turret.setPower(power);

            double rawEncoderDeg = rawEncoderAngleDeg();
            double encoderDeg = rawEncoderDeg - Shooter.TURRET_ENCODER_ZERO_DEG;
            double encoderTicks = turretEncoder.getCurrentPosition();

            long now = System.nanoTime();
            double dtSeconds = (now - lastEncoderTime) / 1e9;
            if (dtSeconds > 0.0001) {
                encoderVelDeg = (rawEncoderDeg - lastEncoderDeg) / dtSeconds;
                lastEncoderDeg = rawEncoderDeg;
                lastEncoderTime = now;
            }

            double motorTicks = turret.getCurrentPosition();
            double motorDeg = motorTicks / Shooter.TURRET_FULL_RANGE_ENCODER * Shooter.TURRET_FULL_RANGE_DEGREE;
            double motorVelTicks = turret.getVelocity();
            double motorVelDeg = motorVelTicks / Shooter.TURRET_FULL_RANGE_ENCODER * Shooter.TURRET_FULL_RANGE_DEGREE;

            telemetry.addData("ELC ticks", "%.0f", encoderTicks);
            telemetry.addData("ELC raw deg", "%.2f", rawEncoderDeg);
            telemetry.addData("ELC zero deg", "%.2f", Shooter.TURRET_ENCODER_ZERO_DEG);
            telemetry.addData("ELC normalized deg", "%.2f", encoderDeg);
            telemetry.addData("ELC vel deg/s", "%.2f", encoderVelDeg);
            telemetry.addData("ELC port", Shooter.TURRET_ENCODER);
            telemetry.addData("motor ticks", "%.0f", motorTicks);
            telemetry.addData("motor deg", "%.2f", motorDeg);
            telemetry.addData("motor vel ticks/s", "%.2f", motorVelTicks);
            telemetry.addData("motor vel deg/s", "%.2f", motorVelDeg);
            telemetry.addData("manual power", "%.2f", power);
            telemetry.addData("Y", "software zero ELC encoder");
            telemetry.addData("X", "reset motor encoder");
            telemetry.addData("B", "reset ELC encoder ticks");
            telemetry.update();
        }
    }

    private double rawEncoderAngleDeg() {
        double angle = Shooter.turretEncoderTicksToTurretDegrees(turretEncoder.getCurrentPosition());
        return Shooter.TURRET_ENCODER_REVERSED ? -angle : angle;
    }

}
