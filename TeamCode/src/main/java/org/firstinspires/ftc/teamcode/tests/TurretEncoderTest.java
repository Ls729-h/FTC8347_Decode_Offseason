package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp(name = "Turret Encoder Test", group = "Tests")
@Configurable
public class TurretEncoderTest extends LinearOpMode {
    public static double MANUAL_POWER = 0.25;

    private DcMotorEx turret;
    private AnalogInput absoluteEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, Shooter.TURRET_ABS_ENCODER);

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                Shooter.TURRET_ABS_ZERO_DEG = rawAbsAngleDeg();
            }
            if (gamepad1.x) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            double power = 0.0;
            if (gamepad1.left_bumper) {
                power = MANUAL_POWER;
            } else if (gamepad1.right_bumper) {
                power = -MANUAL_POWER;
            }
            turret.setPower(power);

            double rawAbs = rawAbsAngleDeg();
            double absDeg = normalizeDegrees(rawAbs - Shooter.TURRET_ABS_ZERO_DEG);
            double motorTicks = turret.getCurrentPosition();
            double motorDeg = motorTicks / Shooter.TURRET_FULL_RANGE_ENCODER * Shooter.TURRET_FULL_RANGE_DEGREE;
            double motorVelTicks = turret.getVelocity();
            double motorVelDeg = motorVelTicks / Shooter.TURRET_FULL_RANGE_ENCODER * Shooter.TURRET_FULL_RANGE_DEGREE;

            telemetry.addData("abs voltage", "%.4f / %.2f", absoluteEncoder.getVoltage(), absMaxVoltage());
            telemetry.addData("abs raw deg", "%.2f", rawAbs);
            telemetry.addData("abs zero deg", "%.2f", Shooter.TURRET_ABS_ZERO_DEG);
            telemetry.addData("abs normalized deg", "%.2f", absDeg);
            telemetry.addData("motor ticks", "%.0f", motorTicks);
            telemetry.addData("motor deg", "%.2f", motorDeg);
            telemetry.addData("motor vel ticks/s", "%.2f", motorVelTicks);
            telemetry.addData("motor vel deg/s", "%.2f", motorVelDeg);
            telemetry.addData("manual power", "%.2f", power);
            telemetry.addData("Y", "zero absolute encoder");
            telemetry.addData("X", "reset motor encoder");
            telemetry.update();
        }
    }

    private double rawAbsAngleDeg() {
        double angle = Range.clip(absoluteEncoder.getVoltage() / absMaxVoltage(), 0.0, 1.0) * 360.0;
        return Shooter.TURRET_ABS_REVERSED ? normalizeDegrees(-angle) : normalizeDegrees(angle);
    }

    private double absMaxVoltage() {
        return Shooter.TURRET_ABS_MAX_VOLTAGE > 0.0
                ? Shooter.TURRET_ABS_MAX_VOLTAGE
                : absoluteEncoder.getMaxVoltage();
    }

    private static double normalizeDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }
}
