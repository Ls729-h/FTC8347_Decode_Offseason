package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.subsystem.Shooter.targetVelocity;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

@TeleOp(name = "A_2 Auto Aim Test", group = "Tests")
@Configurable
public class A_2 extends LinearOpMode {
    public static double TARGET_X = 136.5;
    public static double TARGET_Y = 8.0;
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_RAD = Math.PI / 2.0;
    public static double DRIVE_POWER = 1.0;

    public static double manualHoodPos = 0.5;
    public static double manualShooterVelocity = 2300.0;

    private final Robot robot = new Robot();
    private boolean shooterOn = false;
    private boolean autoTurret = true;
    private boolean autoShooter = true;
    private double turretCorrection = 0.0;
    private double distanceCorrection = 0.0;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.RADIANS, START_HEADING_RAD));
        robot.shooter.setPPHeading(0.0);

        waitForStart();

        while (opModeIsActive()) {
            robot.drivetrain.driveFieldOriented(gamepad1, false, 0.0);

            Pose2D current = robot.drivetrain.getPosition();
            double robotX = current.getX(DistanceUnit.INCH);
            double robotY = current.getY(DistanceUnit.INCH);
            double robotHeading = current.getHeading(AngleUnit.DEGREES);
            double targetAngle = Math.toDegrees(Math.atan2(TARGET_Y - robotY, TARGET_X - robotX));
            double turretTarget = normalizeDegrees(targetAngle - robotHeading);
            double distance = Math.hypot(TARGET_Y - robotY, TARGET_X - robotX);
            boolean intakeRequested = gamepad1.right_trigger > 0.5;

            if (gamepad1.left_bumper && !lastLeftBumper) {
                shooterOn = !shooterOn;
            }
            lastLeftBumper = gamepad1.left_bumper;
            if (gamepad1.right_bumper) {
                shooterOn = false;
            }

            if (gamepad2.left_bumper) autoTurret = true;
            if (gamepad2.right_bumper) autoTurret = false;
            if (gamepad2.y) autoShooter = true;
            if (gamepad2.a) autoShooter = false;

            if (gamepad2.dpad_left) turretCorrection -= 0.05;
            if (gamepad2.dpad_right) turretCorrection += 0.05;
            if (gamepad2.dpad_up) distanceCorrection += 0.05;
            if (gamepad2.dpad_down) distanceCorrection -= 0.05;

            Shooter.TurretAimCommand aimCommand = null;
            if (shooterOn) {
                if (autoTurret) {
                    aimCommand = robot.shooter.updateAutoAimTurret(current, TARGET_X, TARGET_Y, turretCorrection);
                } else {
                    robot.shooter.turretToDegree(0.0);
                }

                if (autoShooter) {
                    robot.shooter.setShooterByDis(distance + distanceCorrection);
                } else {
                    robot.shooter.setShooterVelocity(manualShooterVelocity);
                    robot.shooter.panelTo(manualHoodPos);
                }

                robot.intake.gateOpen();
            } else {
                robot.intake.gateClose();
                robot.shooter.shooterHold();
                robot.shooter.turretToDegree(0.0);
            }

            if (gamepad1.right_trigger > 0.05) {
                robot.intake.setIntakePower(shooterOn ? robot.shooter.calculateIntakePower() : Intake.INTAKE_POWER);
            } else if (gamepad1.left_trigger > 0.05) {
                robot.intake.setIntakePower(-gamepad1.left_trigger);
            } else {
                robot.intake.setIntakePower(0.0);
            }

            telemetry.addData("shooterOn", shooterOn);
            telemetry.addData("autoTurret", autoTurret);
            telemetry.addData("autoShooter", autoShooter);
            telemetry.addData("x", "%.2f", robotX);
            telemetry.addData("y", "%.2f", robotY);
            telemetry.addData("heading", "%.2f", robotHeading);
            telemetry.addData("target angle", "%.2f", targetAngle);
            telemetry.addData("turret target old geometry", "%.2f", turretTarget + turretCorrection);
            telemetry.addData("turret abs", "%.2f", robot.shooter.getTurretAbsoluteDegree());
            telemetry.addData("turret error", "%.2f", aimCommand != null ? aimCommand.error : 0.0);
            telemetry.addData("turret vel", "%.2f", aimCommand != null ? aimCommand.turretVelocityDegPerSec : robot.shooter.getAngVel());
            telemetry.addData("turret vel target", "%.2f", aimCommand != null ? aimCommand.desiredTurretVelocityDegPerSec : 0.0);
            telemetry.addData("aim locked", aimCommand != null && aimCommand.isAimLocked);
            telemetry.addData("distance", "%.2f", distance);
            telemetry.addData("shooter target", "%.2f", autoShooter ? targetVelocity : manualShooterVelocity);
            telemetry.addData("left velocity", "%.2f", robot.shooter.leftShooter.getVelocity());
            telemetry.addData("right velocity", "%.2f", robot.shooter.rightShooter.getVelocity());
            telemetry.addData("turret correction", "%.2f", turretCorrection);
            telemetry.addData("distance correction", "%.2f", distanceCorrection);
            telemetry.update();
        }
    }

    private static double normalizeDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }
}
