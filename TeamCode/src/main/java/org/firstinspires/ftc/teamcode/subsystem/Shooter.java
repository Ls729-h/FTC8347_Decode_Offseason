package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.HOOD;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.INTAKE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class Shooter {
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public DcMotorEx turret;
//    public Servo panel;
    //    public Servo led;
    public static double targetVelocity = 0, spKp = 0.027, spKi = 0.0, spKd = 0.236, basePower = 0.17, tor = 0.025, hoodCorrection = 0.045;
    public static double SHOOTER_KP = 0.0;
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KF = 0.0;
    public static double TURRET_FULL_RANGE_DEGREE = 175.0;
    public static double TURRET_FULL_RANGE_ENCODER = 566.0;
    public static String TURRET_ENCODER = INTAKE;
    public static double TURRET_ENCODER_CPR = 4000.0;
    public static double TURRET_ENCODER_TO_TURRET_RATIO = 8.0;
    public static double TURRET_ENCODER_ZERO_DEG = 0.0;
    public static boolean TURRET_ENCODER_REVERSED = false;
    public static double TURRET_HEADING_SIGN = 1.0;
    public static double TURRET_GEOMETRY_SIGN = -1.0;
    public static double TURRET_POWER_TO_ANGLE_SIGN = 0.0;
    public static double TURRET_AUTO_kP = 5.0;
    public static double TURRET_AUTO_kI = 0.0;
    public static double TURRET_AUTO_kD = 0.0;
    public static double TURRET_AUTO_kV = 0.002014;
    public static double TURRET_AUTO_kS = 0.088620;
    public static double TURRET_AUTO_kA = 0.000126;
    public static double TURRET_AUTO_DEADZONE_DEG = 0.8;
    public static double TURRET_AUTO_MAX_POWER = 1.0;
    public static double TURRET_AUTO_ANGLE_FILTER_ALPHA = 0.7;
    public static double TURRET_AUTO_VEL_FILTER_ALPHA = 0.35;
    public static double TURRET_AUTO_ACCEL_FILTER_ALPHA = 0.10;
    public static double TURRET_AUTO_kLINEAR_BRAKING = 0.071480;
    public static double TURRET_AUTO_kQUADRATIC_BRAKING = 0;
    public static double TURRET_AUTO_MAX_REASONABLE_VEL_DEG_PER_SEC = 1000.0;
    public static double TURRET_AUTO_MIN_ANGLE_DEG = -141.21;
    public static double TURRET_AUTO_MAX_ANGLE_DEG = 200.00;
    public static double TURRET_AUTO_TUNING_VOLTAGE = 12.97;
    ElapsedTime dt = new ElapsedTime();
    double error, lastError, integral, derivative, lastTarget = 0.0, power = 0.0;
    GoBildaPinpointDriver shooterPP;
    private DcMotorEx turretEncoder;
    private HardwareMap hardwareMap;
    private double autoIntegral = 0.0;
    private double filteredTurretAbsAngle = 0.0;
    private double lastRawTurretAngle = 0.0;
    private double filteredTurretVel = 0.0;
    private double lastFilteredTurretVel = 0.0;
    private double filteredTurretAccel = 0.0;
    private double lastAppliedTurretPower = 0.0;
    private double learnedPowerToAngleSign = 0.0;
    private long lastAutoTime = 0;
    private long lastVoltageReadTime = 0;
    private double currentBatteryVoltage = 12.0;
    private boolean autoAimInitialized = false;

    public void init (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
//        panel = hardwareMap.get(Servo.class, HOOD);
        turretEncoder = hardwareMap.get(DcMotorEx.class, TURRET_ENCODER);
        try {
            shooterPP = hardwareMap.get(GoBildaPinpointDriver.class, "spp");
        } catch (Exception ignored) {
            shooterPP = null;
        }

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        currentBatteryVoltage = getBatteryVoltage();
    }

    public void reset() {
        resetAutoAimState();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setTurretMotorPower(0.0);
    }

    public static double turretEncoderTicksToTurretDegrees(double encoderTicks) {
        double cpr = Math.abs(TURRET_ENCODER_CPR) > 1e-9 ? Math.abs(TURRET_ENCODER_CPR) : 4000.0;
        double ratio = Math.abs(TURRET_ENCODER_TO_TURRET_RATIO) > 1e-9
                ? Math.abs(TURRET_ENCODER_TO_TURRET_RATIO)
                : 1.0;
        return encoderTicks / cpr * 360.0 / ratio;
    }

    public static double turretEncoderVelocityToTurretDegPerSec(double encoderTicksPerSecond) {
        return turretEncoderTicksToTurretDegrees(encoderTicksPerSecond);
    }

    public void setShooterVelocity(double velocity) {
        leftShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        rightShooter.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KF);
        leftShooter.setVelocity(velocity);
        rightShooter.setVelocity(velocity);
    }
    public void shooterHold() {
        leftShooter.setVelocity(1360);
        rightShooter.setVelocity(1360);
    }

    public void shooterStop() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }
    public void setShooter(double panel, double velocity) {
//        panelTo(Range.clip(panel, 0.0, 1.0));
        setShooterVelocity(velocity);
    }

    public void setShooterByDis(double distance) {
        targetVelocity = f(0, -0.0071, 7.9078, 928.47, distance * 0.985);
        setShooter(f(0.000000478433, -0.0001598871, 0.018559, -0.152044, distance) - 0.045 + hoodCorrection, targetVelocity);
    }

    public double calculateIntakePower() {
        return Range.clip(f(0, -6E-07, 0.0012, 0.5244, targetVelocity), 0.63, 1.0);
    }
    public double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }
    public void turretToDegree(double degree) {
        turretToDegPP(Range.clip(degree, TURRET_AUTO_MIN_ANGLE_DEG, TURRET_AUTO_MAX_ANGLE_DEG));
    }

    public double getAngVel() {
        return filteredTurretVel;
    }

    public void turretToDegPP(double target) {
        useTurretPowerControl();
        double elapsedMs = Math.max(1.0, dt.milliseconds());
        dt.reset();

        updateTurretFilters(elapsedMs / 1000.0);
        error = target - filteredTurretAbsAngle;
        integral += error * elapsedMs;
        derivative = -filteredTurretVel / 1000.0;
        lastError = error;
        lastTarget = target;
        power = error * spKp + integral * spKi + derivative * spKd;
        if (power > tor && power < basePower) power += basePower;
        else if (power < -tor && power > -basePower) power -= basePower;
        applyTurretAnglePower(Range.clip(power, -1.0, 1.0), 1.0);
    }

    public TurretAimCommand updateAutoAimTurret(
            Pose2D robotPose,
            double targetX,
            double targetY,
            double turretCorrectionDeg) {
        return updateAutoAimTurret(
                robotPose.getX(DistanceUnit.INCH),
                robotPose.getY(DistanceUnit.INCH),
                robotPose.getHeading(AngleUnit.DEGREES),
                targetX,
                targetY,
                turretCorrectionDeg
        );
    }

    public TurretAimCommand updateAutoAimTurret(
            double robotX,
            double robotY,
            double robotHeadingDeg,
            double targetX,
            double targetY,
            double turretCorrectionDeg) {
        long currentTime = System.nanoTime();
        double dtSeconds = (lastAutoTime == 0) ? 0.0 : (currentTime - lastAutoTime) / 1e9;
        lastAutoTime = currentTime;

        if (currentTime - lastVoltageReadTime > 250_000_000L) {
            currentBatteryVoltage = getBatteryVoltage();
            lastVoltageReadTime = currentTime;
        }

        TurretAimCommand command = new TurretAimCommand();
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double distance = Math.hypot(dx, dy);

        if (distance < 0.1) {
            stopAutoAimTurret();
            return command;
        }

        useTurretPowerControl();
        updateTurretFilters(dtSeconds);

        double geometrySign = getTurretGeometrySign();
        double signedRobotHeadingDeg = getTurretHeadingSign() * robotHeadingDeg;
        double targetFieldAngle = normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)));
        double currentTurretAbsFieldAngle = normalizeDegrees(signedRobotHeadingDeg + geometrySign * filteredTurretAbsAngle);
        double fieldErrorDeg = normalizeDegrees(targetFieldAngle - currentTurretAbsFieldAngle);
        double directTargetTurretAngle = geometrySign * normalizeDegrees(targetFieldAngle - signedRobotHeadingDeg) + turretCorrectionDeg;
        double targetTurretRelAngle = chooseReachableTurretAngle(directTargetTurretAngle, filteredTurretAbsAngle, command);
        double errorDeg = targetTurretRelAngle - filteredTurretAbsAngle;

        double tolerance = 5.0 + (1.0 - 5.0) / (150.0 - 20.0) * (distance - 20.0);
        command.hasTarget = true;
        command.targetDistance = distance;
        command.targetFieldAngle = targetFieldAngle;
        command.currentFieldAimAngle = currentTurretAbsFieldAngle;
        command.fieldError = fieldErrorDeg;
        command.directTargetTurretAngle = directTargetTurretAngle;
        command.targetTurretAngle = targetTurretRelAngle;
        command.currentTurretAngle = filteredTurretAbsAngle;
        command.error = errorDeg;
        command.currentTolerance = Range.clip(tolerance, 1.0, 5.0);
        command.isAimLocked = Math.abs(errorDeg) <= command.currentTolerance;
        command.turretVelocityDegPerSec = filteredTurretVel;
        command.turretAccelDegPerSecSq = filteredTurretAccel;

        double predictedTurretAngle = filteredTurretAbsAngle;
        if (command.isUnwinding) {
            double brakingDist = TURRET_AUTO_kLINEAR_BRAKING * Math.abs(filteredTurretVel)
                    + TURRET_AUTO_kQUADRATIC_BRAKING * filteredTurretVel * filteredTurretVel;
            predictedTurretAngle += Math.signum(filteredTurretVel) * brakingDist;
        }

        if (dtSeconds > 0.0001) {
            autoIntegral += errorDeg * dtSeconds;
            autoIntegral = Range.clip(autoIntegral, -3.0, 3.0);
        }

        double desiredTurretVel = TURRET_AUTO_kP * (targetTurretRelAngle - predictedTurretAngle)
                + TURRET_AUTO_kI * autoIntegral
                - TURRET_AUTO_kD * filteredTurretVel;
        double velocityError = desiredTurretVel - filteredTurretVel;

        double turretPower;
        if (Math.abs(errorDeg) < TURRET_AUTO_DEADZONE_DEG && Math.abs(filteredTurretVel) < 3.0) {
            turretPower = 0.0;
            autoIntegral = 0.0;
        } else {
            turretPower = -filteredTurretAccel * TURRET_AUTO_kA
                    + velocityError * TURRET_AUTO_kV
                    + Math.signum(velocityError) * TURRET_AUTO_kS;
            turretPower *= TURRET_AUTO_TUNING_VOLTAGE / currentBatteryVoltage;
        }

        command.desiredTurretVelocityDegPerSec = desiredTurretVel;
        command.velocityErrorDegPerSec = velocityError;
        command.anglePower = Range.clip(turretPower, -TURRET_AUTO_MAX_POWER, TURRET_AUTO_MAX_POWER);
        command.powerToAngleSign = getPowerToAngleSign();
        command.power = applyTurretAnglePower(command.anglePower, TURRET_AUTO_MAX_POWER);
        return command;
    }

    public void stopAutoAimTurret() {
        setTurretMotorPower(0.0);
        resetAutoAimState();
    }

    public double getTurretAbsoluteDegree() {
        return readRawTurretEncoderAngleDeg() - TURRET_ENCODER_ZERO_DEG;
    }

    public double getTurretPowerToAngleSign() {
        return getPowerToAngleSign();
    }

    private void updateTurretFilters(double dtSeconds) {
        double rawAbsAngle = getTurretAbsoluteDegree();

        if (!autoAimInitialized || dtSeconds <= 0.0001) {
            filteredTurretAbsAngle = rawAbsAngle;
            lastRawTurretAngle = rawAbsAngle;
            filteredTurretVel = 0.0;
            lastFilteredTurretVel = 0.0;
            filteredTurretAccel = 0.0;
            autoAimInitialized = true;
            return;
        }

        double angleDelta = rawAbsAngle - filteredTurretAbsAngle;
        filteredTurretAbsAngle = filteredTurretAbsAngle + TURRET_AUTO_ANGLE_FILTER_ALPHA * angleDelta;

        double rawVel = (rawAbsAngle - lastRawTurretAngle) / dtSeconds;
        rawVel = Range.clip(rawVel, -TURRET_AUTO_MAX_REASONABLE_VEL_DEG_PER_SEC, TURRET_AUTO_MAX_REASONABLE_VEL_DEG_PER_SEC);
        filteredTurretVel = TURRET_AUTO_VEL_FILTER_ALPHA * rawVel
                + (1.0 - TURRET_AUTO_VEL_FILTER_ALPHA) * filteredTurretVel;
        updatePowerToAngleSign();

        double rawAccel = (filteredTurretVel - lastFilteredTurretVel) / dtSeconds;
        filteredTurretAccel = TURRET_AUTO_ACCEL_FILTER_ALPHA * rawAccel
                + (1.0 - TURRET_AUTO_ACCEL_FILTER_ALPHA) * filteredTurretAccel;

        lastRawTurretAngle = rawAbsAngle;
        lastFilteredTurretVel = filteredTurretVel;
    }

    private double readRawTurretEncoderAngleDeg() {
        double angle = turretEncoderTicksToTurretDegrees(turretEncoder.getCurrentPosition());
        return TURRET_ENCODER_REVERSED ? -angle : angle;
    }

    private void resetAutoAimState() {
        autoAimInitialized = false;
        autoIntegral = 0.0;
        filteredTurretVel = 0.0;
        lastFilteredTurretVel = 0.0;
        filteredTurretAccel = 0.0;
        lastAutoTime = 0;
        integral = 0.0;
        lastError = 0.0;
        dt.reset();
    }

    private double applyTurretAnglePower(double anglePower, double maxPower) {
        double motorPower = Range.clip(anglePower / getPowerToAngleSign(), -maxPower, maxPower);
        setTurretMotorPower(motorPower);
        return motorPower;
    }

    private void setTurretMotorPower(double motorPower) {
        lastAppliedTurretPower = motorPower;
        turret.setPower(motorPower);
    }

    private void updatePowerToAngleSign() {
        if (Math.abs(TURRET_POWER_TO_ANGLE_SIGN) > 0.5) return;
        if (Math.abs(learnedPowerToAngleSign) > 0.5) return;
        if (Math.abs(lastAppliedTurretPower) < 0.08 || Math.abs(filteredTurretVel) < 2.0) return;

        learnedPowerToAngleSign = Math.signum(filteredTurretVel) * Math.signum(lastAppliedTurretPower);
    }

    private double getPowerToAngleSign() {
        if (Math.abs(TURRET_POWER_TO_ANGLE_SIGN) > 0.5) {
            return Math.signum(TURRET_POWER_TO_ANGLE_SIGN);
        }
        if (Math.abs(learnedPowerToAngleSign) > 0.5) {
            return Math.signum(learnedPowerToAngleSign);
        }
        return 1.0;
    }

    private double getTurretGeometrySign() {
        return TURRET_GEOMETRY_SIGN < 0.0 ? -1.0 : 1.0;
    }

    private double getTurretHeadingSign() {
        return TURRET_HEADING_SIGN < 0.0 ? -1.0 : 1.0;
    }

    private double chooseReachableTurretAngle(double directTargetTurretAngle, double currentTurretAngle, TurretAimCommand command) {
        double normalizedTarget = normalizeDegrees(directTargetTurretAngle);
        double bestTarget = 0.0;
        double bestDistance = Double.POSITIVE_INFINITY;
        boolean foundReachable = false;

        for (int wrap = -2; wrap <= 2; wrap++) {
            double candidate = normalizedTarget + 360.0 * wrap;
            if (candidate < TURRET_AUTO_MIN_ANGLE_DEG || candidate > TURRET_AUTO_MAX_ANGLE_DEG) {
                continue;
            }

            double distance = Math.abs(candidate - currentTurretAngle);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTarget = candidate;
                foundReachable = true;
            }
        }

        if (foundReachable) {
            command.isUnwinding = Math.abs(bestTarget - normalizedTarget) > 1e-6;
            return bestTarget;
        }

        bestDistance = Double.POSITIVE_INFINITY;
        for (int wrap = -2; wrap <= 2; wrap++) {
            double candidate = normalizedTarget + 360.0 * wrap;
            double clipped = Range.clip(candidate, TURRET_AUTO_MIN_ANGLE_DEG, TURRET_AUTO_MAX_ANGLE_DEG);
            double distance = Math.abs(clipped - currentTurretAngle);

            if (distance < bestDistance) {
                bestDistance = distance;
                bestTarget = clipped;
            }
        }

        command.isAtLimit = true;
        command.isUnwinding = Math.abs(bestTarget - normalizedTarget) > 1e-6;
        return bestTarget;
    }

    private void useTurretPowerControl() {
        if (turret.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getBatteryVoltage() {
        if (hardwareMap == null) return TURRET_AUTO_TUNING_VOLTAGE;

        double maxVoltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > maxVoltage) {
                maxVoltage = voltage;
            }
        }
        return Math.max(8.0, maxVoltage > 0.0 ? maxVoltage : TURRET_AUTO_TUNING_VOLTAGE);
    }

    private static double normalizeDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }

    public static class TurretAimCommand {
        public boolean hasTarget = false;
        public boolean isAimLocked = false;
        public boolean isUnwinding = false;
        public boolean isAtLimit = false;
        public double targetDistance = 0.0;
        public double targetFieldAngle = 0.0;
        public double currentFieldAimAngle = 0.0;
        public double fieldError = 0.0;
        public double directTargetTurretAngle = 0.0;
        public double targetTurretAngle = 0.0;
        public double currentTurretAngle = 0.0;
        public double currentTolerance = 1.0;
        public double error = 0.0;
        public double power = 0.0;
        public double anglePower = 0.0;
        public double powerToAngleSign = 1.0;
        public double desiredTurretVelocityDegPerSec = 0.0;
        public double velocityErrorDegPerSec = 0.0;
        public double turretVelocityDegPerSec = 0.0;
        public double turretAccelDegPerSecSq = 0.0;
    }
}
