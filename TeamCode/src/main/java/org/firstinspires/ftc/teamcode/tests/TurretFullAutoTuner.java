package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TURRET;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.Shooter;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "Turret Full Auto Tuner", group = "Tuning")
public class TurretFullAutoTuner extends OpMode {
    private DcMotorEx turret;
    private DcMotorEx turretEncoder;

    public static double MAX_SAFE_ANGLE = 120.0;
    public static double BRAKE_BUFFER_ANGLE = 35.0;
    public static double RETURN_POWER = 0.3;
    public static int REST_TIME_MS = 500;

    public static double[] KV_TEST_POWERS = {0.3, 0.45, 0.6, 0.75, 0.8, 0.9, 1.0};
    public static double[] KA_TEST_POWERS = {0.5, 0.7, 0.9, 1.0};
    public static double[] PB_TEST_POWERS = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
    public static double PB_BRAKING_POWER = 0.3;
    public static int PB_SPIN_TIME_MS = 3000;

    public static double VEL_FILTER_ALPHA = Shooter.TURRET_AUTO_VEL_FILTER_ALPHA;
    public static double ACCEL_FILTER_ALPHA = Shooter.TURRET_AUTO_ACCEL_FILTER_ALPHA;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double POWER_TO_ANGLE_SIGN = 0.0;

    private enum State {
        START,
        KS_RUN, KS_RETURN, KS_WAIT,
        KV_RUN, KV_RETURN, KV_WAIT,
        KA_RUN, KA_RETURN, KA_WAIT,
        PB_RUN, PB_BRAKE, PB_RETURN, PB_WAIT,
        CALCULATE, DONE
    }

    private State state = State.START;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private long lastTime = 0;
    private double lastEncoderAngle = 0.0;
    private double filteredVel = 0.0;
    private double lastFilteredVel = 0.0;
    private double filteredAccel = 0.0;
    private boolean isInitialized = false;

    private double currentVoltage = 12.0;
    private double sumVoltage = 0.0;
    private int voltageSamples = 0;

    private int iteration = 0;
    private int currentDirection = 1;
    private double currentTestPower = 0.0;
    private double lastAppliedPower = 0.0;
    private double learnedPowerToAngleSign = 0.0;
    private double pbStartBrakeAngle = 0.0;
    private double pbMeasuredBrakeVel = 0.0;

    private final List<Double> ksSamples = new ArrayList<>();
    private final List<double[]> kvData = new ArrayList<>();
    private final List<double[]> kaData = new ArrayList<>();
    private final List<double[]> pbData = new ArrayList<>();

    private double rawKs = 0.0;
    private double rawKv = 0.0;
    private double rawKa = 0.0;
    private double rawKLin = 0.0;
    private double rawKQuad = 0.0;
    private double compKs = 0.0;
    private double compKv = 0.0;
    private double compKa = 0.0;
    private double compKLin = 0.0;
    private double compKQuad = 0.0;
    private double avgTestVoltage = 0.0;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, TURRET);
        turretEncoder = hardwareMap.get(DcMotorEx.class, Shooter.TURRET_ENCODER);

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready to tune turret. ELC uses the intake motor encoder port.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        voltageTimer.reset();
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        if (dt <= 0.00001) return;
        lastTime = currentTime;

        double currentAngle = getEncoderAngleDeg();

        if (voltageTimer.milliseconds() > 250 && state != State.DONE && state != State.CALCULATE) {
            currentVoltage = getBatteryVoltage();
            sumVoltage += currentVoltage;
            voltageSamples++;
            voltageTimer.reset();
        }

        if (!isInitialized) {
            lastEncoderAngle = currentAngle;
            filteredVel = 0.0;
            lastFilteredVel = 0.0;
            filteredAccel = 0.0;
            isInitialized = true;
            return;
        }

        double rawVel = (currentAngle - lastEncoderAngle) / dt;
        rawVel = Range.clip(rawVel, -Shooter.TURRET_AUTO_MAX_REASONABLE_VEL_DEG_PER_SEC, Shooter.TURRET_AUTO_MAX_REASONABLE_VEL_DEG_PER_SEC);
        filteredVel = VEL_FILTER_ALPHA * rawVel + (1.0 - VEL_FILTER_ALPHA) * filteredVel;

        double rawAccel = (filteredVel - lastFilteredVel) / dt;
        filteredAccel = ACCEL_FILTER_ALPHA * rawAccel + (1.0 - ACCEL_FILTER_ALPHA) * filteredAccel;

        lastEncoderAngle = currentAngle;
        lastFilteredVel = filteredVel;
        updatePowerToAngleSign();

        double limitThreshold = MAX_SAFE_ANGLE - BRAKE_BUFFER_ANGLE;
        int currentAngleDirection = angleDirectionForPower(currentDirection);
        boolean hitSafetyLimit = (currentAngle >= limitThreshold && currentAngleDirection > 0)
                || (currentAngle <= -limitThreshold && currentAngleDirection < 0);

        if (Math.abs(currentAngle) > MAX_SAFE_ANGLE + 15.0
                && !isReturningState(state)
                && state != State.DONE
                && state != State.CALCULATE) {
            setTurretPower(0);
            telemetry.addLine("Past safe turret angle. Calculating with collected samples.");
            switchState(State.CALCULATE);
        }

        switch (state) {
            case START:
                iteration = 0;
                ksSamples.clear();
                kvData.clear();
                kaData.clear();
                pbData.clear();
                switchState(State.KS_RUN);
                break;

            case KS_RUN:
                if (iteration >= 2) {
                    rawKs = average(ksSamples);
                    iteration = 0;
                    switchState(State.KV_RUN);
                    break;
                }

                currentDirection = iteration % 2 == 0 ? 1 : -1;
                currentTestPower += 0.1 * dt;
                setTurretPower(currentTestPower * currentDirection);

                if (Math.abs(filteredVel) > 2.0 || hitSafetyLimit) {
                    ksSamples.add(currentTestPower);
                    currentTestPower = 0.0;
                    setTurretPower(0);
                    switchState(State.KS_RETURN);
                }
                break;

            case KS_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    setTurretPower(0);
                    switchState(State.KS_WAIT);
                }
                break;

            case KS_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KS_RUN);
                }
                break;

            case KV_RUN:
                if (iteration >= KV_TEST_POWERS.length * 2) {
                    iteration = 0;
                    rawKv = calculateLeastSquaresSlope(kvData);
                    switchState(State.KA_RUN);
                    break;
                }

                currentDirection = iteration % 2 == 0 ? 1 : -1;
                double kvPower = KV_TEST_POWERS[iteration / 2];
                setTurretPower(kvPower * currentDirection);

                if (timer.milliseconds() > 700 || hitSafetyLimit) {
                    if (Math.abs(filteredVel) > 3.0) {
                        kvData.add(new double[]{Math.abs(filteredVel), Math.max(0.0, kvPower - rawKs)});
                    }
                    setTurretPower(0);
                    switchState(State.KV_RETURN);
                }
                break;

            case KV_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    setTurretPower(0);
                    switchState(State.KV_WAIT);
                }
                break;

            case KV_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KV_RUN);
                }
                break;

            case KA_RUN:
                if (iteration >= KA_TEST_POWERS.length * 2) {
                    iteration = 0;
                    switchState(State.PB_RUN);
                    break;
                }

                currentDirection = iteration % 2 == 0 ? 1 : -1;
                double kaPower = KA_TEST_POWERS[iteration / 2];
                setTurretPower(kaPower * currentDirection);

                if (timer.milliseconds() < 300 && Math.abs(filteredAccel) > 10.0) {
                    double netPower = Math.max(0.0, kaPower - rawKs - rawKv * Math.abs(filteredVel));
                    kaData.add(new double[]{Math.abs(filteredAccel), netPower});
                }

                if (timer.milliseconds() > 400 || hitSafetyLimit) {
                    setTurretPower(0);
                    switchState(State.KA_RETURN);
                }
                break;

            case KA_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    setTurretPower(0);
                    switchState(State.KA_WAIT);
                }
                break;

            case KA_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KA_RUN);
                }
                break;

            case PB_RUN:
                if (iteration >= PB_TEST_POWERS.length * 2) {
                    switchState(State.CALCULATE);
                    break;
                }

                currentDirection = iteration % 2 == 0 ? 1 : -1;
                double pbPower = PB_TEST_POWERS[iteration / 2] * currentDirection;
                setTurretPower(pbPower);

                if (timer.milliseconds() >= PB_SPIN_TIME_MS || hitSafetyLimit) {
                    pbMeasuredBrakeVel = filteredVel;
                    pbStartBrakeAngle = currentAngle;
                    setTurretPower(-currentDirection * PB_BRAKING_POWER);
                    switchState(State.PB_BRAKE);
                }
                break;

            case PB_BRAKE:
                if (((Math.signum(filteredVel) != Math.signum(pbMeasuredBrakeVel) || Math.abs(filteredVel) < 2.0)
                        && timer.milliseconds() > 150)
                        || timer.milliseconds() > 3000) {
                    setTurretPower(0);
                    pbData.add(new double[]{Math.abs(pbMeasuredBrakeVel), Math.abs(currentAngle - pbStartBrakeAngle)});
                    switchState(State.PB_RETURN);
                }
                break;

            case PB_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    setTurretPower(0);
                    switchState(State.PB_WAIT);
                }
                break;

            case PB_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.PB_RUN);
                }
                break;

            case CALCULATE:
                avgTestVoltage = voltageSamples > 0 ? sumVoltage / voltageSamples : getBatteryVoltage();
                rawKa = Math.max(0.0, calculateLeastSquaresSlope(kaData));
                calculatePbCoefficients();

                double voltageScaleRatio = avgTestVoltage / NOMINAL_VOLTAGE;
                compKs = rawKs * voltageScaleRatio;
                compKv = rawKv * voltageScaleRatio;
                compKa = rawKa * voltageScaleRatio;
                compKLin = rawKLin * voltageScaleRatio;
                compKQuad = rawKQuad * voltageScaleRatio;

                switchState(State.DONE);
                break;

            case DONE:
                setTurretPower(0);
                telemetry.addLine("=== TUNING COMPLETE ===");
                telemetry.addData("Data Points", "kS:%d, kV:%d, kA:%d, PB:%d", ksSamples.size(), kvData.size(), kaData.size(), pbData.size());
                telemetry.addData("Avg Test Voltage", "%.2f V", avgTestVoltage);
                telemetry.addData("Shooter.TURRET_AUTO_kS", "%.6f", compKs);
                telemetry.addData("Shooter.TURRET_AUTO_kV", "%.6f", compKv);
                telemetry.addData("Shooter.TURRET_AUTO_kA", "%.6f", compKa);
                telemetry.addData("Shooter.TURRET_AUTO_kLINEAR_BRAKING", "%.6f", compKLin);
                telemetry.addData("Shooter.TURRET_AUTO_kQUADRATIC_BRAKING", "%.6f", compKQuad);
                telemetry.update();
                return;
        }

        telemetry.addData("State", state);
        telemetry.addData("Voltage", "%.2f V", currentVoltage);
        telemetry.addData("ELC Angle", "%.1f deg", currentAngle);
        telemetry.addData("ELC Ticks", turretEncoder.getCurrentPosition());
        telemetry.addData("ELC Vel", "%.1f deg/s", filteredVel);
        telemetry.addData("ELC Accel", "%.1f deg/s^2", filteredAccel);
        telemetry.addData("Power->Angle Sign", "%.0f", getPowerToAngleSign());
        telemetry.addData("Iteration", iteration);
        telemetry.update();
    }

    private void switchState(State newState) {
        state = newState;
        timer.reset();
    }

    private double getBatteryVoltage() {
        double maxVoltage = 0.0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > maxVoltage) {
                maxVoltage = voltage;
            }
        }
        return maxVoltage > 0.0 ? maxVoltage : NOMINAL_VOLTAGE;
    }

    private boolean isReturningState(State s) {
        return s == State.KS_RETURN
                || s == State.KV_RETURN
                || s == State.KA_RETURN
                || s == State.PB_RETURN
                || s == State.PB_BRAKE;
    }

    private boolean returnToCenter(double currentAngle) {
        if (Math.abs(currentAngle) > 5.0) {
            double returnPower = -Math.signum(currentAngle) * RETURN_POWER / getPowerToAngleSign();
            setTurretPower(returnPower);
            return false;
        }

        setTurretPower(0);
        return true;
    }

    private void setTurretPower(double power) {
        lastAppliedPower = power;
        turret.setPower(power);
    }

    private void updatePowerToAngleSign() {
        if (Math.abs(POWER_TO_ANGLE_SIGN) > 0.5) return;
        if (Math.abs(learnedPowerToAngleSign) > 0.5) return;
        if (Math.abs(lastAppliedPower) < 0.08 || Math.abs(filteredVel) < 2.0) return;

        learnedPowerToAngleSign = Math.signum(filteredVel) * Math.signum(lastAppliedPower);
    }

    private double getPowerToAngleSign() {
        if (Math.abs(POWER_TO_ANGLE_SIGN) > 0.5) {
            return Math.signum(POWER_TO_ANGLE_SIGN);
        }
        if (Math.abs(learnedPowerToAngleSign) > 0.5) {
            return Math.signum(learnedPowerToAngleSign);
        }
        return 1.0;
    }

    private int angleDirectionForPower(int powerDirection) {
        return (int) Math.signum(powerDirection * getPowerToAngleSign());
    }

    private double getEncoderAngleDeg() {
        double angle = Shooter.turretEncoderTicksToTurretDegrees(turretEncoder.getCurrentPosition());
        if (Shooter.TURRET_ENCODER_REVERSED) {
            angle = -angle;
        }
        return angle - Shooter.TURRET_ENCODER_ZERO_DEG;
    }

    private double average(List<Double> data) {
        if (data.isEmpty()) return 0.0;

        double sum = 0.0;
        for (Double value : data) {
            sum += value;
        }
        return sum / data.size();
    }

    private double calculateLeastSquaresSlope(List<double[]> data) {
        if (data.isEmpty()) return 0.0;

        double sumXY = 0.0;
        double sumXX = 0.0;
        for (double[] point : data) {
            sumXY += point[0] * point[1];
            sumXX += point[0] * point[0];
        }
        return sumXX == 0.0 ? 0.0 : sumXY / sumXX;
    }

    private void calculatePbCoefficients() {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumXX = 0.0;
        int n = 0;

        for (double[] point : pbData) {
            double velocity = point[0];
            double distance = point[1];
            if (velocity < 5.0) continue;

            double x = velocity;
            double y = distance / velocity;

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
            n++;
        }

        if (n > 1) {
            double denominator = n * sumXX - sumX * sumX;
            if (Math.abs(denominator) < 1e-9) return;

            rawKQuad = (n * sumXY - sumX * sumY) / denominator;
            rawKLin = (sumY * sumXX - sumX * sumXY) / denominator;

            rawKQuad = Math.max(0.0, rawKQuad);
            rawKLin = Math.max(0.0, rawKLin);
        }
    }

    private static double normalizeDegrees(double degrees) {
        while (degrees > 180.0) degrees -= 360.0;
        while (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }
}
