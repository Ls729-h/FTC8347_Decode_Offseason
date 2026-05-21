package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.GATE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.INTAKE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intake {
    public static double INTAKE_POWER = 0.8;
    public static double INTAKE_SLOW_POWER = 0.35;
    public static double FIRE_POWER = 1.0;
    public static double OUTTAKE_POWER = 1.0;
    public static double HOLD_POWER = 0.2;
    public static double GATE_OPEN = 0.358;
    public static double GATE_CLOSE = 0.553;

    private DcMotorEx intake;
    private Servo gate;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        gate = hardwareMap.get(Servo.class, GATE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gateClose();
    }

    public void intakeIn() {
        intake.setPower(INTAKE_POWER);
    }

    public void intakeIn(double power) {
        intake.setPower(power);
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void intakeFire() {
        intake.setPower(FIRE_POWER);
    }

    public void intakeFire(double power) {
        intake.setPower(power);
    }

    public void intakeInSlow() {
        intake.setPower(INTAKE_SLOW_POWER);
    }

    public void intakeOut() {
        intake.setPower(-OUTTAKE_POWER);
    }

    public void intakeOut(double power) {
        intake.setPower(-power);
    }

    public void intakeHold() {
        intake.setPower(HOLD_POWER);
    }

    public void intakeStop() {
        intake.setPower(0.0);
    }

    public void gateOpen() {
        gate.setPosition(GATE_OPEN);
    }

    public void gateClose() {
        gate.setPosition(GATE_CLOSE);
    }
}
