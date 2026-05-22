package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
    }
    public void prepareShootAuto(double distance, double turretDeg) {
        intake.gateOpen();
        shooter.setShooterByDis(distance);
        shooter.turretToDegree(turretDeg);
    }
    public void stopShootAuto() {
        intake.gateClose();
        intake.intakeStop();
        shooter.shooterStop();
        shooter.stopAutoAimTurret();
    }
}
