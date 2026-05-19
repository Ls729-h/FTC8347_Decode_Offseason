package Command.Based.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Constants;

public class Turret extends SubsystemBase {

    public final MotorEx turret;
    public final PIDController positionPID;
    public final PIDController velocityPID;
    public final SimpleMotorFeedforward feedforward;
    public final AnalogInput encoder;
    private final Telemetry telemetry;
    private double ANGLE_ZEROOFFSET;
    public double targetAngle = 0.0;
    public double targetVelocity = 0.0;
    private double angle;
    private double output;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry){

        turret = new MotorEx(hardwareMap, Constants.TURRET_MOTOR_ID);
        turret.setInverted(false);
        turret.setRunMode(Motor.RunMode.RawPower);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.positionPID = new PIDController(Constants.turret_PkP,0,0);
        this.velocityPID = new PIDController(Constants.turret_VkP,0,0);
        this.feedforward = new SimpleMotorFeedforward(Constants.turret_kS,Constants.turret_kV);

        encoder = hardwareMap.get(AnalogInput.class,Constants.turret_ENCODER_ID);

        this.telemetry = telemetry;


    }
    public void zeroEncoder(){
        ANGLE_ZEROOFFSET= (encoder.getVoltage() / encoder.getMaxVoltage()) * 2.0 * Math.PI;


    }


    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
           angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public double getCurrentAngle(){
        angle= (encoder.getVoltage() / encoder.getMaxVoltage()) * 2.0 * Math.PI;

        return normalizeAngle(angle-ANGLE_ZEROOFFSET);

    }
    public double getCurrentVelocity() {
        return turret.getVelocity();
    }


    public void setControl(double targetAngle) {

        this.targetAngle = targetAngle;

        double error = normalizeAngle(targetAngle-getCurrentAngle());
        targetVelocity = positionPID.calculate(
                    0,
                    error
        );
        targetVelocity = MathUtils.clamp(
                targetVelocity,
                -1,1
        );
        output = velocityPID.calculate(
                getCurrentVelocity(),
                targetVelocity
        ) + feedforward.calculate(targetVelocity);

        turret.set(output);
    }
//    public boolean atTarget() {
//        return Math.abs(
//                normalizeAngle(targetAngle-getCurrentAngle())
//        )
//                < Constants.Turret.ANGLE_TOLERANCE;
//    }
    public void stop(){
        turret.set(0);
    }

    @Override
    public void periodic() {

        telemetry.addData("encoder voltage", encoder.getVoltage());
        telemetry.addData("target angle", targetAngle);
        telemetry.addData("current angle", getCurrentAngle());

        telemetry.addData("angle error",
                normalizeAngle(
                        targetAngle - getCurrentAngle()
                )
        );


        telemetry.addData("current velocity", getCurrentVelocity());
        telemetry.addData("target velocity",targetVelocity);
        telemetry.addData("output",output);


    }



}
