package Command.Based.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import Command.Based.Constants;

public class Shooter extends SubsystemBase {

    private final MotorEx shooter_left;
    private final MotorEx shooter_right;

//    private final Servo angleServo;

    private PIDController shooter_PID_left;
    private PIDController shooter_PID_right ;

    private SimpleMotorFeedforward  shooter_FF_left;
    private SimpleMotorFeedforward shooter_FF_right;
    public double TargetV;
//    private double targetServoPos;

    public Shooter(HardwareMap hardwareMap) {
        shooter_left = new MotorEx(hardwareMap,Constants.SHOOTER_LEFT_ID);
        shooter_right = new MotorEx(hardwareMap,Constants.SHOOTER_RIGHT_ID);

        shooter_right.setInverted(false);
        shooter_left.setInverted(true);

        shooter_left.setRunMode(Motor.RunMode.RawPower);
        shooter_right.setRunMode(Motor.RunMode.RawPower);

        shooter_PID_left = new PIDController(Constants.shooter_kP_left,0,0);
        shooter_FF_left = new SimpleMotorFeedforward(Constants.shooter_kS_left,Constants.shooter_kV_left);

        shooter_PID_right = new PIDController(Constants.shooter_kP_right,0,0);
        shooter_FF_right = new SimpleMotorFeedforward(Constants.shooter_kS_right,Constants.shooter_kV_right);

//        angleServo = hardwareMap.get(Servo.class, Constants.shooter.SERVO_ID);

    }

//    public void setShooterAngle(double pos) {this.targetServoPos = pos;
    public void stop(){
        TargetV = 0;
    }


    public double getMotorV_left(){
        double leftV = shooter_left.getVelocity();
        return leftV;
    }

    public double getMotorV_right(){
        double rightV = shooter_right.getVelocity();
        return rightV;
    }

    public void setTargetV(double TargetV){

        this.TargetV = TargetV;
    }
    public double getTargetV() {
        return TargetV;

    }


    @Override
    public void periodic() {
//         This method will be called once per scheduler run
        double CurrentV_right = getMotorV_right();
        double CurrentV_left = getMotorV_left();
        double TargetV = getTargetV();

        double PID_Output_right = shooter_PID_right.calculate(CurrentV_right, TargetV);
        double FF_Output_right = shooter_FF_right.calculate(TargetV);
        double Output_right = PID_Output_right + FF_Output_right;
        Output_right = Math.max(-1, Math.min(1, Output_right));
        shooter_right.set(Output_right);

        double PID_Output_left = shooter_PID_left.calculate(CurrentV_left, TargetV);
        double FF_Output_left = shooter_FF_left.calculate(TargetV);
        double Output_left = PID_Output_left + FF_Output_left;
        Output_left = Math.max(-1, Math.min(1, Output_left));
        shooter_left.set(Output_left);
    }

//        angleServo.setPosition(targetServoPos);

}
