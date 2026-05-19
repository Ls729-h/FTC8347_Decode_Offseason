package Command.Based.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static Command.Based.RobotState.RobotState.alliance;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;




import java.util.function.DoubleSupplier;

import Command.Based.Constants;
import Command.Based.RobotState.Alliance;
import Command.Based.RobotState.RobotState;

public class Drivetrain extends SubsystemBase {
    private final MotorEx frontright,backright,frontleft,backleft;
    private final MecanumDrive drive;

    boolean fieldCentricEnabled;

    private final double BLUE_OFFSET = -90;
    private final double RED_OFFSET = 90;

//    private final RevIMU imu;

    private double heading = 0;

    public Drivetrain(HardwareMap hardwareMap) {
            frontright = new MotorEx(hardwareMap, Constants.Drivetrain_RIGHT_FRONT_ID);
            backright = new MotorEx(hardwareMap, Constants.Drivetrain_RIGHT_BACK_ID);
            frontleft= new MotorEx(hardwareMap, Constants.Drivetrain_LEFT_FRONT_ID);
            backleft = new MotorEx(hardwareMap, Constants.Drivetrain_LEFT_BACK_ID);

            frontright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            backright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            frontleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            backleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

            frontleft.setInverted(true);
            backleft.setInverted(true);
            backright.setInverted(true);
            frontright.setInverted(true);

            drive = new MecanumDrive(frontleft, frontright, backleft, backright);



//            imu = new RevIMU(hardwareMap);
//            imu.init();

        }



//    private double deadband(double val) {
//
//        return Math.abs(val) > 0.05 ? val : 0;
//    }

    private static double[] rotateVector(
            double x,
            double y,
            double angleDeg
    ) {

        double angleRad = Math.toRadians(angleDeg);

        double rotatedX =
                x * Math.cos(angleRad)
                        - y * Math.sin(angleRad);

        double rotatedY =
                x * Math.sin(angleRad)
                        + y * Math.cos(angleRad);

        return new double[]{
                rotatedX,
                rotatedY
        };
    }

    public void drive(double strafe, double forward, double  turn, double heading) {

//        Rotation2d adjusted = robotHeading.plus(getAllianceOffset());
//        if (fieldCentricEnabled) {
//            double offset;
//            if (RobotState.alliance == Alliance.BLUE) {
//                offset = BLUE_OFFSET;
//            } else if (RobotState.alliance == Alliance.RED) {
//                offset = RED_OFFSET;
//            } else {
//                offset = 0;
//            }
//
//            double[] rotate = rotateVector(strafe, forward, offset);
//            double forward_rotated = rotate[1];
//            double strafe_rotated = rotate[0];


            drive.driveFieldCentric(strafe,
                    forward,
                    turn,
                    heading, false);

    }
//
//    public Rotation2d getAllianceOffset() {
//        switch (alliance) {
//            case BLUE:
//                return Rotation2d.fromDegrees(-90);
//            case RED:
//                return Rotation2d.fromDegrees(90);
//        }
//        return new Rotation2d();
//    }

//    public void changeAlliance(){
//        alliance = (alliance == Alliance.BLUE)
//                ? alliance.RED
//                :alliance.BLUE;
//    }
//
//    Test
//    public void test_frontright() {
//    frontright.set(1);}
//
//    public void test_backright() {
//        backright.set(1);
//    }
//
//    public void test_frontleft() {
//        frontleft.set(1);
//    }
//
//    public void test_backleft() {
//        backleft.set(1);
//    }

    @Override
    public void periodic() {

    }


}
