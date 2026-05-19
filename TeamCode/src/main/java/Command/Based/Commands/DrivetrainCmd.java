package Command.Based.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Subsystems.Drivetrain;
import Command.Based.Subsystems.Pinpoint;
import java.util.function.DoubleSupplier;


public class DrivetrainCmd extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pinpoint pinpoint;
    public final GamepadEx gamepad;
//    private  double forward;
//    private double strafe;
//    private double turn;
    private Telemetry telemetry;



    public DrivetrainCmd (Drivetrain drivetrain, Pinpoint pinpoint, Telemetry telemetry, GamepadEx gamepad) {
        this.drivetrain = drivetrain;
        this.pinpoint = pinpoint;
        this.gamepad = gamepad;


        this.telemetry = telemetry;
        addRequirements(drivetrain);
    }


    @Override
    public void initialize() {
        pinpoint.recalibrateIMU();
    }

    @Override
    public void execute() {

//        Rotation2d robotAngle = Pinpoint.getHeading();
//        Rotation2d offsetAngle = Drive.getAllianceOffset();
//
//        Rotation2d adjustAngle = robotAngle.plus(offsetAngle);
        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();
        double rot = gamepad.getRightX();
        double heading_deg = pinpoint.getHeadingDegrees();
        drivetrain.drive(
                x,
                y,
                rot,
                heading_deg
        );


        telemetry.addData("forward",y);
        telemetry.addData("strafe", x);
        telemetry.addData("turn", rot);
        telemetry.addData("heading_deg",heading_deg);


        telemetry.update();

    }

    @Override
    public boolean isFinished() {
     return false;

    }

    @Override
    public void end(boolean interrupted) {


    }
}
