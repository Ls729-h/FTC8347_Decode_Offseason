package Command.Based.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Constants;
import Command.Based.Subsystems.IntakeTransfer;
import Command.Based.Subsystems.Limelight;
import Command.Based.Util.ShooterInterpolation;
import Command.Based.Subsystems.Pinpoint;
import Command.Based.Subsystems.Shooter;
import Command.Based.Util.TargetingUtil;


public class ShooterCmd extends CommandBase {
    private final Shooter shooter;
   private final Pinpoint pinpoint;
    private final Translation2d Target;
    private final Telemetry telemetry;
    private final Limelight limelight;
//    private final IntakeTransfer intake;

    public ShooterCmd(Shooter shooter,
                      Pinpoint pinpoint,
                      Translation2d Target, Telemetry telemetry,
//                      IntakeTransfer intake,
                      Limelight limelight
    ) {
        this.shooter = shooter;
        this.pinpoint = pinpoint;
        this.Target = Target;
        this.telemetry = telemetry;
//        this.intake = intake;
        this.limelight = limelight;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        pinpoint.setPosition(0,0,0);
        
    }



    @Override
    public void execute() {
        Pose2d robotPose;
        Pose2d VisionPose = limelight.AbsolutePosition(pinpoint.getHeadingDegrees());
        if (VisionPose != null){
            robotPose = VisionPose;
            pinpoint.setPosition(VisionPose.getX(),VisionPose.getY(),VisionPose.getHeading());
        }
        else {
            robotPose = pinpoint.getPose();
        }

        double distance = TargetingUtil.getDistanceToTarget(robotPose,Target);
        ShooterInterpolation.ShooterProfile profile = ShooterInterpolation.get(distance);
        double rpm = profile.rpm;
//        double servo = profile.servo;
//        shooter.setShooterAngle(0.2);

//       intake.suckArtifacts(1);
        shooter.setTargetV(1);

//        shooter.setTargetV(0.8);
//        double targetV = shooter.getTargetV();
//        telemetry.addData("target V",targetV);

//        telemetry.addData("shooter_RPM",rpm);
//        telemetry.addData("servo_ANGLE",servo);
//        telemetry.update();

    }


    @Override
    public boolean isFinished(){
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();

    }
}
