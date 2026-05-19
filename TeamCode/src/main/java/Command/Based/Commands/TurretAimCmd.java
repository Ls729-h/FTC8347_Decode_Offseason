package Command.Based.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import Command.Based.RobotState.Alliance;
import Command.Based.RobotState.RobotState;
import Command.Based.Subsystems.Pinpoint;
import Command.Based.Subsystems.Turret;
import Command.Based.Util.TargetingUtil;

public class TurretAimCmd extends CommandBase {
//    private final Turret Turret;
//    private final Pinpoint Pinpoint;
//    private Translation2d Target;
//
//
//    public TurretAimCmd(Turret Turret, Pinpoint Pinpoint) {
//        this.Turret = Turret;
//        this.Pinpoint = Pinpoint;
//        addRequirements(Turret);
//    }
//
//    @Override
//    public void initialize() {
//        Turret.zeroEncoder();
//        if (RobotState.alliance == Alliance.BLUE) {
//            Target = TargetingUtil.BLUE_GOAL;
//        } else {
//            Target = TargetingUtil.RED_GOAL;
//        }
//
//    }
//
//    @Override
//    public void execute() {
//        Pose2d robotPose = Pinpoint.getPose();
//        double targetAngle = TargetingUtil.calculateTurretAngle(
//                robotPose,Target);
//
//        Turret.setControl(targetAngle);
//
//
//
//    }
//
//    @Override
//    public boolean isFinished() {
//
//        return false;
//
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        Turret.stop();
//
//    }


}
