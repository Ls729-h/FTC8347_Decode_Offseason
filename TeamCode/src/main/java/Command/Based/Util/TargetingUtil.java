package Command.Based.Util;

import static Command.Based.RobotState.RobotState.alliance;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import Command.Based.RobotState.Alliance;


public class TargetingUtil {

    public void turretTargetingUtil() {}
    public static final Translation2d BLUE_GOAL = new Translation2d(0, 144);

    public static final Translation2d RED_GOAL = new Translation2d(144, 144);
    public static Translation2d getDelta(Pose2d robotPose, Translation2d target) {

            return target.minus(robotPose.getTranslation());

        }

    public static double calculateTurretAngle(Pose2d robotPose,Translation2d target){


        Translation2d delta = getDelta(robotPose, target);
        double field_angle = Math.atan2(delta.getY(),delta.getX());
        double Angle = field_angle - robotPose.getHeading();
        if (Angle >= Math.PI){
            Angle = Angle - 2 * Math.PI;
            return Angle;

        }
        else if (Angle <= -Math.PI){
            Angle = Angle + 2 * Math.PI;
            return Angle;
        }
        else{
            return Angle;
        }


    }

    public static double getDistanceToTarget(Pose2d robotPose,Translation2d target){
        double distance = Math.sqrt(
                Math.pow(robotPose.getX()- target.getX(), 2)
                +  Math.pow(robotPose.getY()- target.getY(), 2)
        );
        return distance;
    }




}
