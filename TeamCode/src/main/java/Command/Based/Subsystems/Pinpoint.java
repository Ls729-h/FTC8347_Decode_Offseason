package Command.Based.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Constants;

public class Pinpoint extends SubsystemBase {

    private final Telemetry telemetry;
    private GoBildaPinpointDriver pinpoint;
    public static final double X_OFFSET = 1.357;
    public static final double Y_OFFSET = 6.348;
    Pose2d pose;

    public Pinpoint(HardwareMap hardwareMap,Telemetry telemetry) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.PINPOINT_ID);
        pinpoint.setOffsets(X_OFFSET,Y_OFFSET, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
                );
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        this.telemetry = telemetry;
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus(){
        return pinpoint.getDeviceStatus();
    }
    public boolean isReady(){
        return getDeviceStatus()
                ==GoBildaPinpointDriver.DeviceStatus.READY;
    }
    public boolean calibrate(){
        return getDeviceStatus()
                ==GoBildaPinpointDriver.DeviceStatus.CALIBRATING;
    }

    public void recalibrateIMU(){

        pinpoint.recalibrateIMU();
    }

    public double getHeadingDegrees() {

        return pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
    }
    public double getHeadingRadians() {

        return pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS);
    }
    //function for continuous angle
    public Rotation2d getHeading() {

        return Rotation2d.fromDegrees(getHeadingRadians());
    }
    //function for geometric position calculation

    public double getPositionX(){


        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getPositionY(){


        return pinpoint.getPosY(DistanceUnit.INCH);
    }


    public void test_resetIMU(){

        pinpoint.resetPosAndIMU();
    }

    public void setPosition(double x,double y,double z){
        new Pose2d(x,y,new Rotation2d(z));
    }


    @Override
    public void periodic() {

        pinpoint.update();
        Pose2D sdkPose = pinpoint.getPosition();

        pose = new Pose2d(
                sdkPose.getX(DistanceUnit.INCH),
                sdkPose.getY(DistanceUnit.INCH),
                Rotation2d.fromDegrees(
                        sdkPose.getHeading(AngleUnit.DEGREES)
                )
        );

        GoBildaPinpointDriver.DeviceStatus Status = getDeviceStatus();

        telemetry.addData("pp_heading deg",pose.getHeading());
        telemetry.addData("pp_Pos_X",pose.getX());
        telemetry.addData("pp_Pos_Y",pose.getY());
        telemetry.addData("pp_Status",Status);

        telemetry.update();

    }

    public Pose2d getPose() {
        return pose;
    }


}
