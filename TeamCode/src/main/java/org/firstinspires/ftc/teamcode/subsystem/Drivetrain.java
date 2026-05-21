package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PIN_POINT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FRONT;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
public class Drivetrain {
    private DcMotorEx leftFront = null;
    public DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    public GoBildaPinpointDriver pinPoint;
    private double theta, power, turn, realTheta;
    private double headingError = 0.0, currentHeading = 0.0, lastHeadingError = 0.0, headingCorrection = 0.0;

    public void init (HardwareMap hardwareMap) {
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, PIN_POINT);
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pinPoint.setOffsets(151.45, 15.05, DistanceUnit.MM);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinPoint.resetPosAndIMU();
    }
    public void drive(GamepadManager gamepad, double powerScale) {
        double y = -gamepad.getLeftStickY(), x = gamepad.getLeftStickX(), rx = gamepad.getRightStickX() * 0.85;
        leftFront.setPower((y + x + rx) * powerScale);
        leftBack.setPower((y - x + rx) * powerScale);
        rightFront.setPower((y - x - rx) * powerScale);
        rightBack.setPower((y + x - rx) * powerScale);
    }

    public double getHeading() {
        return pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public void driveFieldOriented(Gamepad gamepad, boolean aim, double target) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x;
        pinPoint.update();
        currentHeading = (pinPoint.getHeading(AngleUnit.RADIANS) + 2 * Math.PI) % (Math.PI * 2) - Math.PI;
        theta = Math.atan2(y, x) * 180 / Math.PI;
        power = Math.hypot(x, y);

        realTheta = (360 - pinPoint.getPosition().getHeading(AngleUnit.DEGREES)) + theta;

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));
        turn = rx;

        double leftFrontPower = (power * cos / maxSinCos + turn);
        double rightFrontPower = (power * sin / maxSinCos - turn);
        double leftBackPower = (power * sin / maxSinCos + turn);
        double rightBackPower = (power * cos / maxSinCos - turn);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public Pose2D getPosition() {
        pinPoint.update();
        return pinPoint.getPosition();
    }
}
