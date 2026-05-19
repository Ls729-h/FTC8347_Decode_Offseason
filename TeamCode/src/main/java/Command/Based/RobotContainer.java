package Command.Based;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.panels.Panels;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Commands.DrivetrainCmd;
import Command.Based.Commands.IntakeGetBallCmd;
import Command.Based.Commands.Mapping_drive_test;
import Command.Based.Commands.ShooterCmd;
import Command.Based.Commands.TurretAimCmd;
import Command.Based.Subsystems.Drivetrain;
import Command.Based.Subsystems.IntakeTransfer;
import Command.Based.Subsystems.Limelight;
import Command.Based.Subsystems.Pinpoint;
import Command.Based.Subsystems.Shooter;
import Command.Based.Subsystems.Turret;
import Command.Based.Util.TargetingUtil;

public class RobotContainer {

    public final Shooter shooter;
    public final IntakeTransfer intake;
    public final Drivetrain drive;
    private final Pinpoint pinpoint;
    private final GamepadEx gamepad;
    private final Telemetry telemetry;
    private final Limelight limelight;
    //    private final Turret turret;
    Translation2d target = TargetingUtil.BLUE_GOAL;


    public RobotContainer(HardwareMap hardwareMap,Gamepad gamepad2,Telemetry telemetry) {
        this.shooter = new Shooter(hardwareMap);
        this.intake = new IntakeTransfer(hardwareMap);
        this.drive = new Drivetrain(hardwareMap);
        this.pinpoint = new Pinpoint(hardwareMap,telemetry);
//        this.turret = new Turret(hardwareMap,telemetry);

        this.gamepad = new GamepadEx(gamepad2);
        this.telemetry = telemetry;
        this.limelight = new Limelight(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(drive);

        configureBindings();

    }
    public void configureBindings() {


        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(new IntakeGetBallCmd(intake));

        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(new ShooterCmd(shooter,
                        pinpoint,
                        target,telemetry,
//                        intake
                        limelight
                ));


        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new InstantCommand(pinpoint::test_resetIMU));
//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                        .whenPressed(new Mapping_drive_test(drive));
        drive.setDefaultCommand(
                new DrivetrainCmd(
                        drive,
                        pinpoint,
                        telemetry,
                        gamepad
                )
        );


//        turret.setDefaultCommand(
//         new TurretAimCmd(turret, pinpoint)
//        );
//        gamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(drive::changeAlliance);


    }

}
