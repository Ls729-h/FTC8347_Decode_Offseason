package Command.Based.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Subsystems.Drivetrain;
import Command.Based.Subsystems.Pinpoint;
import java.util.function.DoubleSupplier;

public class Mapping_drive_test extends CommandBase {
    private final Drivetrain drivetrain;




    public Mapping_drive_test (Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
//        drivetrain.test_backleft();


    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {


    }
}