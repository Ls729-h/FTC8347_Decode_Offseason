package Command.Based.Commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Command.Based.Subsystems.IntakeTransfer;

public class IntakeGetBallCmd extends CommandBase {

    private final IntakeTransfer intake;
//    private final Telemetry telemetry;
    public IntakeGetBallCmd(IntakeTransfer intake) {
        this.intake = intake;
//        this.telemetry = telemetry;
       addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {


        intake.suckArtifacts(1);

    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        intake.stop();
    }



}



