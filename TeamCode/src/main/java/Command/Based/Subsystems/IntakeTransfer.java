package Command.Based.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Command.Based.Constants;


public class IntakeTransfer extends SubsystemBase {
    private MotorEx intaketransfer;


    //hardware connection
    public IntakeTransfer(final HardwareMap hardwareMap) {
        intaketransfer = new MotorEx(hardwareMap, Constants.INTAKE_TRANSFER_MOTOR_ID);
        intaketransfer.setInverted(false);
        intaketransfer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }


    public void suckArtifacts(double power) {


        intaketransfer.set(power);
    }

    public void spitArtifacts(){


        intaketransfer.set(-1);
    }

    public void holdArtifacts(){


        intaketransfer.set(0.2);
    }

    public void stop(){


        intaketransfer.set(0);
    }

}
