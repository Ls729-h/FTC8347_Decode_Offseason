package Command.Based;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class Constants {
    public static  double shooter_kP_left = 0;
    public static  double shooter_kS_left = 0;
    public static  double shooter_kV_left = 0;
    public static  double shooter_kP_right = 0;
    public static  double shooter_kS_right = 0;
    public static  double shooter_kV_right = 0;

        public static final String INTAKE_TRANSFER_MOTOR_ID = "intake";

        public static final String SHOOTER_RIGHT_ID = "rightshooter";

        public static final String SHOOTER_LEFT_ID = "leftshooter";
        public static final String SERVO_ID = "servo";

//        public static final double SHOOTER_CPR = 537.6; // RPM_312

        public static final String Drivetrain_LEFT_FRONT_ID = "leftfront";
        public static final String Drivetrain_LEFT_BACK_ID = "leftback";
        public static final String Drivetrain_RIGHT_FRONT_ID = "rightfront";
        public static final String Drivetrain_RIGHT_BACK_ID = "rightback";
//        public static final double TICKS_PER_REV_SHOOTER_M = 537.6;

        public static final String TURRET_MOTOR_ID = "turret";
        public static  double turret_PkP = 0;
        public static  double turret_VkP = 0;
        public static  double turret_kS = 0;
        public static  double turret_kV = 0;
        public static final String turret_ENCODER_ID = "turretEncoder";
        public static  double TICKS_TO_RADIANS = 0;
//        public static final double ANGLE_TOLERANCE = 0;

        public static final String PINPOINT_ID = "pinpoint";


    }

