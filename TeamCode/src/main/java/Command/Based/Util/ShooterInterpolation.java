package Command.Based.Util;

public class ShooterInterpolation {

    private static final double[] DISTANCE = {1, 2};
    private static final double[] RPM = {2500, 1200};
    private static final double [] SERVO_ANGLE = {0,0.1};
    public static class ShooterProfile {
        public double rpm;
        public double servo;

        public ShooterProfile(double rpm, double servo) {
            this.rpm = rpm;
            this.servo = servo;
        }
    }
    public static ShooterProfile get(double distance) {

        double rpm = interpolate(DISTANCE, RPM, distance);
        double servo = interpolate(DISTANCE, SERVO_ANGLE, distance);

        return new ShooterProfile(rpm, servo);
    }


    private static double interpolate(double[] x, double[] y, double target) {

        if (target <= x[0]) return y[0];
        if (target >= x[x.length - 1]) return y[y.length - 1];

        for (int i = 0; i < x.length - 1; i++) {

            double x1 = x[i];
            double x2 = x[i + 1];

            if (target >= x1 && target <= x2) {

                double y1 = y[i];
                double y2 = y[i + 1];

                double t = (target - x1) / (x2 - x1);

                return y1 + t * (y2 - y1);
            }
        }

        return y[0];
    }
}



