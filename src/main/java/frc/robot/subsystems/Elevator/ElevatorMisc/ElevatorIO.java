package frc.robot.subsystems.Elevator.ElevatorMisc;


public interface ElevatorIO {
    public static class ElevatorData {
        public double positionMeters = 0;
        public double velocityMetersPerSecond = 0;
        public double accelerationMetersPerSecondSquared = 0;
        public double leftCurrentAmps = 0;
        public double rightCurrentAmps = 0;
        public double leftAppliedVolts = 0;
        public double rightAppliedVolts = 0;
        public double leftTempCelcius = 0;
        public double rightTempCelcius = 0;
    }

    public default void updateData(ElevatorData data) {
    };

    public default void setVoltage(double volts) {
    };

    public default void setPosition(double setpointVelocity, double feedforward) {
    }

    public default void setBrakeMode(boolean enable) {

    }

}