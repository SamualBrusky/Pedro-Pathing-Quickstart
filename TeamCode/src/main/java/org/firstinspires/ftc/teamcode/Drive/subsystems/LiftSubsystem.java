package org.firstinspires.ftc.teamcode.Drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {
    private final MotorEx m_Lift_Height_Motor;
    private final MotorEx m_Lift_Angle_Motor;

    private final double HEIGHT_PID_TOLERANCE = 0.1;
    private final double ANGLE_PID_TOLERANCE = 0.5;

    private static final double TICKS_PER_REVOLUTION = 537.6;
    private static final double INCHES_PER_REVOLUTION = 0.5;
    private static final double DEGREES_PER_REVOLUTION = 360;

    private final double kPHeight = 0.01;
    private final double kIHeight = 0;
    private final double kDHeight = 0.0000;
    private final double kFHeight = 0;

    private final double kPAngle = 0.01;
    private final double kIAngle = 0;
    private final double kDAngle = 0.0000;
    private final double kFAngle = 0;

    private final PIDFController heightPID = new PIDFController(kPHeight, kIHeight, kDHeight, kFHeight);
    private final PIDFController anglePID = new PIDFController(kPAngle, kIAngle, kDAngle, kFAngle);

    public enum LiftPosition {
        STOW_POSITION(0.0, 0.0),
        INTAKE_POSITION(10, 45),  // Example: 10 inches height, 45 degrees angle
        HIGH_BUCKET_POSITION(15, 90),
        LOW_BUCKET_POSITION(5, 30),
        SUBMERSIBLE_SETUP_POSITION(7, 60),
        SUBMERSIBLE_SCORE_POSITION(12, 120);

        private final double heightInches;
        private final double angleDegrees;

        LiftPosition(double heightInches, double angleDegrees) {
            this.heightInches = heightInches;
            this.angleDegrees = angleDegrees;
        }

        public double getHeightInches() {
            return heightInches;
        }

        public double getAngleDegrees() {
            return angleDegrees;
        }
    }

    public LiftSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        m_Lift_Height_Motor = new MotorEx(hardwareMap, "heightMotor", Motor.GoBILDA.RPM_312);
        m_Lift_Angle_Motor = new MotorEx(hardwareMap, "angleMotor", Motor.GoBILDA.RPM_312);

        // Set motors to brake when no power is applied
        m_Lift_Height_Motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_Lift_Angle_Motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Set initial tolerances for PID
        heightPID.setTolerance(HEIGHT_PID_TOLERANCE);  // Tolerance in inches
        anglePID.setTolerance(ANGLE_PID_TOLERANCE);     // Tolerance in degrees
    }

    // Convert encoder ticks to inches
    private double ticksToInches(double ticks) {
        return (ticks / TICKS_PER_REVOLUTION) * INCHES_PER_REVOLUTION;
    }

    // Convert encoder ticks to degrees
    private double ticksToDegrees(double ticks) {
        return (ticks / TICKS_PER_REVOLUTION) * DEGREES_PER_REVOLUTION;
    }

    // Convert inches to motor ticks
    private double inchesToTicks(double inches) {
        return (inches / INCHES_PER_REVOLUTION) * TICKS_PER_REVOLUTION;
    }

    // Convert degrees to motor ticks
    private double degreesToTicks(double degrees) {
        return (degrees / DEGREES_PER_REVOLUTION) * TICKS_PER_REVOLUTION;
    }

    // Set the setpoint for both height and angle
    public void setSetpoint(LiftPosition position) {
        heightPID.setSetPoint(inchesToTicks(position.getHeightInches()));
        anglePID.setSetPoint(degreesToTicks(position.getAngleDegrees()));
    }

    // Check if both motors are at their setpoints
    public boolean atSetpoint() {
        return heightPID.atSetPoint() && anglePID.atSetPoint();
    }

    // Get the current position of the height motor in inches
    public double getHeightPositionInches() {
        return ticksToInches(m_Lift_Height_Motor.getCurrentPosition());
    }

    // Get the current position of the angle motor in degrees
    public double getAnglePositionDegrees() {
        return ticksToDegrees(m_Lift_Angle_Motor.getCurrentPosition());
    }

    // Reset the height motor encoder to zero
    public void resetHeightEncoder() {
        m_Lift_Height_Motor.resetEncoder();
    }

    // Reset the angle motor encoder to zero
    public void resetAngleEncoder() {
        m_Lift_Angle_Motor.resetEncoder();
    }

    // Set motor power directly (for manual override or tuning)
    public void setHeightPower(double power) {
        m_Lift_Height_Motor.set(power);
    }

    public void setAnglePower(double power) {
        m_Lift_Angle_Motor.set(power);
    }

    // Main control loop (runs periodically)
    @Override
    public void periodic() {
        // Calculate the output for both motors using their respective PID controllers
        double heightPower = heightPID.calculate(m_Lift_Height_Motor.getCurrentPosition());
        double anglePower = anglePID.calculate(m_Lift_Angle_Motor.getCurrentPosition());

        // Set motor powers, clamped to valid ranges
        m_Lift_Height_Motor.set(Math.max(-0.5, Math.min(0.5, heightPower)));
        m_Lift_Angle_Motor.set(Math.max(-0.5, Math.min(0.5, anglePower)));
    }
}