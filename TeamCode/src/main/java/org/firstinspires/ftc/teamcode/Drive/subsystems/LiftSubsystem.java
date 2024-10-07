package org.firstinspires.ftc.teamcode.Drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {
    private final MotorEx m_Lift_Motor_Left;
    private final MotorEx m_Lift_Motor_Right;

    private final double HEIGHT_PID_TOLERANCE = 0.5; //inch

    private final double kP = 0.01;
    private final double kI = 0;
    private final double kD = 0.00;
    private final double kF = 0;

    private final PIDFController heightPID = new PIDFController(kP, kI, kD, kF);

    private MotorGroup liftGroup;

    public enum LiftPosition {
        STOW_POSITION(0.0),
        INTAKE_POSITION(10),  // Example: 10 inches height, 45 degrees angle
        HIGH_BUCKET_POSITION(400),
        LOW_BUCKET_POSITION(1000),
        SUBMERSIBLE_SETUP_POSITION(300),
        SUBMERSIBLE_SCORE_POSITION(225);
        Climb_Up(500);

        Climb_Down(200);
        private final double height;

        LiftPosition(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }

    }

    public LiftSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        m_Lift_Motor_Left = new MotorEx(hardwareMap, "liftMotorLeft", Motor.GoBILDA.RPM_435);
        m_Lift_Motor_Right = new MotorEx(hardwareMap, "liftMotorRight", Motor.GoBILDA.RPM_435);

        m_Lift_Motor_Right.setInverted(true);

        liftGroup = new MotorGroup(m_Lift_Motor_Left, m_Lift_Motor_Right);

        liftGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Set initial tolerances for PID
        heightPID.setTolerance(HEIGHT_PID_TOLERANCE);  // Tolerance in inches
    }

    // Set the setpoint for both height and angle
    public void setSetpoint(LiftPosition position) {
        heightPID.setSetPoint(position.getHeight());
    }

    // Check if both motors are at their setpoints
    public boolean atSetpoint() {
        return heightPID.atSetPoint();
    }

    // Get the current position of the height motor in inches
    public double getHeightPosition() {
        return liftGroup.getCurrentPosition();
    }

    // Reset the height motor encoder to zero
    public void resetHeightEncoder() {
        liftGroup.resetEncoder();
    }

    // Set motor power directly (for manual override or tuning)
    public void setHeightPower(double power) {
        liftGroup.set(power);
    }

    // Main control loop (runs periodically)
    @Override
    public void periodic() {
        // Calculate the output for both motors using their respective PID controllers
        double heightPower = heightPID.calculate(m_Lift_Motor_Left.getCurrentPosition());

        // Set motor powers, clamped to valid ranges
        liftGroup.set(Math.max(-0.75, Math.min(0.75, heightPower)));
    }
}