package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * A command to move the lift to the high bucket position.
 */
public class LiftSubmersibleScoreCmd extends CommandBase {

    private final LiftSubsystem m_Lift;

    /**
     * Creates a new LiftHighBucketPosCmd.
     *
     * @param subsystem The lift subsystem this command will run on.
     */
    public LiftSubmersibleScoreCmd(LiftSubsystem subsystem) {
        m_Lift = subsystem;
        addRequirements(m_Lift);  // Ensures no other commands use this subsystem during execution
    }

    // Set the setpoint when the command is initialized
    @Override
    public void initialize() {

    }

    // No need to do anything continuously
    @Override
    public void execute() {
        m_Lift.setSetpoint(LiftSubsystem.LiftPosition.SUBMERSIBLE_SCORE_POSITION);
    }

    // The command is finished when the lift reaches the setpoint
    @Override
    public boolean isFinished() {
        return m_Lift.atSetpoint();  // Check if both height and angle motors are at the setpoint
    }

    // Optionally stop the lift motors when the command ends
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Optionally set motors to 0 if you want to stop them when interrupted
            m_Lift.setHeightPower(0);
            m_Lift.setAnglePower(0);
        }
    }
}