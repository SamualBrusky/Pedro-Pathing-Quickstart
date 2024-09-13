package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Drive.subsystems.IntakeSubsystem;

public class StopIntakeCmd extends CommandBase {
    private final IntakeSubsystem m_Intake;

    public StopIntakeCmd(IntakeSubsystem subsystem) {
        m_Intake = subsystem;
        addRequirements(m_Intake);
    }

    @Override
    public void execute() {

        m_Intake.rotationToPosition(IntakeSubsystem.IntakeDirection.FORWARD);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
