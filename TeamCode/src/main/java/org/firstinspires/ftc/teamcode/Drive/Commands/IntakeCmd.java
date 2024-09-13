package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Drive.subsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {
    private final IntakeSubsystem m_Intake;

    public IntakeCmd(IntakeSubsystem subsystem) {
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
