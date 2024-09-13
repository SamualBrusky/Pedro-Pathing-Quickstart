package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Drive.subsystems.IntakeSubsystem;

public class OuttakeCmd extends CommandBase {
    private final IntakeSubsystem m_Intake;

    public OuttakeCmd(IntakeSubsystem subsystem) {
        m_Intake = subsystem;
        addRequirements(m_Intake);
    }

    @Override
    public void execute() {

        m_Intake.rotationToPosition(IntakeSubsystem.IntakeDirection.BACKWARD);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
