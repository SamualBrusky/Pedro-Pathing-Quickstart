package org.firstinspires.ftc.teamcode.Drive.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.LiftSubsystem;

public class IntakeCmd extends CommandBase {
    private final IntakeSubsystem m_Intake;

    public IntakeCmd(IntakeSubsystem intakeSubsystem) {
        m_Intake = intakeSubsystem;
        addRequirements(m_Intake);
    }

    @Override
    public void execute() {
        m_Intake.rotationToPosition(IntakeSubsystem.HandDirection.STOPPED);
        m_Intake.setArmPosition(IntakeSubsystem.ArmPosition.HOLD_POSITION);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
