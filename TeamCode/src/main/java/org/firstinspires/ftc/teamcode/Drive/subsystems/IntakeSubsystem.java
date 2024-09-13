package org.firstinspires.ftc.teamcode.Drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo1;
    private final CRServo intakeServo2;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
    }


}
