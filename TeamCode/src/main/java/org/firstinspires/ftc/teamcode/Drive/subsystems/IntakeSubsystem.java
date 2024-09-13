package org.firstinspires.ftc.teamcode.Drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo1;
    private final CRServo intakeServo2;

    public enum IntakeDirection {
        FORWARD(1),
        STOPPED(0),
        BACKWARD(-1);

        private final double Direction;

        IntakeDirection(double Direction) {
            this.Direction = Direction;
        }

        public double getDirection() {
            return Direction;
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
    }

    public void rotationToPosition(IntakeDirection position) {
        intakeServo1.setPower(position.getDirection());
        intakeServo1.setPower(position.getDirection());
    }
}
