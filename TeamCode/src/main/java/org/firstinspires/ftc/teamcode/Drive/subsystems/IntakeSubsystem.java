package org.firstinspires.ftc.teamcode.Drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo backHand;
    private final CRServo frontHand;

    private final Servo leftArm;
    private final Servo rightArm;

    public enum HandDirection {
        FORWARD(1),
        STOPPED(0),
        BACKWARD(-1);

        private final double Direction;

        HandDirection(double Direction) {
            this.Direction = Direction;
        }

        public double getDirection() {
            return Direction;
        }
    }

    public enum ArmPosition {
        TUCK_POSITION(1.0),
        HOLD_POSITION(0.75),
        HALFWAY_POSITION(0.35),
        GRAB_POSITION(0.1);

        private final double Position;

        ArmPosition(double Position) {
            this.Position = Position;
        }

        public double getPosition() {
            return Position;
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        backHand = hardwareMap.get(CRServo.class, "backHand");
        frontHand = hardwareMap.get(CRServo.class, "frontHand");

        leftArm = hardwareMap.get(Servo.class, "armLeft");
        rightArm = hardwareMap.get(Servo.class, "armRight");
        rightArm.setDirection(Servo.Direction.REVERSE);

    }

    public void rotationToPosition(HandDirection position) {
        backHand.setPower(position.getDirection());
        frontHand.setPower(-position.getDirection());
    }

    public void setArmPosition(ArmPosition position) {
        leftArm.setPosition(position.getPosition());
        rightArm.setPosition(position.getPosition());

    }
}
