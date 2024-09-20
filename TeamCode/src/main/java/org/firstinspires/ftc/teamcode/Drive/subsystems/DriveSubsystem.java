package org.firstinspires.ftc.teamcode.Drive.subsystems;

import static com.arcrobotics.ftclib.kotlin.extensions.geometry.Vector2dExtKt.getAngle;

import org.firstinspires.ftc.robotcore.external.Func;
import  org.firstinspires.ftc.robotcore.external.Telemetry;


import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2020
 * to 2021 season.
 */
public class DriveSubsystem extends SubsystemBase {

    Telemetry telemetry = new Telemetry() {
        @Override
        public Item addData(String caption, String format, Object... args) {
            return null;
        }

        @Override
        public Item addData(String caption, Object value) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return null;
        }

        @Override
        public boolean removeItem(Item item) {
            return false;
        }

        @Override
        public void clear() {

        }

        @Override
        public void clearAll() {

        }

        @Override
        public Object addAction(Runnable action) {
            return null;
        }

        @Override
        public boolean removeAction(Object token) {
            return false;
        }

        @Override
        public void speak(String text) {

        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {

        }

        @Override
        public boolean update() {
            return false;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            return false;
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {

        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {

        }

        @Override
        public String getItemSeparator() {
            return "";
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return "";
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return null;
        }
    };
    private Follower follower;
    private final DcMotorEx m_FrontLeft;
    private final DcMotorEx m_FrontRight;
    private final DcMotorEx m_BackLeft;
    private final DcMotorEx m_BackRight;

    private final IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = new Follower(hardwareMap);
        m_FrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        m_FrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        m_BackLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        m_BackRight = hardwareMap.get(DcMotorEx.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        m_FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void PedroDrive(double x, double y, double z) {
        telemetry.addLine("Hi");
        telemetry.update();
        this.follower.setTeleOpMovementVectors(x, y, z, false);
        this.follower.update();
    }

//    public double getHeading() {
//        return imu.getHeading();
//    }

//    public void resetAngle() {
//        imu.reset();
//    }

    public void StopDrive(Follower follower){
        follower.setTeleOpMovementVectors(0,0,0, false);
    }
}
