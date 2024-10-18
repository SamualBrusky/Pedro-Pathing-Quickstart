package org.firstinspires.ftc.teamcode.Drive.OpModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drive.Commands.DriveCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.IntakeCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftClimbCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftClimbDownCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftHighBucketPosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftLowBucketPosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.LiftStartPosCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.OuttakeCmd;
import org.firstinspires.ftc.teamcode.Drive.Commands.StopIntakeCmd;
import org.firstinspires.ftc.teamcode.Drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.GrabberSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Drive.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Blue TeleOp")
public class TeleOp extends CommandOpMode {
    //drive subsystem
   // private MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private IMU m_imu;
    //GamePads
    public GamepadEx m_driverOp, m_engineerOp;
    public Follower follower;

//    private LiftHighBucketPosCmd m_LiftHighBucketPosCmd;
//    private LiftStartPosCmd m_LiftStartPosCmd;
//    private LiftIntakePosCmd m_LiftintakePosCmd;
//    private GrabberOpenCmd m_GrabberOpenCmd;
//    private GrabberCloseCmd m_GrabberCloseCmd;
//    private LiftSubmersibleSetupCmd m_LiftSubmersibleSetupCmd;
//    private LiftSubmersibleScoreCmd m_LiftSubmersibleScoreCmd;
//    private LiftClimbCmd m_LiftClimbCmd;
//    private LiftClimbDownCmd m_LiftClimbDownCmd;
//    private LiftSubersibleSetupCmd m_LiftSubmersibleSetupCmd;

    public DriveSubsystem m_DriveSubsystem;
    public GrabberSubsystem m_GrabberSubsystem;
    public LiftSubsystem m_LiftSubsystem;
    public IntakeSubsystem m_IntakeSubsystem;


    public void whileWaitingToStart() {
      CommandScheduler.getInstance().run();
    }

    @Override
    public void initialize() {
        m_imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        m_LiftSubsystem = new LiftSubsystem(hardwareMap);
        m_IntakeSubsystem = new IntakeSubsystem(hardwareMap);
        m_DriveSubsystem = new DriveSubsystem(hardwareMap);
    //    m_GrabberSubsystem = new GrabberSubsystem(hardwareMap);
        follower = new Follower(hardwareMap);

        //Gamepads
        m_driverOp = new GamepadEx(gamepad1);
        m_engineerOp = new GamepadEx(gamepad2);

        m_LiftSubsystem.resetHeightEncoder();

        m_IntakeSubsystem.setArmPosition(IntakeSubsystem.ArmPosition.TUCK_POSITION);
        /*
        init commands

        m_LiftStartPosCmd = new LiftStartPosCmd(m_LiftSubsystem);
        m_LiftHighBucketPosCmd = new LiftHighBucketPosCmd(m_LiftSubsystem);
        m_LiftintakePosCmd = new LiftIntakePosCmd(m_LiftSubsystem);
        m_GrabberCloseCmd = new GrabberCloseCmd(m_GrabberSubsystem);
        m_GrabberOpenCmd = new GrabberOpenCmd(m_GrabberSubsystem);
        m_LiftSubmersibleScoreCmd = new LiftSubmersibleScoreCmd(m_LiftSubsystem);
        m_LiftSubmersibleSetupCmd = new LiftSubmersibleSetupCmd(m_LiftSubsystem);
        m_LiftClimbCmd = new LiftClimbCmd(m_LiftSubsystem);
        m_LiftClimbDownCmd = new LiftClimbDownCmd(m_LiftSubsystem);
        m_LiftSubmersibleSetupCmd = new LiftSubmersibleSetupCmd(m_LiftSubsystem);
*/

        m_engineerOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(new LiftLowBucketPosCmd(m_LiftSubsystem));

        m_engineerOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new LiftStartPosCmd(m_LiftSubsystem));

        m_engineerOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(new LiftHighBucketPosCmd(m_LiftSubsystem));

        m_engineerOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new IntakeCmd(m_IntakeSubsystem));

        m_engineerOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new OuttakeCmd(m_IntakeSubsystem));

        m_engineerOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new StopIntakeCmd(m_IntakeSubsystem));


        m_DriveSubsystem.setDefaultCommand( new DriveCmd(
                m_DriveSubsystem,
                follower,
                () -> -m_driverOp.getLeftY(),
                () -> m_driverOp.getLeftX(),
                () -> m_driverOp.getRightX()
        ));
    }
    @Override
    public void reset(){
        m_IntakeSubsystem.setArmPosition(IntakeSubsystem.ArmPosition.TUCK_POSITION);
    }
}