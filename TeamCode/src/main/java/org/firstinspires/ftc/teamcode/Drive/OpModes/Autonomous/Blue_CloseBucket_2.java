//package org.firstinspires.ftc.teamcode.Drive.OpModes.Autonomous;
//
//import static java.lang.Thread.sleep;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.ArrayList;
//
//@Autonomous(name = "Blue Close Side 1 + 1", group = "Autonomous")
//public class Blue_CloseBucket_2 extends OpMode {
//
//
//    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;
//
//
//    private String navigation;
//
//// IMPORTANT: y increasing is towards the backstage from the audience,
//// while x increasing is towards the red side from the blue side
//// this means that 0 heading is pointing from the blue side to the red side
//
//    // all spike mark locations since I'm lazy
//
//    private Pose blueLeftSpikeMark = new Pose(0, 0,0);
//    private Pose blueMiddleSpikeMark = new Pose(0, 0,0);
//    private Pose blueRightSpikeMark = new Pose(0,0,0);
//    private Pose redLeftSpikeMark = new Pose(0,0,0);
//    private Pose redMiddleSpikeMark = new Pose(0,0,0);
//    private Pose redRightSpikeMark = new Pose(0,0,0);
//
//    private Pose blueNeutralLeftSpikeMark = new Pose(0,0, 0);
//    private Pose blueNeutralMiddleSpikeMark = new Pose(0,0,0);
//    private Pose blueNeutralRightSpikeMark = new Pose(0,0,0);
//    private Pose redNeutralLeftSpikeMark = new Pose(0,0,0);
//    private Pose redNeutralMiddleSpikeMark = new Pose(0,0,0);
//    private Pose redNeutralRightSpikeMark = new Pose(0,0,0);
//
//    private Pose redSubmersibleZone = new Pose(0,0, 0);
//    private Pose blueSubmersibleZone = new Pose(0,0, 0);
//
//    private Pose redBucketScorePosition = new Pose(0,0,0);
//    private Pose blueBucketScorePosition = new Pose(0,0,0);
//
//    // TODO: adjust this for each auto
//    private Pose blueFarStartPose = new Pose(-14.5+72, 61.5+72, 270);
//    private Pose blueCloseStartPose = new Pose(32.5+72,61.5+72,270);
//    private Pose redFarStartPose = new Pose(14.5+72,-61.5+72,90);
//    private Pose redCloseStartPose = new Pose(-32.5+72,-61.5+72,90);
//
//
//    // TODO: dont forget to adjust this too
//    private Point abortPoint = new Point(144-83.5, 120, Point.CARTESIAN), backdropGoalPoint;
//
//    private Follower follower;
//
//    private PathChain grabNeutralLeft, grabNeutralMiddle, grabNeutralRight, scoreNeutralLeft, scoreNeutralMiddle, scoreNeutralRight, grabBlueLeft, grabBlueMiddle, grabBlueRight, scoreBlueLeft, scoreBlueMiddle, scoreBlueRight, grabredLeft, grabredMiddle, grabredRight, scoreRedLeft, scoreRedMiddle, scoreRedRight;
//
//    private int pathState;
//    public void buildPaths() {
//
//        grabNeutralLeft = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(blueCloseStartPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(blueNeutralLeftSpikeMark.getX()+2, 79, Point.CARTESIAN)))
//                .setConstantHeadingInterpolation(blueNeutralLeftSpikeMark.getHeading())
//                .build();
//    }
//
//    public void autonomousPathUpdate() {
//            case 10: // starts following the first path to score on the spike mark
//                follower.followPath(grabNeutralLeft);
//
//                break;
//            case 11: // detects the path to progress away from the wall and sets tangent interpolation
//                if (follower.getCurrentTValue() > 0.1) {
////scoreSpikeMark.setReversed(false);
//                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                if (!follower.isBusy()) {
//                    follower.holdPoint(new BezierPoint(scoreSpikeMark.getLastControlPoint()), scoreSpikeMark.getEndTangent().getTheta());
//                    setPathState(13);
//                }
//                break;
//            case 13: // detects for the end of the path and everything else to be in order and releases the pixel
//                if (twoPersonDrive.intakeState == INTAKE_OUT) {
//                    twoPersonDrive.setIntakeClawOpen(true);
//                    setPathState(14);
//                }
//                break;
//            case 14: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
//                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
//                    twoPersonDrive.setTransferState(TRANSFER_OUT);
//                    twoPersonDrive.outtakeWristOffset = -15;
//                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
//                    setPathState(15);
//                }
//                break;
//            case 15:
//                if (pathTimer.getElapsedTime() > 500) {
//                    follower.followPath(initialScoreOnBackdrop);
//                    setPathState(16);
//                }
//                break;
//            case 16: // detects for end of the path and outtake out and drops pixel
//                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
//                    Follower.useHeading = true;
//                    backdropGoalPoint = new Point(initialBackdropGoalPose);
//                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
//                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_YELLOW_SCORE_POSITION);
//                    distanceSensorDecimationTimer.resetTimer();
//                    startDistanceSensorDisconnectDetection(-1);
//                    setPathState(17);
//                }
//                break;
//            case 17:
//                if (rearDistanceSensorDisconnected) {
//                    setPathState(18);
//                    break;
//                }
//                backdropCorrection(initialBackdropGoalPose, 3.6);
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(18);
//                }
//                break;
//            case 18:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    startDistanceSensorDisconnectDetection(1);
//                    setPathState(19);
//                }
//                break;
//            case 19: // detects for end of the path and outtake out and drops pixel
//                if (pathTimer.getElapsedTime() > 700) {
//                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
//                    setPathState(110);
//                }
//                break;
//            case 110:
//                if (pathTimer.getElapsedTime() > 500) {
//                    twoPersonDrive.setTransferState(TRANSFER_RESET);
//                    setPathState(20);
//                }
//                break;
//
//
//            case 20: // starts the robot off on to the first stack once the pixels have been dropped
//                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
//                    Follower.useHeading = true;
//                    follower.resetOffset();
//                    if (distanceSensorDisconnected) {
//                        setPathState(50);
//                        break;
//                    }
//                    follower.followPath(firstCycleToStack);
//                    setPathState(21);
//                }
//                break;
//            case 21:
//                if (!follower.isBusy()) {
////startDistanceSensorDisconnectDetection(1);
////follower.holdPoint(new BezierPoint(new Point(follower.getPose())), firstCycleStackPose.getHeading());
//                    follower.holdPoint(new BezierPoint(new Point(firstCycleToStack.getPath(1).getLastControlPoint().getX(), firstCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), firstCycleStackPose.getHeading());
//                    distanceSensorDecimationTimer.resetTimer();
//                    setPathState(22);
//                }
//                break;
//            case 22:
//                if (distanceSensorDisconnected) {
//                    setPathState(50);
//                    break;
//                }
//                stackCorrection(5.5);
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(23);
//                }
//                break;
//            case 23:
//                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_TOP_POSITION);
//                setPathState(24);
//                break;
//            case 24:
//                if (twoPersonDrive.intakeArmAtTargetPosition()) {
//                    setPathState(25);
//                }
//                break;
//            case 25:
//                if (pathTimer.getElapsedTime() > 300) {
//                    follower.followPath(firstCycleStackGrab);
//                    setPathState(26);
//                }
//                break;
//            case 26:
//                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
////Follower.useHeading = false;
////follower.holdPoint(new BezierPoint(new Point(firstCycleStackPose)), Math.PI * 1.5);
//                    twoPersonDrive.setIntakeClawOpen(false);
//                    setPathState(27);
//                }
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(27);
//                }
//                break;
//            case 27: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
//                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
//                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
//                    follower.resetOffset();
//                    Follower.useHeading = true;
//                    follower.followPath(firstCycleScoreOnBackdrop);
//                    setPathState(28);
//                }
//                break;
//            case 28:
//                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
//                    twoPersonDrive.setTransferState(TRANSFER_OUT);
//                    setPathState(29);
//                }
//                break;
//            case 29: // detects for end of the path and outtake out and drops pixel
//                if (follower.atParametricEnd() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
//                    twoPersonDrive.setLiftTargetPosition(250);
//                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_FIRST_SCORE_POSITION, 100);
//
//                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
////Follower.useHeading = false;
//                    backdropGoalPoint = new Point(firstCycleBackdropGoalPose);
//                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
//                    startDistanceSensorDisconnectDetection(-1);
//                    distanceSensorDecimationTimer.resetTimer();
//                    setPathState(210);
//                }
//                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
//                }
//                break;
//            case 210:
//                if (rearDistanceSensorDisconnected) {
//                    setPathState(211);
//                    break;
//                }
//                backdropCorrection(firstCycleBackdropGoalPose, 3.2);
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(211);
//                }
//                break;
//            case 211:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    startDistanceSensorDisconnectDetection(1);
//                    setPathState(212);
//                }
//                break;
//            case 212:
//                if (pathTimer.getElapsedTime() > 300) {
//                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
//                    setPathState(213);
//                }
//                break;
//            case 213:
//                setPathState(214);
///*
//if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
//twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION, 300);
//setPathState(214);
//}
//*/
//                break;
//            case 214:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    setPathState(215);
//                }
//                break;
//            case 215:
//                if (pathTimer.getElapsedTime() > 300) {
//                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_SECOND_SCORE_POSITION, 100);
//                    setPathState(216);
//                }
//                break;
//            case 216:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    setPathState(217);
//                }
//                break;
//            case 217: // once the outer pixel has dropped, drop the inner one and fold up
//                if (pathTimer.getElapsedTime() > 2 * OUTTAKE_CLAW_DROP_TIME) {
//                    twoPersonDrive.setTransferState(TRANSFER_RESET);
//                    setPathState(30);
//                }
//                break;
//
//
//            case 30: // once the inner pixel has dropped, start the robot off to the second pass on the first stack
//                if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
//                    Follower.useHeading = true;
//                    follower.resetOffset();
//                    if (distanceSensorDisconnected) {
//                        setPathState(50);
//                        break;
//                    }
//                    follower.followPath(secondCycleToStack);
//                    setPathState(31);
//                }
//                break;
//            case 31:
//                if (!follower.isBusy()) {
////startDistanceSensorDisconnectDetection(1);
//                    follower.holdPoint(new BezierPoint(new Point(secondCycleToStack.getPath(1).getLastControlPoint().getX(), secondCycleToStack.getPath(1).getLastControlPoint().getY() + 1, Point.CARTESIAN)), secondCycleStackPose.getHeading());
//                    distanceSensorDecimationTimer.resetTimer();
//                    setPathState(32);
//                }
//                break;
//            case 32:
//                if (distanceSensorDisconnected) {
//                    setPathState(50);
//                    break;
//                }
//                stackCorrection(5.5);
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(33);
//                }
//                break;
//            case 33:
//                twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_STACK_MIDDLE_POSITION);
//                setPathState(34);
//                break;
//            case 34:
//                if (twoPersonDrive.intakeArmAtTargetPosition()) {
//                    setPathState(35);
//                }
//                break;
//            case 35:
//                if (pathTimer.getElapsedTime() > 300) {
//                    follower.followPath(secondCycleStackGrab);
//                    setPathState(36);
//                }
//                break;
//            case 36:
//                if (follower.getCurrentTValue() > 0.92) {//!follower.isBusy()) {
////Follower.useHeading = false;
////follower.holdPoint(new BezierPoint(new Point(secondCycleStackPose)), Math.PI * 1.5);
//                    twoPersonDrive.setIntakeClawOpen(false);
//                    setPathState(37);
//                }
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(37);
//                }
//                break;
//            case 37: // waits for the intake claw to close and then sets the intake to move back in while pulling the extension back in slightly
//                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
//                    twoPersonDrive.setTransferState(TRANSFER_POSITIONING);
//                    Follower.useHeading = true;
//                    follower.resetOffset();
//                    follower.followPath(secondCycleScoreOnBackdrop);
//                    setPathState(38);
//                }
//                break;
//            case 38:
//                if (((follower.getCurrentPathNumber() == 1 && follower.getCurrentTValue() > 0.1) || !follower.isBusy()) && twoPersonDrive.transferState == TRANSFER_PRESET_HOLD) {
//                    twoPersonDrive.setTransferState(TRANSFER_OUT);
//                    setPathState(39);
//                }
//                break;
//            case 39: // detects for end of the path and outtake out and drops pixel
//                if (follower.atParametricEnd() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
//                    twoPersonDrive.setLiftTargetPosition(700);
//                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_FIRST_SCORE_POSITION, 100);
//
////twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
////Follower.useHeading = false;
//                    backdropGoalPoint = new Point(secondCycleBackdropGoalPose);
//                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
//                    distanceSensorDecimationTimer.resetTimer();
//                    startDistanceSensorDisconnectDetection(-1);
//                    setPathState(310);
//                }
//                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
//                }
//                break;
//            case 310:
//                if (rearDistanceSensorDisconnected) {
//                    setPathState(311);
//                    break;
//                }
//                backdropCorrection(secondCycleBackdropGoalPose, 3.2);
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(311);
//                }
//                break;
//            case 311:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    startDistanceSensorDisconnectDetection(1);
//                    setPathState(312);
//                }
//                break;
//            case 312:
//                if (pathTimer.getElapsedTime() > 300) {
//                    twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_OPEN);
//                    setPathState(313);
//                }
//                break;
//            case 313:
//                setPathState(314);
///*
//if (pathTimer.getElapsedTime() > OUTTAKE_CLAW_DROP_TIME) {
//twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_OUT_POSITION, 300);
//setPathState(314);
//}
//*/
//                break;
//            case 314:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    setPathState(315);
//                }
//                break;
//            case 315:
//                if (pathTimer.getElapsedTime() > 300) {
//                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_CYCLE_SECOND_SCORE_POSITION, 100);
//                    setPathState(316);
//                }
//                break;
//            case 316:
//                if (twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    setPathState(317);
//                }
//                break;
//            case 317: // once the outer pixel has dropped, drop the inner one and fold up
//                if (pathTimer.getElapsedTime() > 2 * OUTTAKE_CLAW_DROP_TIME) {
////twoPersonDrive.setTransferState(TRANSFER_RESET);
//                    Follower.useHeading = true;
//                    setPathState(40);
//                }
//                break;
//
//
//            case 40: // move the intake in
//                twoPersonDrive.setTransferState(TRANSFER_RESET);
//                twoPersonDrive.moveIntake(INTAKE_IN);
//                setPathState(41);
//                break;
//            case 41: // once the robot is nice and folded up, request stop
//                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.intakeArmAtTargetPosition() && twoPersonDrive.outtakeState == OUTTAKE_IN && twoPersonDrive.outtakeArmAtTargetPosition()) {
//                    setPathState(-1);
//                }
//                break;
//
//            case 50:
//                twoPersonDrive.setTransferState(TRANSFER_RESET);
//                twoPersonDrive.moveIntake(INTAKE_IN);
//                setPathState(51);
//                break;
//            case 51:
//                if (twoPersonDrive.intakeState == INTAKE_IN && twoPersonDrive.intakeArmAtTargetPosition() && twoPersonDrive.outtakeState == OUTTAKE_IN && twoPersonDrive.outtakeArmAtTargetPosition() && twoPersonDrive.liftEncoder.getCurrentPosition() < LIFT_TRANSFER_UPPER_LIMIT) {
//                    follower.resetOffset();
//                    PathChain abort = follower.pathBuilder()
//                            .addPath(new BezierLine(new Point(follower.getPose()), abortPoint))
//                            .setConstantHeadingInterpolation(Math.PI * 1.5)
//                            .build();
//                    follower.followPath(abort);
//                    setPathState(52);
//                }
//                break;
//            case 52:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//
//            default:
//                requestOpModeStop();
//                break;
//        }
//
//        if (opmodeTimer.getElapsedTimeSeconds() > 28) {
//            foldUp.run();
//        }
//    }
//
//    public void setPathState(int state) {
//        pathState = state;
//        pathTimer.resetTimer();
//        autonomousPathUpdate();
//    }
//
//    public void stackCorrection(double correctionPower) {
//        if (distanceSensorDecimationTimer.getElapsedTime() > 20) {
//
//            double left = leftDistanceSensor.getDistance(DistanceUnit.MM);
//
//            if (!(left == 65535)) {
//
//                double right = rightDistanceSensor.getDistance(DistanceUnit.MM);
//
//                if (!(right == 65535)) {
//
//                    double error = (left / 25.4) - (right / 25.4);
//
//                    error *= -1;
//
//                    if (Math.abs(error) > 0.5) {
//                        follower.setXOffset(follower.getXOffset() + distanceSensorDecimationTimer.getElapsedTimeSeconds() * correctionPower * MathFunctions.getSign(error));
//                    } else {
//                        follower.setXOffset(follower.getXOffset() + follower.getTranslationalError().getXComponent());
//                    }
//
//                    follower.setXOffset(MathFunctions.clamp(follower.getXOffset(), -6, 6));
//
////telemetry.addData("error", error);
//                    distanceSensorDecimationTimer.resetTimer();
//                } else {
//                    distanceSensorDisconnected = true;
//                }
//            } else {
//                distanceSensorDisconnected = true;
//            }
//        }
//    }
//
//    public boolean leftDistanceSensorDisconnected() {
//        return leftDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
//    }
//
//    public boolean rightDistanceSensorDisconnected() {
//        return rightDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
//    }
//
//    public void startDistanceSensorDisconnectDetection(int state) {
//        detectDistanceSensorDisconnect = 0;//state;
//        distanceSensorDisconnectCycleCount = 0;
//        distanceSensorDisconnects.clear();
//    }
//
//    public void updateDistanceSensorDisconnects() {
//        if (detectDistanceSensorDisconnect == 1) {
//            if (distanceSensorDisconnectCycleCount < 10) {
//                if (distanceSensorUpdateTimer.getElapsedTime() > 20) {
//                    distanceSensorDisconnectCycleCount++;
//                    distanceSensorUpdateTimer.resetTimer();
//
//                    distanceSensorDisconnects.add(leftDistanceSensorDisconnected() || rightDistanceSensorDisconnected());
//                }
//            } else {
//                detectDistanceSensorDisconnect = 0;
//
//                distanceSensorDisconnected = true;
//                for (Boolean detection : distanceSensorDisconnects) {
//                    if (!detection) {
//                        distanceSensorDisconnected = false;
//                    }
//                }
//            }
//        } else if (detectDistanceSensorDisconnect == -1) {
//            if (distanceSensorDisconnectCycleCount < 10) {
//                if (distanceSensorUpdateTimer.getElapsedTime() > 20) {
//                    distanceSensorDisconnectCycleCount++;
//                    distanceSensorUpdateTimer.resetTimer();
//
//                    distanceSensorDisconnects.add(rearDistanceSensorDisconnected());
//                }
//            } else {
//                detectDistanceSensorDisconnect = 0;
//
//                rearDistanceSensorDisconnected = true;
//                for (Boolean detection : distanceSensorDisconnects) {
//                    if (!detection) {
//                        rearDistanceSensorDisconnected = false;
//                    }
//                }
//            }
//        }
//    }
//
//    public void backdropCorrection(Pose scorePose, double distanceGoal) {
//        if (distanceSensorDecimationTimer.getElapsedTime() > 20) {
//
//            double distance = rearDistanceSensor.getDistance(DistanceUnit.MM);
//
//            if (distance != 65535) {
////follower.holdPoint(new BezierPoint(new Point(scorePose.getX(), MathFunctions.clamp(follower.getPose().getY() + (distance / 25.4) - distanceGoal, scorePose.getY() - 4, scorePose.getY() + 4), Point.CARTESIAN)), Math.PI * 1.5);
//                backdropGoalPoint.setCoordinates(scorePose.getX(), MathFunctions.clamp(follower.getPose().getY() + ((distance / 25.4) - distanceGoal), scorePose.getY() - 4, scorePose.getY() + 4), Point.CARTESIAN);
//            } else {
//                rearDistanceSensorDisconnected = true;
//            }
///*
//// too close
//if (distance < 0.5)
//follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() + distanceSensorDecimationTimer.getElapsedTimeSeconds() * 1.5);
//
//// too far
//if (distance > 0.75)
//follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - distanceSensorDecimationTimer.getElapsedTimeSeconds() * 1.5);
//
//// to do add some sort of deadzone or dampening
//// perhaps take note of the estimated pose at the start and see how far off we need to go instead of incrementing off of the current one
//// or just remove the getyoffset thing? think about later
//follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - (distance - 2));
//
//if (Math.abs(follower.poseUpdater.getYOffset()) > 1.5)
//follower.poseUpdater.setYOffset(1.5 * MathFunctions.getSign(follower.poseUpdater.getYOffset()));
//*/
////telemetry.addData("rear distance value", distance);
//            distanceSensorDecimationTimer.resetTimer();
//        }
//    }
//
//    public boolean rearDistanceSensorDisconnected() {
//        return rearDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
//    }
//
//    @Override
//    public void loop() {
//        updateDistanceSensorDisconnects();
//        follower.update();
//        twoPersonDrive.autonomousControlUpdate();
//
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        twoPersonDrive.telemetry();
////telemetry.update();
//    }
//
//    @Override
//    public void init() {
////PhotonCore.start(this.hardwareMap);
//
//        foldUp = new SingleRunAction(()-> {
//            if (Integer.parseInt(String.valueOf(pathState).substring(0,1)) < 4) setPathState(40);
//        });
//
//        distanceSensorDisconnects = new ArrayList<>();
//
//        rearDistanceSensor = hardwareMap.get(DistanceSensor.class, "rearDistanceSensor");
//        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
//        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
//
//        twoPersonDrive = new TwoPersonDrive(true);
//        twoPersonDrive.hardwareMap = hardwareMap;
//        twoPersonDrive.telemetry = telemetry;
//
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        scanTimer = new Timer();
//        distanceSensorUpdateTimer = new Timer();
//        distanceSensorDecimationTimer = new Timer();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        teamPropPipeline = new VisionPortalTeamPropPipeline(2);
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
//                .addProcessors(teamPropPipeline)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
//
//        twoPersonDrive.initialize();
//        twoPersonDrive.setIntakeArmPosition(INTAKE_ARM_IN_POSITION+0.1);
//
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        if (leftDistanceSensorDisconnected()) {
//            try {
//                throw new Exception("left distance sensor disconnected");
//            } catch (Exception e) {
//                throw new RuntimeException(e);
//            }
//        }
//
//        if (rightDistanceSensorDisconnected()) {
//            try {
//                throw new Exception("right distance sensor disconnected");
//            } catch (Exception e) {
//                throw new RuntimeException(e);
//            }
//        }
//
//        if (rearDistanceSensorDisconnected()) {
//            try {
//                throw new Exception("color sensor disconnected");
//            } catch (Exception e) {
//                throw new RuntimeException(e);
//            }
//        }
//
//        twoPersonDrive.outerOuttakeClaw.setPosition(OUTER_OUTTAKE_CLAW_CLOSED);
//
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        twoPersonDrive.intakeClaw.setPosition(INTAKE_CLAW_CLOSED);
//        twoPersonDrive.intakeClawIsOpen=false;
//
//        try {
//            sleep(2500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        scanTimer.resetTimer();
//    }
//
//    @Override
//    public void init_loop() {
//        if (scanTimer.getElapsedTime() > 750) {
//            navigation = teamPropPipeline.getNavigation();
//            telemetry.addData("Navigation:", navigation);
//            telemetry.update();
//            scanTimer.resetTimer();
//        } else if (scanTimer.getElapsedTime() > 700){
//            visionPortal.setProcessorEnabled(teamPropPipeline, true);
//        } else {
//            visionPortal.setProcessorEnabled(teamPropPipeline, false);
//        }
//    }
//
//    @Override
//    public void start() {
//        visionPortal.stopStreaming();
//        twoPersonDrive.frameTimer.resetTimer();
//        setBackdropGoalPose();
//        buildPaths();
//        opmodeTimer.resetTimer();
//        setPathState(10);
//    }
//
//    @Override
//    public void stop() {
//    }
//}
