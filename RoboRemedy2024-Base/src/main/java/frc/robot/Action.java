// Action.java - Action that can be used for autonomous behaviour
package frc.robot;

import edu.wpi.first.wpilibj.*;

public class Action {
    private Electronics e;
    private RoboLog rLog;
    private RobotState robotState;
    private OI oi;
    private static LinearMapper driveToTargetMapper = new LinearMapper();

    private double[] driveEncoderZeros;
    private Timer actionTimer = new Timer();
    private Timer logTimer = new Timer();
    int actionCounter = 0;
    private double calcDistTraveled = 0;
    private boolean lastLeftCameraAttached = false;
    private boolean lastRightCameraAttached = false;
    private boolean lastCubeCameraAttached = false;
    private boolean lastDriverStationAttached = false;
    private boolean lastAprilTagFound = false;
    private boolean lastTooFar = false;
    private boolean lastTooClose = false;
    private boolean lastTooRight = false;
    private boolean lastTooLeft = false;
    private boolean isPrinted = false;
    private int lastAprilTagID = 0;
    private double lastInchesRightOfTarget = 0;
    private double lastInchesInFrontOfTarget = 0;
    private int step = 1;
    private static final double TARGET_RANGE = 0;
    private boolean driveToMode = false;

    Action(Electronics elec, RoboLog rLog, RobotState robotState, OI oi) {
        this.rLog = rLog;
        this.e = elec;
        this.robotState = robotState;
        this.oi = oi;
        if (this.rLog == null) {
            System.out.println("Warning: Action.rLog is null");
        }
        if (this.e == null) {
            rLog.print("Warning: Action.e is null");
        }
        if (this.robotState == null) {
            rLog.print("Warning: Action.robotState is null");
        }
        if (this.oi == null) {
            rLog.print("Warning: Action.oi is null");
        }
        driveToTargetMapper.add(20, 4);
        driveToTargetMapper.add(60, 2);
        actionTimer.start();
        logTimer.start();
    }

    /**
     * Resets all variables used in action, including step number.
     */
    public void actionReset() {
        step = 1;
        subActionReset();
    }

    /**
     * Resets all variables used in action. Normally used in actionReset.
     */
    public void subActionReset() {
        driveEncoderZeros = e.getAllDriveEncoders();
        calcDistTraveled = 0;
        actionTimer.reset();
        actionTimer.start();
        actionCounter = 0;
        lastTooFar = false;
        lastTooClose = false;
        lastTooRight = false;
        lastTooLeft = false;
        isPrinted = false;
        driveToMode = false;
    }

    /**
     * Resets all variables used in action and sets step number to specified number.
     * 
     * @param number What to set the step number to
     */
    public void setAutoStepNumber(int number) {
        step = number;
        subActionReset();
        rLog.print("New auto action step " + step);
    }

    /**
     * Resets all variables used in action and sets step number to the next step.
     */
    void setNextAutoStepNumber() {
        setAutoStepNumber(step + 1);
    }

    /**
     * Gets the action timer's value and returns it as a double.
     * 
     * @return The current time on actionTimer
     */
    public double getActTime() {
        return actionTimer.get();
    }

    /**
     * Drive straight a certain distance without attempting to keep the robot facing
     * a specific direction.
     * The distance traveled will be more accurate than when using
     * driveStraightWithFacing
     * 
     * @param travelAngle           Which way the bot will move
     * @param travelInchesPerSecond How fast the robot will move
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     */
    public boolean driveStraightNoFacing(double travelAngle, double travelInchesPerSecond, double maxTravelAcceleration,
            double distance, double endingInchesPerSecond) {
        double distTraveled = Math.abs((e.getAllDriveEncoders()[0] - driveEncoderZeros[0]));
        double distYetToGo = distance - distTraveled;
        if (distTraveled > distance - TARGET_RANGE) {
            e.assignRobotMotionField(travelAngle, endingInchesPerSecond, 0);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double upInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            double dwnInPerSec = endingInchesPerSecond + (distYetToGo * 2);
            double calcInPerSec = Math.min(upInPerSec, dwnInPerSec);
            e.assignRobotMotionField(travelAngle, calcInPerSec, 0);
        }
        return false;
    }

    /**
     * Drive straight a certain distance while attempting to keep the robot facing a
     * specific direction.
     * The distance traveled will be less accurate than when using
     * driveStraightNoFacing.
     * 
     * @param travelAngle           Which way the bot will move
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           Which way the robot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     */
    public boolean driveStraightWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration, double distance, double endingInchesPerSecond) {
        calcDistTraveled += e.getCalculatedTravelSinceLastCommand();
        double distYetToGo = distance - calcDistTraveled;
        if (calcDistTraveled > distance - TARGET_RANGE) {
            e.assignRobotMotionAndHeadingField(travelAngle, endingInchesPerSecond, facingAngle);
            return true;
        } else {
            if (travelInchesPerSecond < 0) {
                travelInchesPerSecond = -1 * travelInchesPerSecond;
                travelAngle = travelAngle + 180;
            }
            double upInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            double dwnInPerSec = endingInchesPerSecond + (distYetToGo * 2);
            double calcInPerSec = Math.min(upInPerSec, dwnInPerSec);
            e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
        }
        return false;
    }

    /**
     * This method will drive to a point specified by the inches forward and inches
     * right.
     * 
     * @param inchesForward         How far in front of the robot the target point
     *                              is
     * @param inchesRight           How far to the right of the robot the target
     *                              point is
     * @param travelInchesPerSecond How fast the robot will drive
     * @param facingAngle           Which way the robot will face
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond What speed the robot will slow down to before
     *                              moving on
     */
    public boolean driveStraightToPointWithFacing(double inchesForward, double inchesRight,
            double travelInchesPerSecond, double facingAngle, double maxTravelAcceleration,
            double endingInchesPerSecond) {
        double travelAngle = Math.toDegrees(Math.atan2(inchesRight, inchesForward));
        double distance = RobotMath.pythagoreanTheorem(inchesForward, inchesRight);
        if (!isPrinted) {
            rLog.print("calc travelAngle; " + travelAngle + ", calc distance; " + distance);
            isPrinted = true;
        }
        return driveStraightWithFacing(travelAngle, travelInchesPerSecond, facingAngle, maxTravelAcceleration,
                distance, endingInchesPerSecond);
    }

    /**
     * Drive a curve with a specific turn radius at a constant speed the whole way
     * while maintaining the current facing angle, whatever that happens to be.
     * Picture the
     * robot as driving part of an imaginary circle.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     * @param turnRadius            The radius of the imaginary circle that the
     *                              robot drives on.
     */
    public boolean driveCurveRadiusNoFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double maxTravelAcceleration, double turnRadius) {
        return driveCurveRadiusWithFacing(startingAngle, targetAngle, travelInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, turnRadius);
    }

    /**
     * Drive a curve with a specific turn radius at a constant speed the whole way
     * while attempting to keep the robot facing a specific direction. Picture the
     * robot
     * as driving part of an imaginary circle.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           What direction the robot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param distance              How far (in inches) the robot will move
     * @param endingInchesPerSecond What the robot will slow down to before
     *                              considering this action done
     * @param turnRadius            The radius of the imaginary circle that the
     *                              robot drives on.
     */
    public boolean driveCurveRadiusWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double turnRadius) {
        double degreesPerSecond = (travelInchesPerSecond * 180) / (Math.PI * turnRadius);
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, facingAngle,
                maxTravelAcceleration, degreesPerSecond);
    }

    /**
     * Normally use driveCurveRadiusNoFacing instead.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param degreesPerSecond      How many degrees of the circle the robot would
     *                              cover per second at its current speed
     */
    public boolean driveCurveNoFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double maxTravelAcceleration, double degreesPerSecond) {
        return driveCurveWithFacing(startingAngle, targetAngle, travelInchesPerSecond, e.getGyro(),
                maxTravelAcceleration, degreesPerSecond);
    }

    /**
     * Normally use driveCurveRadiusWithFacing instead.
     * 
     * @param startingAngle         What angle the robot starts on in the circle
     * @param targetAngle           What angle the robot wants to be on in the
     *                              circle
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           Which way the robot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     * @param degreesPerSecond      How many degrees of the circle the robot would
     *                              cover at its current speed
     */
    public boolean driveCurveWithFacing(double startingAngle, double targetAngle, double travelInchesPerSecond,
            double facingAngle, double maxTravelAcceleration, double degreesPerSecond) {
        boolean goalReached = false;
        if (startingAngle > targetAngle && degreesPerSecond > 0) {
            degreesPerSecond = -degreesPerSecond;
        }
        double goalHeading = startingAngle + (getActTime() * degreesPerSecond);
        if (degreesPerSecond > 0) {
            goalReached = targetAngle - goalHeading < 0.5;
        } else {
            goalReached = targetAngle - goalHeading > -0.5;
        }
        if (goalReached) {
            e.assignRobotMotionAndHeadingField(targetAngle, travelInchesPerSecond, facingAngle);
            return true;
        } else {
            double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
            e.assignRobotMotionAndHeadingField(goalHeading, calcInPerSec, facingAngle);
        }
        return false;
    }

    /**
     * Drive straight at a specific speed with no specific ending point
     * 
     * @param travelAngle           Which way the bot will move
     * @param travelInchesPerSecond How fast the robot will move
     * @param facingAngle           Which way the bot faces while driving
     * @param maxTravelAcceleration How fast the robot is allowed to accelerate
     */
    public void driveStraightContinuousWithFacing(double travelAngle, double travelInchesPerSecond, double facingAngle,
            double maxTravelAcceleration) {
        double calcInPerSec = limitAccel(travelInchesPerSecond, maxTravelAcceleration);
        e.assignRobotMotionAndHeadingField(travelAngle, calcInPerSec, facingAngle);
    }

    /**
     * Drive straight to a specific location using the Limelight data.
     * 
     * @param facingAngle              Which way the bot will face while driving
     * @param targetDistanceInInches   How far in front of the target the bot wats
     *                                 to be
     * @param targetInchesRightOfRobot How far to the left the robot wants to be
     *                                 from the target
     * @param distanceRange            How precise the robot will try to be going
     *                                 forward/back
     * @param horzDistRange            How precise the robot will try to be going
     *                                 right/left
     * @param maxTravelSpeed           How fast the robot will move
     * @param maxTravelAcceleration    How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond    What the robot will slow down to before
     *                                 considering this action done
     * @param visionPipeline           Where to take vision data from
     */
    public boolean driveToLimelightTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfRobot, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, double endingInchesPerSecond, Limelight.VisionPipeline visionPipeline) {
        Limelight.setPipeline(visionPipeline);
        double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        double forwardDistToTarget = Limelight.getAdjustedDistanceInInches();
        double inchesRightOfRobot = Limelight.getInchesRightOfRobot(gyroAngle, facingAngle);
        boolean isTargetFound = Limelight.isTargetFound();
        return driveToVisionTarget(facingAngle, targetDistanceInInches, targetInchesRightOfRobot, distanceRange,
                horzDistRange, maxTravelSpeed, maxTravelAcceleration, endingInchesPerSecond, isTargetFound,
                forwardDistToTarget, inchesRightOfRobot);
    }

    /**
     * Drive straight to a specific location using the PhotonVision data
     * 
     * @param facingAngle              Which way the bot will face while driving
     * @param targetDistanceInInches   How far in front of the target the bot wats
     *                                 to be
     * @param targetInchesRightOfRobot How far to the left the robot wants to be
     *                                 from the target
     * @param distanceRange            How precise the robot will try to be going
     *                                 forward/back
     * @param horzDistRange            How precise the robot will try to be going
     *                                 right/left
     * @param maxTravelSpeed           How fast the robot will move
     * @param maxTravelAcceleration    How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond    What the robot will slow down to before
     *                                 considering this action done
     */
    public boolean driveToPhotonVisionTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfRobot, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, double endingInchesPerSecond) {
        double forwardDistToTarget = robotState.aprilTagInchesInFrontOfTarget;
        double inchesRightOfRobot = -robotState.aprilTagInchesRightOfTarget;
        boolean isAprilTagFound = robotState.isAprilTagFound;
        return driveToVisionTarget(facingAngle, targetDistanceInInches, targetInchesRightOfRobot, distanceRange,
                horzDistRange, maxTravelSpeed, maxTravelAcceleration, endingInchesPerSecond, isAprilTagFound,
                forwardDistToTarget, inchesRightOfRobot);
    }

    /**
     * Drive to a specific location using any vision data. Usually used inside of
     * driveToLimelightTarget or driveToPhotonVisionTarget.
     * 
     * @param facingAngle              Which way the bot will face while driving
     * @param targetDistanceInInches   How far in front of the target the bot wats
     *                                 to be
     * @param targetInchesRightOfRobot How far to the left the robot wants to be
     *                                 from the target
     * @param distanceRange            How precise the robot will try to be going
     *                                 forward/back
     * @param horzDistRange            How precise the robot will try to be going
     *                                 right/left
     * @param maxTravelSpeed           How fast the robot will move
     * @param maxTravelAcceleration    How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond    What the robot will slow down to before
     *                                 considering this action done
     * @param isTargetFound            Whether or not target is found, usually
     *                                 assigned by other methods
     * @param forwardDistToTarget      Current distance in fronnt of target, usually
     *                                 assigned by other methods
     * @param inchesRightOfRobot       Current distance to the left of target,
     *                                 usually assignned by other methods
     */
    public boolean driveToVisionTarget(double facingAngle, double targetDistanceInInches,
            double targetInchesRightOfRobot, double distanceRange, double horzDistRange, double maxTravelSpeed,
            double maxTravelAcceleration, double endingInchesPerSecond, boolean isTargetFound,
            double forwardDistToTarget,
            double inchesRightOfRobot) {
        double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        if (isTargetFound && forwardDistToTarget > 0) {
            double horzDistToTarget = inchesRightOfRobot
                    - targetInchesRightOfRobot;
            double forwardDistToGoal = forwardDistToTarget - targetDistanceInInches;
            double calcDistToGo = Math.sqrt((Math.pow(horzDistToTarget, 2)) + Math.pow(forwardDistToGoal, 2));
            double calcHeading = gyroAngle + (90 - Math.toDegrees(Math.atan2(forwardDistToGoal, horzDistToTarget)));
            if (Math.abs(horzDistToTarget) < horzDistRange && Math.abs(forwardDistToGoal) < distanceRange
                    && Math.abs(facingAngle - e.getGyroCenteredOnGoal(facingAngle)) < 2) {
                e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                return true;
            } else {
                double dwnInPerSec = endingInchesPerSecond + (calcDistToGo * 1.6);
                double calcInPerSec = Math.min(maxTravelSpeed, dwnInPerSec);
                double finalVel = limitAccel(calcInPerSec, maxTravelAcceleration);
                e.assignRobotMotionAndHeadingField(calcHeading, finalVel, facingAngle);
            }
        } else {
            e.assignRobotMotionAndHeadingField(0, 0, facingAngle);
        }
        return false;
    }

    /**
     * Drives to a specific spot in front of an AprilTag.
     * 
     * @param facingAngle               Which way the robot will face while driving
     * @param cameraAngle               Which way the camera's expected to face
     * @param targetDistance            The distance the robot will want to be in
     *                                  front of the target
     * @param targetInchesRightOfTarget How far to the right the robot will want to
     *                                  be from the target
     * @param travelSpd                 How fast the robot will drive
     * @param maxTravelAcceleration     How fast the robot is allowed to accelerate
     * @param endingInchesPerSecond     The speed the robot will slow down to before
     *                                  moving on
     * @param distanceRange             How precise the robot will be going
     *                                  forward/back
     * @param horzDistRange             How precise the robot will be going
     *                                  right/left
     */
    public boolean driveAlignedToAprilTagTarget(double facingAngle, double cameraAngle, double targetDistance,
            double targetInchesRightOfTarget,
            double travelSpd, double maxTravelAcceleration, double endingInchesPerSecond,
            double distanceRange, double horzDistRange) {
        double currentDistToTarget = robotState.aprilTagInchesInFrontOfTarget;
        double currentInchesRightOfTarget = robotState.aprilTagInchesRightOfTarget;
        boolean isAprilTagFound = robotState.isAprilTagFound;
        return driveAlignedToTarget(facingAngle, cameraAngle, targetDistance, currentDistToTarget,
                targetInchesRightOfTarget, currentInchesRightOfTarget,
                travelSpd, maxTravelAcceleration, endingInchesPerSecond, distanceRange, horzDistRange, isAprilTagFound);
    }

    
    public boolean driveStraightToAprilTagDistance(double facingAngle, double cameraAngle, 
            double targetDistance, double distanceRange, double maxAcceleration, double travelInchesPerSecond) {
        double currentDistance = robotState.aprilTagInchesInFrontOfTarget;
        boolean isTargetFound = robotState.isAprilTagFound;
        return driveStraightToTargetDistance(facingAngle, cameraAngle, targetDistance, currentDistance,
                distanceRange, maxAcceleration, travelInchesPerSecond, isTargetFound);
    }

    public boolean driveStraightToTargetDistance(double facingAngle, double cameraAngle, double targetDistance,
            double currentDistance, double distanceRange, double maxAcceleration, double travelInchesPerSecond,
            boolean isTargetFound) {
        double distanceToGo = currentDistance - targetDistance;
        double travelSpd = RobotMath.minMax(-travelInchesPerSecond, travelInchesPerSecond, distanceToGo * 1.6);
        if (isTargetFound) {
            if (Math.abs(distanceToGo) < distanceRange) {
                if (actionCounter % 5 == 0) {
                    rLog.print("DIST_GOOD: " + RobotMath.round2(currentDistance) + " / " +
                            RobotMath.round2(distanceToGo));
                }
                actionCounter++;        
                e.stopSwerveMotors();
                return true;
            } else {
                if (actionCounter % 5 == 0) {
                    rLog.print("DIST_BAD: " + RobotMath.round2(currentDistance) + " / " +
                            RobotMath.round2(distanceToGo) + " / " +
                            RobotMath.round2(travelSpd));
                }
                actionCounter++;        
                // e.assignRobotMotionAndHeadingFieldAccel(cameraAngle, travelSpd, facingAngle, maxAcceleration); 
                e.stopSwerveMotors();
            }
        } else {
            e.stopSwerveMotors();
        }
        return false;
    }

    /**
     * Drives in an arc to get to a set distance and offset from target. This method
     * is used inside of driveToCube
     * and driveAlignedToAprilTagTarget.
     * 
     * @param facingAngle                Which way the robot will face while driving
     * @param cameraAngle                Which way the camera will be expected to
     *                                   face
     * @param targetDistance             How far in front of the camera the robot
     *                                   will want to be
     * @param currentDistance            The current distance in front of the
     *                                   target, usually provided by another method
     * @param targetInchesRightOfTarget  How far to the right of the target the
     *                                   robot will want to be
     * @param currentInchesRightOfTarget The curret distance to the right of the
     *                                   target, usually provided by another method
     * @param travelSpd                  How fast (in inches per second) the robot
     *                                   will move
     * @param maxTravelAcceleration      How fast the robot is able to accelerate
     * @param endingInchesPerSecond      The speed the robot will slow down to
     *                                   before moving on
     * @param distanceRange              How precise the robot will be moving
     *                                   forward/back
     * @param horzDistRange              How precise the robot will be moving
     *                                   right/left
     * @param isTargetFound              Whether or ot the target is found, usually
     *                                   probided by aother method
     */
    public boolean driveAlignedToTarget(double facingAngle, double cameraAngle, double targetDistance,
            double currentDistance,
            double targetInchesRightOfTarget, double currentInchesRightOfTarget,
            double travelSpd, double maxTravelAcceleration, double endingInchesPerSecond,
            double distanceRange, double horzDistRange, boolean isTargetFound) {
        double calcHeading;
        double directionFacing = cameraAngle;

        if (!isPrinted) {
            rLog.print("Start driveAlignedToTarget to Distance=" + targetDistance + "+/-" + distanceRange +
                    " Right=" + targetInchesRightOfTarget + "+/-" + horzDistRange);
            isPrinted = true;
        }

        // double gyroAngle = e.getGyroCenteredOnGoal(facingAngle);
        if (isTargetFound) {
            double horzDistToTarget = -(currentInchesRightOfTarget - targetInchesRightOfTarget);
            double distToTargetDist = Math.abs(currentDistance - targetDistance);
            double dwnInPerSec = endingInchesPerSecond + (Math.max(distToTargetDist, Math.abs(horzDistToTarget)) * 1.6);
            double calcInPerSec = Math.min(travelSpd, dwnInPerSec);
            double finalVel = limitAccel(calcInPerSec, maxTravelAcceleration);

            double adjustmentAngle = Math.min(Math.max(horzDistToTarget * driveToTargetMapper.calculate(finalVel), -90),
                    90);

            boolean isTooFar = (currentDistance > targetDistance + distanceRange);
            boolean isTooClose = (currentDistance < targetDistance - distanceRange);
            boolean isTooRight = (currentInchesRightOfTarget > targetInchesRightOfTarget + horzDistRange);
            boolean isTooLeft = (currentInchesRightOfTarget < targetInchesRightOfTarget - horzDistRange);

            if (currentDistance <= targetDistance + distanceRange
                    && currentDistance >= targetDistance - distanceRange
                    && Math.abs(horzDistToTarget) > horzDistRange) {
                if (driveToMode == false) {
                    rLog.print("Enter DriveToMode");
                }
                driveToMode = true;
                if (currentInchesRightOfTarget >= targetInchesRightOfTarget) {
                    calcHeading = 270 + directionFacing;
                } else {
                    calcHeading = 90 + directionFacing;
                }
            } else {
                if (currentDistance > targetDistance) {
                    calcHeading = adjustmentAngle + directionFacing;
                } else {
                    calcHeading = 180 - adjustmentAngle + directionFacing;
                }
            }
            if (distToTargetDist <= distanceRange && Math.abs(horzDistToTarget) <= horzDistRange) {
                if (endingInchesPerSecond < 0.01) {
                    e.stopSwerveMotors();
                } else {
                    e.assignRobotMotionAndHeadingField(calcHeading, endingInchesPerSecond, facingAngle);
                }
                rLog.print("driveAligned Done. Dist=" + RobotMath.round2(currentDistance) +
                        " Right=" + RobotMath.round2(currentInchesRightOfTarget));
                return true;
            } else {
                if (driveToMode == false) {
                    e.assignRobotMotionAndHeadingField(calcHeading, finalVel, facingAngle);
                } else {
                    driveToVisionTarget(facingAngle, targetDistance, -targetInchesRightOfTarget, distanceRange,
                            horzDistRange, travelSpd, maxTravelAcceleration, endingInchesPerSecond, isTargetFound,
                            currentDistance, -currentInchesRightOfTarget);
                }
            }

            if (isTooFar != lastTooFar || isTooClose != lastTooClose ||
                    isTooRight != lastTooRight || isTooLeft != lastTooLeft) {
                rLog.print("driveAligned: TF: " + isTooFar + " TC: " + isTooClose +
                        " TR: " + isTooRight + " TL: " + isTooLeft +
                        " Dist=" + RobotMath.round2(currentDistance) +
                        " Right=" + RobotMath.round2(currentInchesRightOfTarget));
            }
            lastTooFar = isTooFar;
            lastTooClose = isTooClose;
            lastTooRight = isTooRight;
            lastTooLeft = isTooLeft;
        } else {
            double finalVel = limitAccel(0, maxTravelAcceleration);
            e.assignRobotMotionAndHeadingField(facingAngle, finalVel, facingAngle);
        }

        return false;
    }

    /**
     * Limits the acceleration of the robot based on the travel speed and max travel
     * acceleration.
     * 
     * @param goalInPerSec
     * @param maxTravelAcceleration
     */
    public double limitAccel(double goalInPerSec, double maxTravelAcceleration) {
        double calcInPerSec;
        if (goalInPerSec >= e.getLastTravelVelocityCommand()) {
            calcInPerSec = Math.min(goalInPerSec,
                    e.getLastTravelVelocityCommand() + e.getMaxSpeedChange(maxTravelAcceleration));
        } else {
            calcInPerSec = Math.max(goalInPerSec,
                    e.getLastTravelVelocityCommand() - e.getMaxSpeedChange(maxTravelAcceleration));
        }
        return calcInPerSec;
    }

    public double calcFinalVel(double endingInchesPerSecond, double distToGo, double maxTravelSpeed,
            double maxTravelAcceleration) {
        double dwnInPerSec = endingInchesPerSecond + (distToGo * 1.6);
        double calcInPerSec = Math.min(maxTravelSpeed, dwnInPerSec);
        double finalVel = limitAccel(calcInPerSec, maxTravelAcceleration);
        return finalVel;
    }

    public double limitUpAccel(double goalInPerSec, double maxTravelAcceleration) {
        return (Math.min(goalInPerSec, e.getLastTravelVelocityCommand() + e.getMaxSpeedChange(maxTravelAcceleration)));
    }

    public boolean alignWheelsForawrd() {
        e.alignSwerveMotorsForward();
        return true;
    }

    public double getDistanceTraveled() {
        return calcDistTraveled;
    }

    public void logChanges() {
        if (logTimer.get() < 3.0) {
            return; // Start logging changes after 3 seconds
        }

        boolean leftCameraAttached = PhotonVision.isAprilTagLeftCameraAttached();
        boolean rightCameraAttached = PhotonVision.isAprilTagRightCameraAttached();
        boolean cubeCameraAttached = PhotonVision.isCubeCameraAttached();
        boolean aprilTagFound = robotState.isAprilTagFound;
        int aprilTagID = robotState.aprilTagId;
        boolean driverStationAttached = DriverStation.isDSAttached();

        if (leftCameraAttached != lastLeftCameraAttached) {
            rLog.print("LeftCamera Attached: " + leftCameraAttached);
        }
        if (rightCameraAttached != lastRightCameraAttached) {
            rLog.print("RightCamera Attached: " + rightCameraAttached);
        }
        if (cubeCameraAttached != lastCubeCameraAttached) {
            rLog.print("CubeCamera Attached: " + cubeCameraAttached);
        }
        if (driverStationAttached != lastDriverStationAttached) {
            rLog.print("Driver Station Attached: " + driverStationAttached);
        }
        if (robotState.logAprilTags && (aprilTagFound != lastAprilTagFound || robotState.aprilTagId != lastAprilTagID)) {
            if (robotState.isAprilTagFound == false) {
                rLog.print("No April Tag. Previous ID " + lastAprilTagID +
                        " FrontOf " + RobotMath.round2(lastInchesInFrontOfTarget) +
                        " RightOf " + RobotMath.round2(lastInchesRightOfTarget));
            } else {
                rLog.print("April Tag Found " + aprilTagID +
                        " FrontOf " + RobotMath.round2(robotState.aprilTagInchesInFrontOfTarget) +
                        " RightOf " + RobotMath.round2(robotState.aprilTagInchesRightOfTarget));
            }
        }
        lastLeftCameraAttached = leftCameraAttached;
        lastRightCameraAttached = rightCameraAttached;
        lastCubeCameraAttached = cubeCameraAttached;
        lastDriverStationAttached = driverStationAttached;
        lastAprilTagFound = aprilTagFound;
        lastAprilTagID = aprilTagID;
        lastInchesInFrontOfTarget = robotState.aprilTagInchesInFrontOfTarget;
        lastInchesRightOfTarget = robotState.aprilTagInchesRightOfTarget;
    }
}