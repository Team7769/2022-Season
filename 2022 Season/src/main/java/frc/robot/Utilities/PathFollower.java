package frc.robot.Utilities;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private RamseteController _controller;
    private Trajectory _currentPath;
    private Timer _timer;
    
    /** 
        This is a PathFollower constructor
    */
    public PathFollower()
    {
        _controller = new RamseteController();
        _timer = new Timer();
    }

    /**
     * Reset old timer and starts a new one
     */
    public void startPath()
    {
        _timer.reset();
        _timer.start();
    }

    /** 
     * Checks if the current path has been finished
    */
    public boolean isPathFinished()
    {
        return _timer.get() > _currentPath.getTotalTimeSeconds();
    }

    // public void setLineToTrenchPath(TrajectoryConfig config)
    // {
    //     _currentPath = getLineToTrenchTrajectory(config);
    // }
    // public void setTrenchToLinePath(TrajectoryConfig config)
    // {
    //     _currentPath = getTrenchToLineTrajectory(config);
    // }

    /** 
     * Sets the current Path of the pathFollower to be the path of TestPath
     * @param config - The trajectory configuration for the new path
    */
    public void setTestPath(TrajectoryConfig config)
    {
        _currentPath = generateTestTrajectory(config);
    }

    /** 
     * Sets current Path as drive Forward and shoot
     * @param config - The trajectory configuration for the new path
    */
    public void setDriveForwardAndShootPath(TrajectoryConfig config)
    {
        _currentPath = getDriveForwardAndShootTrajectory(config);
    }

    /** 
     * Sets current Path as Collect Two From Terminal
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setCollectTwoFromTerminalPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getCollectTwoFromTerminalTrajectory(config, startingPose);
    }

    /** 
     * Sets current Path as Drive Back From Terminal
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setDriveBackFromTerminalPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getDriveBackFromTerminalTrajectory(config, startingPose);
    }

    /** 
     * Sets current Path as Fifth Ball
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setFifthBallPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getFifthBallTrajectory(config, startingPose);
    }

    /** 
     * Sets current Path as The first Part for Five Ball
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setFiveBallPartOnePath(TrajectoryConfig config)
    {
        _currentPath = getFiveBallPartOneTrajectory(config);
    }

    /** 
     * Sets current Path as Part Two TO Terminal
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setFiveBallPartTwoToTerminalPath(TrajectoryConfig config, Rotation2d startingPose)
    {
        _currentPath = getFiveBallPartTwoToTerminalTrajectory(config, startingPose);
    }

    /** 
     * Sets current Path as Part Two FROM Terminal
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setFiveBallPartTwoFromTerminalPath(TrajectoryConfig config, Rotation2d startingPose)
    {
        _currentPath = getFiveBallPartTwoFromTerminalTrajectory(config, startingPose);
    }

    /** 
     * Sets current Path as Four Ball Path One
     * @param config - The trajectory configuration for the new path
    */
    public void setFourBallFarPartOneTrajectory(TrajectoryConfig config)
    {
        _currentPath = getFourBallFarPartOneTarejctory(config);
    }

    /** 
     * Sets current Path as Four Ball Part Two Out
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setFourBallFarPartTwoOutPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getFourBallFarPartTwoOutTrajectory(config, startingPose);
    }

    /** 
     * Sets current Path as Four Ball Far Part Two Back Path
     * @param config - The trajectory configuration for the new path
     * @param startingPose - The Starting position of the robot when the path started
    */
    public void setFourBallFarPartTwoBackPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getFourBallFarPartTwoBackTrajectory(config, startingPose);
    }

    /** 
     * Gives the current Path of the Path Follower
     * @return - The Current Path
    */
    public Trajectory getCurrentTrajectory()
    {
        return _currentPath;
    }

    /**
     * Gives the inital position of the current Path
     * @return - Current Path Initial Position
     */
    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialPose();
    }

    /**
     * Gets the path target speed
     * @param currentPose - Current X, Y Cordinates of the Robot
     * @return - The differential Drive Wheel Speeds
     */
    public DifferentialDriveWheelSpeeds getPathTarget(Pose2d currentPose)
    {
        var time = _timer.get();
        var goal = _currentPath.sample(time);
        var targetSpeeds = _controller.calculate(currentPose, goal);
        SmartDashboard.putNumber("currentPoseX", currentPose.getX());
        SmartDashboard.putNumber("currentPoseY", currentPose.getY());
        SmartDashboard.putNumber("currentPoseDegrees", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("goalDegrees", goal.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("totalPathTime", _currentPath.getTotalTimeSeconds());
        SmartDashboard.putNumber("goalX", goal.poseMeters.getTranslation().getX());
        SmartDashboard.putNumber("goalY", goal.poseMeters.getTranslation().getY());

        return Constants.kDriveKinematics.toWheelSpeeds(targetSpeeds);
    }

    // Private Functions --------------------------------

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Test Path Cordinates
     */
    private Trajectory generateTestTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Drive Forward And Shoot Path Cordinates
     */
    private Trajectory getDriveForwardAndShootTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kDriveForwardStartX, Constants.kDriveForwardStartY, new Rotation2d(Constants.kDriveForwardStartRotation)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kDriveForwardMidX, Constants.kDriveForwardMidY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kDriveForwardEndX, Constants.kDriveForwardEndY, new Rotation2d(-2.7716865861332294)),
        // Pass confighh
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Collect Two From Terminal Path Cordinates
     */
    private Trajectory getCollectTwoFromTerminalTrajectory(TrajectoryConfig config, Pose2d currentPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kCollectTwoWayPointX, Constants.kCollectTwoWayPointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kCollectTwoEndPointX, Constants.kCollectTwoEndPointY, new Rotation2d(Constants.kCollectTwoEndRotation)),
        // Pass config
        config);
    }
      
    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Drive Back From Terminal Path Cordinates
     */
    private Trajectory getDriveBackFromTerminalTrajectory(TrajectoryConfig config, Pose2d currentPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kDriveBackWayPointX, Constants.kDriveBackWayPointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kDriveBackEndPointX, Constants.kDriveBackEndPointY, new Rotation2d(Constants.kDriveBackEndPointRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Fifth Ball Path Cordinates
     */
    private Trajectory getFifthBallTrajectory(TrajectoryConfig config, Pose2d currentPose) {

        return TrajectoryGenerator.generateTrajectory(currentPose,
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kFifthBallWayPointX, Constants.kFifthBallWayPointY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFifthBallEndPointX, Constants.kFifthBallEndPointY, new Rotation2d(Constants.kFifthBallEndPointRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Five Ball Part One Path Cordinates
     */
    private Trajectory getFiveBallPartOneTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kFiveBallPartOneStartX, Constants.kFiveBallPartOneStartY, new Rotation2d(Constants.kFiveBallPartOneStartRotation)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kFiveBallPartOneFirstX, Constants.kFiveBallPartOneFirstY),
            new Translation2d(Constants.kFiveBallPartOneSecondX, Constants.kFiveBallPartOneSecondY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFiveBallPartOneEndX, Constants.kFiveBallPartOneEndY, new Rotation2d(Constants.kFiveBallPartOneEndRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Five Ball Part Two To Terminal Path Cordinates
     */
    private Trajectory getFiveBallPartTwoToTerminalTrajectory(TrajectoryConfig config, Rotation2d startingPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kFiveBallPartTwoToTerminalStartX, Constants.kFiveBallPartTwoToTerminalStartY, startingPose),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFiveBallPartTwoToTerminalEndX, Constants.kFiveBallPartTwoToTerminalEndY, new Rotation2d(Constants.kFiveBallPartTwoToTerminalEndRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Five Ball Part Two From Terminal Path Cordinates
     */
    private Trajectory getFiveBallPartTwoFromTerminalTrajectory(TrajectoryConfig config, Rotation2d startingPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kFiveBallPartTwoFromTerminalStartX, Constants.kFiveBallPartTwoFromTerminalStartY, startingPose),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFiveBallPartTwoFromTerminalEndX, Constants.kFiveBallPartTwoFromTerminalEndY, new Rotation2d(Constants.kFiveBallPartTwoFromTerminalEndRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Four Ball Far Part One Path Cordinates
     */
    private Trajectory getFourBallFarPartOneTarejctory(TrajectoryConfig config)
    { 
        return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kFourBallFarPartOneStartX, Constants.kFourBallFarPartOneStartX, new Rotation2d(Constants.kFourBallFarPartOneStartRotation)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFourBallFarPartOneEndX, Constants.kFourBallFarPartOneEndY, new Rotation2d(Constants.kFourBallFarPartOneEndRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Four Ball Far Part Two Out Path Cordinates
     */
    private Trajectory getFourBallFarPartTwoOutTrajectory(TrajectoryConfig config, Pose2d currentPose)
    { 
        return TrajectoryGenerator.generateTrajectory(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kFourBallFarPartTwoMidX, Constants.kFourBallFarPartTwoMidY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFourBallFarPartTwoEndPointX, Constants.kFourBallFarPartTwoEndPointY, new Rotation2d(Constants.kFourBallFarPartTwoEndRotation)),
        // Pass config
        config);
    }

    /**
     * Returns new Trajectory class
     * @param config - Configuration for the trajectory
     * @return - new Trajectory class with the Four Ball Far Part Two Back Path Cordinates
     */
    private Trajectory getFourBallFarPartTwoBackTrajectory(TrajectoryConfig config, Pose2d currentPose)
    { 
        return TrajectoryGenerator.generateTrajectory(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(Constants.kFourBallFarPartTwoBackMidX, Constants.kFourBallFarPartTwoBackMidY)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(Constants.kFourBallFarPartTwoBackEndPointX, Constants.kFourBallFarPartTwoBackEndPointY, new Rotation2d(Constants.kFourBallFarPartTwoBackEndRotation)),
        // Pass config
        config);
    }

    /*private Trajectory generateTrajectory(TrajectoryConfig config) {

        var start = new Pose2d(3.129, 2.402, Rotation2d.fromDegrees(0));
        var end = new Pose2d(8.106, .75, Rotation2d.fromDegrees(0));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(5.708, .75));
    
        var trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        return trajectory;
      }*/
      

    //   private Trajectory getTrenchToLineTrajectory(TrajectoryConfig config)
    //   {
    //     return TrajectoryGenerator.generateTrajectory(new Pose2d(Constants.kTrenchPathEndX,Constants.kTrenchPathEndY, new Rotation2d(Constants.kTrenchPathEndAngle)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(Constants.kTrenchPathMidpointX, Constants.kTrenchPathMidpointY)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(Constants.kCenterGoalLineX, Constants.kCenterGoalLineY, new Rotation2d(Constants.kCenterGoalLineStartAngle)),
    //     // Pass config
    //     config);
    //   }
    }