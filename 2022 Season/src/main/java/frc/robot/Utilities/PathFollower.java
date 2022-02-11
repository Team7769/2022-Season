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

    public PathFollower()
    {
        _controller = new RamseteController();
        _timer = new Timer();
    }
    public void startPath()
    {
        _timer.reset();
        _timer.start();
    }
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
    public void setTestPath(TrajectoryConfig config)
    {
        _currentPath = generateTestTrajectory(config);
    }
    public void setDriveForwardAndShootPath(TrajectoryConfig config)
    {
        _currentPath = getDriveForwardAndShootTrajectory(config);
    }
    public void setCollectTwoFromTerminalPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getCollectTwoFromTerminalTrajectory(config, startingPose);
    }
    public void setDriveBackFromTerminalPath(TrajectoryConfig config, Pose2d startingPose)
    {
        _currentPath = getDriveBackFromTerminalTrajectory(config, startingPose);
    }
    public void setFiveBallPartOnePath(TrajectoryConfig config)
    {
        _currentPath = getFiveBallPartOneTrajectory(config);
    }
    public void setFiveBallPartTwoToTerminalPath(TrajectoryConfig config, Rotation2d startingPose)
    {
        _currentPath = getFiveBallPartTwoToTerminalTrajectory(config, startingPose);
    }
    public void setFiveBallPartTwoFromTerminalPath(TrajectoryConfig config, Rotation2d startingPose)
    {
        _currentPath = getFiveBallPartTwoFromTerminalTrajectory(config, startingPose);
    }

    public Trajectory getCurrentTrajectory()
    {
        return _currentPath;
    }
    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialPose();
    }
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
    /*private Trajectory generateTrajectory(TrajectoryConfig config) {

        var start = new Pose2d(3.129, 2.402, Rotation2d.fromDegrees(0));
        var end = new Pose2d(8.106, .75, Rotation2d.fromDegrees(0));
    
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(5.708, .75));
    
        var trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
        return trajectory;
      }*/
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

      private Trajectory getDriveForwardAndShootTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(6.71, 2.440, new Rotation2d(-2.437994417066567)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(5.874, 2.135)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.08, 1.831, new Rotation2d(-2.7716865861332294)),
        // Pass config
        config);
      }

      private Trajectory getCollectTwoFromTerminalTrajectory(TrajectoryConfig config, Pose2d currentPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2.6, 2.078)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.128, 1.156, new Rotation2d(-2.442932828868268)),
        // Pass config
        config);
      }
      
      private Trajectory getDriveBackFromTerminalTrajectory(TrajectoryConfig config, Pose2d currentPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2.6, 2.078)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.08, 1.83, new Rotation2d(-2.7716865861332294)),
        // Pass config
        config);
      }

      private Trajectory getFiveBallPartOneTrajectory(TrajectoryConfig config) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(8.663, 1.730, new Rotation2d(-1.1722738811284743)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(7.707, 0.054),
            new Translation2d(6.886, 0.740)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.818, 0.864, new Rotation2d(-2.4346845928296763)),
        // Pass config
        config);
      }

      private Trajectory getFiveBallPartTwoToTerminalTrajectory(TrajectoryConfig config, Rotation2d startingPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(5.818, 0.864, startingPose),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.173, 2, new Rotation2d(-2.442932828868268)),
        // Pass config
        config);
      }

      private Trajectory getFiveBallPartTwoFromTerminalTrajectory(TrajectoryConfig config, Rotation2d startingPose) {

        return TrajectoryGenerator.generateTrajectory(new Pose2d(1.173, 2, startingPose),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(5.975, 1.156, new Rotation2d(-2.5878351309802463)),
        // Pass config
        config);
      }

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