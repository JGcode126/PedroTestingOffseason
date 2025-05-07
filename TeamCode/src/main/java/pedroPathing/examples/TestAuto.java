package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Test Auto", group = "Examples")
public class TestAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8, 80, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(35, 80, Math.toRadians(0));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose transitionPose = new Pose(40, 30, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose forward1 = new Pose(66, 30, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pushback1 = new Pose(12, 24, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose forward2 = new Pose(66, 17, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose pushback2 = new Pose(10, 11, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain moveToTransition, firstForward, firstBackward, secondForward, secondBackward, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        moveToTransition = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(35.286, 79, Point.CARTESIAN), new Point(0,31,Point.CARTESIAN), new Point(7, 24, Point.CARTESIAN), new Point(40,31, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()
                .build();
        firstForward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(transitionPose), new Point(forward1)))
                .setLinearHeadingInterpolation(transitionPose.getHeading(), forward1.getHeading())
                .build();
        firstBackward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forward1), new Point(pushback1)))
                .setLinearHeadingInterpolation(forward1.getHeading(), pushback1.getHeading())
                .build();
        secondForward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushback1), new Point(forward2)))
                .setLinearHeadingInterpolation(pushback1.getHeading(), forward2.getHeading())
                .build();
        secondBackward = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forward2), new Point(pushback2)))
                .setLinearHeadingInterpolation(forward2.getHeading(), pushback2.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(moveToTransition,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(firstForward,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(firstBackward,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(secondForward,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(secondBackward,true);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

