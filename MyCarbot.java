package robotcontroller;

import java.util.ArrayList;

import basics.grid.LongGrid;
import basics.points.PointCloudCreator2D;
import basics.points.PointList2D;
import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.RobotGeomUtil;
import robotinterface.Time;
import robotinterface.debug.DebugPainterOverlay;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.mss.MotionSubsystemListener;
import robotlib.driver.Driver;
import robotlib.driver.RegulatedAheadDriver;
import robotlib.nav.Pos2PosRouting;
import robotlib.nav.grid.Grid_Astar;
import robotlib.nav.grid.Pos2PosRoutingGrid;
import robotlib.navtraj.NavTrajPlanning;
import robotlib.navtraj.NavTrajSplitPlanning;
import robotlib.navtraj.RouteTraj;
import robotlib.traj.TrajectoryPlanner;
import robotlib.traj.atomic.TurnInPlaceTrajectory;
import robotlib.traj.longrange.LongrangePlanner;
import robotlib.traj.longrange.LongrangeProfile;
import robotlib.traj.seq.Maneuver;
import robotlib.traj.seq.TrajectorySequence;
import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;

/**
 * A Robot controller to demonstrate a rectangle-partitioning cleaning strategy.
 */
public class MyCarbot extends RobotController implements MotionSubsystemListener, LidarSubsystemListenerSlam {
  private NavTrajPlanning navTraj = null;
  private Driver driver = null;

  private double posXMSS = Double.NaN; // Last x position (obtained via async mss message)
  private double posYMSS = Double.NaN; // Last y position (obtained via async mss message)
  private double posAngMSS = Double.NaN; // Last angle (obtained via async mss message)

  private ObstacleContainer obstaclesLidar = null;

  private CleaningPlanner planner = new CleaningPlanner();

  /** The vertical offset when cleaning a rectangle. */
  private final int CLEANING_OFFSET_VERICAL = (int) Robot.robotWidth;

  /** Column width of the rectangles. Needs to be smaller than the robot. */
  private final double CLEANING_STEP_HORIZONTAL = Robot.robotWidth * 0.75;

  /** Maximum number of attempts to drive to a rectangle before failing. */
  private final int DRIVE_TO_RECTANGLE_MAX_ATTEMPTS = 3;

  /**
   * Trying to drive through a wall? Drive back by this amount.
   */
  private final int DRIVE_BACK_SIZE = 20;

  /**
   * If the target is set too close to a wall, set it iteratively closer to the
   * robot by this amount until it works.
   */
  private final int DRIVE_STRAIGHT_OFFSET = 5;

  /** Robot drove into a wall. */
  private boolean isStuck;

  public static void main(String[] args) {
    CleaningPlannerTest.test();
  }

  public MyCarbot() {
    Robot.motionSubsystem.registerMotionListener(this);

    // Can't run without Lidar -> exit
    if (Robot.lidarSubsystem == null) {
      LogUtils.println("No Lidar Subsystem available - I cannot see anything!");
      return;
    }

    // Start Lidar-SLAM
    try {
      Robot.lidarSubsystem.setTiming(LidarSubsystem.EQUIDISTANT, 1000);
      Robot.lidarSubsystem.registerLidarListenerSlam(this);
      Robot.lidarSubsystem.setMSSCorrection(true);
    } catch (UnsupportedOperationException e) {
      LogUtils.println("Lidar Subsystem does not provide SLAM-correction");
    }

    // Set up obstacle container
    obstaclesLidar = new ObstacleContainer(
        PointCloudCreator2D.TYPE_GRID, 10.0d,
        ObstacleContainer.ADD_MODE_FUSION_WEIGHT,
        3.0d);
  }

  /**
   * Set up everything related to path planning.
   */
  private void initPathPlanning() {
    LogUtils.println("Instantiate path planning...");

    TrajectoryPlanner trajectoryPlannerWithBackdriving = new LongrangePlanner(
        LongrangeProfile.createBestPracticeProfile()
            .setPlanningFlagsWithTargetAngle(
                Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES
                    | Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL,
                Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES
                    | Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL)
            .setPlanningFlagsWithoutTargetAngle(
                Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES
                    | Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL,
                Maneuver.FLAGS_ALLOW_BACKDRIVING | Maneuver.FLAGS_ARCS_UPTO_360_DEGREES
                    | Maneuver.FLAGS_ALL_WITHOUT_TARGET_ANGLE_WO_CL)
            .setSplitDist(300.0d)
            .setArcarcProfile(new double[] { 1.01d, 3.0d, 5.0d }, new double[] { 1.5d, 1.0d, 1.0d })
            .setIntArcsProfile(new double[] { 1.01d, 3.0d, 5.0d }, new double[] { 1.5d, 1.0d, 1.0d })
            .setWingArcProfile(new double[] { 1.01d, 4.0d, 8.0d }, new double[] { 1.5d, 1.0d, 1.0d })
            .setSnakeProfile(new double[] { 1.01d, 1.7d }, new double[] { 1.5d, 1.0d })
            .setDubinsArcsProfile(new double[] { 1.01d, 1.5d, 3.0d }, new double[] { 1.5d, 1.3d, 1.0d })
            .setTurninplaceData(500, 2) // constantTurnInPlaceCosts,relativeTurnInPlaceCosts
            .setBackdrivingData(100, 10) // constantBackwardCosts,relativeBackwardCosts
            .setChangeDrivingDirectionData(50, // changeDrivingDirectionCosts
                Robot.headingDistance * 3.0d) // changeDrivingDirectionMinObstacleDist
            .setAlgoFlags(
                LongrangePlanner.FLAGS_TRAJ_ALGO_ANGLE_ALL_INCLUDING_REVERSE | LongrangePlanner.FLAGS_TRAJ_ALGO_VITERBI)
            .setName("Longrange(WITH back)"),
        null);

    Pos2PosRouting p2p = new Pos2PosRoutingGrid(
        1.1d * Robot.robotWidth / 2d, // obstacleBuffer
        10, // cellSize
        Grid_Astar.FLAGS_STEPWIDTH5 | Grid_Astar.FLAGS_DONT_CUT_CORNER
            | Grid_Astar.FLAGS_REMOVE_COLLINEAR_AND_BYPASSES | Grid_Astar.FLAGS_CONSIDER_ADDITIONAL_COSTS, // A*-Flags
        2.0d * Robot.robotWidth / 2d, // additionalCostsBuffer
        2.0d, // maxAdditionalCostsFactor
        3.0d // acceptedShortcutCosts
    );

    navTraj = new NavTrajSplitPlanning(p2p, trajectoryPlannerWithBackdriving);

    driver = new RegulatedAheadDriver(
        new LongrangePlanner(
            LongrangeProfile.createBestPracticeProfile()
                .setMaxTrajectoryStretch(2.0d)
                .setPlanningFlagsWithTargetAngle(
                    Maneuver.FLAGS_ALL_WITH_TARGET_ANGLE_WO_CL & ~Maneuver.FLAGS_SNAKE & ~Maneuver.FLAGS_SNAKE2 // Snake
                        // ausklammern,
                        // da
                        // die
                        // zu
                        // "Trudeln"
                        // führen
                        | Maneuver.FLAGS_ALLOW_BACKDRIVING
                        | Maneuver.FLAGS_NO_CHANGING_BACK_FORE
                        | Maneuver.FLAGS_ARCS_LESS_180_DEGREES
                        | Maneuver.FLAGS_ALLOW_TURN_IN_PLACE) // flags_single_trajectory
                .setArcarcProfile(new double[] { 1.02d, 3.0d, 5.0d }, new double[] { 2.5d, 1.0d, 1.0d })
                .setIntArcsProfile(new double[] { 1.02d, 3.0d, 5.0d }, new double[] { 2.5d, 1.0d, 1.0d })
                .setWingArcProfile(new double[] { 1.02d, 4.0d, 8.0d }, new double[] { 2.5d, 1.0d, 1.0d })
                .setSnakeProfile(new double[] { 1.02d, 1.7d }, new double[] { 1.5d, 1.0d }) // Wird eigentlich nicht
                // gebraucht, da oben
                // ausgeschlossen
                .setDubinsArcsProfile(new double[] { 1.02d, 1.5d, 3.0d }, new double[] { 1.5d, 1.3d, 1.0d })
                .setTurninplaceData(500, 2) // constantTurnInPlaceCosts,relativeTurnInPlaceCosts,
                .setBackdrivingData(100, 5) // constantBackwardCosts,relativeBackwardCosts
                .setChangeDrivingDirectionData(50, // changeDrivingDirectionCosts
                    Double.NaN) // changeDrivingDirectionMinObstacleDist
                .setAlgoFlags(LongrangePlanner.FLAGS_TRAJ_ALGO_VITERBI), // Keine FLAGS_TRAJ_ALGO_ANGLE_... notwendig,
            // da immer nur Routen mit 2 Punkten erfragt
            // werden
            null),
        RegulatedAheadDriver.REGULATION_FLAGS_BESTPRACTICE,
        60, // aheadDist,
        90, // rotateSpeed,
        29, // travelSpeedArc,
        30, // travelSpeedArcFast,
        5, // travelSpeedArcBack,
        50, // minFastArcRadius,
        50, // minFastArcLength,
        25, // travelSpeedLinear,
        30, // travelSpeedLinearFast,
        10, // travelSpeedLinearBack,
        50, // minFastLinearLength
        false);
    driver.setTrajectoryEvaluator(navTraj);
  }

  @Override
  public String getDescription() {
    return "A Robot controller to demonstrate a rectangle-partitioning cleaning strategy.";
  }

  @Override
  public boolean requiresConfiguration() {
    return false;
  }

  @Override
  public void run() throws Exception {
    initPathPlanning();

    LogUtils.println("Init subsystems...");

    // Set up Lidar
    if (Robot.lidarSubsystem == null) {
      LogUtils.println("Navigation without Lidar Subsystem not possible!");
      return;
    }

    try {
      Robot.lidarSubsystem.resetWorldModel();
    } catch (UnsupportedOperationException e) {
      LogUtils.println("Lidar Subsystem does not use a World Model");
    }

    Robot.lidarSubsystem.startup();

    // Set up motion subsystem
    Robot.motionSubsystem.sendCommand("stoprule T");
    Robot.motionSubsystem.sendCommand("rotaterule T");

    // Wait until Lidar is ready
    Time.sleep(2000);

    scanWorld();
    onObstacleFound();

    Time.sleep(2000);

    planner.printGrid();

    CleanableRectangle rect;

    // In a perfect world, this would never get into an infinite loop,
    // but it's way too easy for a small bug to cause it anyway.
    int attemptsLeft = 1000;
    while (attemptsLeft > 0) {
      rect = planner.nextRectangle();

      if (rect == null) {
        LogUtils.println("No more rectangles to clean!");
        break;
      }

      boolean successDriveTo = driveToRectangle(rect);

      if (!successDriveTo) {
        LogUtils.println("Could not drive to rectangle. Marked the column as cleaned to avoid infinite loop.");
        markColumnAsDone(rect, rect.getXWorld());

        planner.printGrid();
        attemptsLeft--;

        continue;
      }

      boolean success = cleanRectangle(rect);
      LogUtils.println("cleanRectangle() done: " + success);

      planner.printGrid();
    }

    Robot.debugOut.sayToast("Done cleaning.");
    planner.printGrid();
  }

  /**
   * Drive to a given rectangle
   * 
   * @param rect The rectangle to drive to
   * @return True if the drive was successful, false otherwise
   */
  public boolean driveToRectangle(CleanableRectangle rect) {
    LogUtils.println("driveToRectangle() " + rect);

    if (!planner.isInRectBounds(posXMSS, posYMSS, rect)) {
      double xCenter = rect.getXWorld() + 0.5 * rect.getWidthWorld();
      double yCenter = rect.getYWorld() + 0.5 * rect.getHeightWorld();

      boolean success = navigateTo(xCenter, yCenter);

      if (!success)
        return false;
    }

    for (int i = 0; i < DRIVE_TO_RECTANGLE_MAX_ATTEMPTS; i++) {
      double x = rect.getXWorld();
      double y = rect.getYWorld() + i * CLEANING_OFFSET_VERICAL;
      Robot.debugOut.sayToast("Driving to bottom edge: " + GridUtils.getInGrid(x) + " " + GridUtils.getInGrid(y));

      if (canFindRoute(x, y) && navigateTo(x, y))
        return true;
    }

    return false;
  }

  /**
   * Look for a route to a given target.
   * 
   * @param x The x coordinate of the target in world space.
   * @param y The y coordinate of the target in world space.
   * @return True if a route was found, false otherwise.
   */
  private boolean canFindRoute(double x, double y) {
    return computeRouteAndTrajectories(x, y) != null;
  }

  /**
   * Tries to scan the world or at least most of it by navigating out of the
   * bounds until the path planning can't find a way anymore.
   * At that point, the outer bounds are scanned.
   * The walls received by Lidar aren't necessarily complete without gaps after
   * this.
   */
  public void scanWorld() {
    Robot.debugOut.sayToast("Scanning the world...");

    double x = posXMSS;
    double y = posYMSS;

    boolean keepGoing = true;
    do {
      y -= 1000;
      keepGoing = navigateTo(x, y);

      if (!keepGoing && planner.bounds.bottom < y) {
        keepGoing = true;
      }
    } while (keepGoing);

    Robot.debugOut.sayToast("Scanned the world...");
    LogUtils.println("#### Starting to clean ####", LogUtils.ANSI_GREEN);
  }

  /**
   * Navigate to a given world coordinate.
   * 
   * @param x The x coordinate in the world.
   * @param y The y coordinate in the world.
   * @return True if the navigation was successful.
   */
  public boolean navigateTo(double x, double y) {
    Robot.debugOut
        .sayToast("navigateTo() " + x + " " + y + "(" + GridUtils.getInGrid(x) + " " + GridUtils.getInGrid(y) + ")");

    debugTarget(x, y);

    // Try to find a route
    TrajectorySequence firstRouteTraj = computeRouteAndTrajectories(x, y);

    // Can't find a route
    if (firstRouteTraj == null) {
      LogUtils.println("Can't find route to " + x + " " + y);
      return false;
    }

    // Drive the route
    try {
      driver.drive(firstRouteTraj);
    } catch (Exception e) {
      LogUtils.println("IGNORE:");
      e.printStackTrace();
      return false;
    }

    int backAttemptsLeft = 4;
    while (isRunning() && !targetReached(x, y)) {
      Time.sleep(100);

      // If the starting position is a wall, fix this first.
      if (backAttemptsLeft > 0 && isStuck) {
        LogUtils.println("Stuck in obstacle. Driving back a little and trying again.");
        try {
          Robot.motionSubsystem.sendCommand("back " + DRIVE_BACK_SIZE);
          Time.sleep(500);
        } catch (Exception e) {
          e.printStackTrace();
        }
        backAttemptsLeft--;
        continue;
      }

      onObstacleFound();

      if (driver.isHalted()) {
        if (targetReached(x, y))
          continue;

        TrajectorySequence routeTraj = computeRouteAndTrajectories(x, y);

        if (routeTraj == null)
          return false;

        try {
          driver.drive(routeTraj);
        } catch (Exception e) {
          LogUtils.println("IGNORE:");
          e.printStackTrace();
          return false;
        }
      }
    }
    return true;
  }

  /**
   * Takes all the Lidar obstacles and passes them to the
   * planner.
   */
  private void onObstacleFound() {
    double[][] arrObstacles = obstaclesLidar.getObstactles2D();

    for (int i = 0; i < arrObstacles.length; i++) {
      int x = GridUtils.getInGrid(arrObstacles[i][0]);
      int y = GridUtils.getInGrid(arrObstacles[i][1]);

      // Enter the obstacle into the planning grid
      planner.handleObstacle(x, y);
    }
  }

  /**
   * Cleans a rectangle.
   * 
   * @param rect The rectangle to clean.
   * @return True if the cleaning was successful.
   */
  private boolean cleanRectangle(CleanableRectangle rect) {
    Robot.debugOut.sayToast("Cleaning the rectangle: " + rect.id + " (" + (rect.id % 9) + ")");

    final double x = rect.getXWorld();
    final double y = rect.getYWorld();
    final double w = rect.getWidthWorld();
    final double h = rect.getHeightWorld();

    Robot.debugOut.sayToast("cleanRectangle()" + x + " " + y + " " + w + " " + h);

    double yBottom = y + CLEANING_OFFSET_VERICAL;
    double yTop = y + h - CLEANING_OFFSET_VERICAL;
    double xCurrent = x;
    double xRight = x + w;

    Robot.debugOut.sayToast("cleanRectangle() " + xCurrent + " | " + yBottom + " | " + xRight + " | " + yTop);

    boolean success = false;

    // Path:
    // 1-2
    // | |
    // | |
    // 0 3-4 (repeat)
    while (xCurrent < xRight) {
      // up
      debugArrow(posXMSS, yTop, "red");
      success = driveStraight(xCurrent, yTop, true);
      if (!success)
        return false;
      markColumnAsDone(rect, xCurrent);

      LogUtils.println("Driving right.");
      // right
      xCurrent += CLEANING_STEP_HORIZONTAL;
      debugArrow(xCurrent, posYMSS, "red");
      success = driveStraight(xCurrent, posYMSS);
      if (!success)
        return false;

      // down
      debugArrow(xCurrent, yBottom, "red");
      success = driveStraight(xCurrent, yBottom, true);
      if (!success)
        return false;
      markColumnAsDone(rect, xCurrent);

      // right
      xCurrent += CLEANING_STEP_HORIZONTAL;
      debugArrow(xCurrent, posYMSS, "red");
      success = driveStraight(xCurrent, posYMSS);
      if (!success)
        return false;
    }

    return true;
  }

  /**
   * Mark a rectangle as partially done.
   * Everything in the rectangle is marked as done
   * up to the given x coordinate (inclusive).
   * 
   * @param rect The rectangle that's currently being cleaned (or that has failed
   *             to clean.)
   * @param x    The x coordinate in world coordinates.
   */
  private void markColumnAsDone(MyRectangle rect, double x) {
    LogUtils.println("markColumnAsDone()");

    int xGrid = GridUtils.getInGrid(x) + GridUtils.getInGrid(Robot.robotWidth / 3);
    int widthCleaned = xGrid - rect.x;

    /** The remaining dirty width in this rectangle. */
    int remaining = Math.abs(rect.width - widthCleaned);
    if (remaining < 3) {
      LogUtils.println("Rectangle is almost done. Marking as done: " + widthCleaned + " | " + remaining);
      planner.setDone(rect);
      return;
    }

    planner.setDone(new MyRectangle(rect.x, rect.y, widthCleaned, rect.height));
    planner.printGrid();
  }

  @Override
  public void pause() throws Exception {
    Robot.debugOut.sayToast("Didn't bother to implement this, good luck.");
    if (Robot.lidarSubsystem != null) {
      Robot.lidarSubsystem.shutdown();
    }
    if (driver != null)
      driver.halt(Driver.HALT_REASON_CONTROLLER_PAUSE);
  }

  @Override
  public void stop() throws Exception {
    if (Robot.lidarSubsystem != null) {
      Robot.lidarSubsystem.shutdown();
    }

    if (driver != null) {
      driver.halt(Driver.HALT_REASON_CONTROLLER_STOP);
    }

    obstaclesLidar.clear();
  }

  @Override
  public void mssResponse(ArrayList<String> messages, int responseType) throws Exception {
    if (MotionSubsystemListener.isFailureResponse(responseType))
      LogUtils.println("Failure response " + messages.get(0));
  }

  @Override
  public void mssAsyncMessages(ArrayList<String> messages, AsyncMotionMessageBundle bundle) throws Exception {
    if (bundle.containsPos()) {
      posXMSS = bundle.getDouble(AsyncMotionMessage.X);
      posYMSS = bundle.getDouble(AsyncMotionMessage.Y);
      posAngMSS = bundle.getDouble(AsyncMotionMessage.ANG);
    }

    if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL)) {
      isStuck = true;
    } else {
      isStuck = false;
    }

    if (bundle.containsType(AsyncMotionMessage.STOPPED)) {
      driver.mssStoppedReceived();
    }
  }

  @Override
  public void observedLidarPointsSlam(LidarPackageSlam lidarPackageSlam) throws Exception {
    if (!lidarPackageSlam.isSuccessful())
      return;

    if (driver != null && !driver.isHalted() && navTraj.hasRouted()) {

      // Add Lidar points to the path planning
      PointList2D<ObstaclePoint> newObstacles = obstaclesLidar.addLidarPoints(lidarPackageSlam, 1.0d);
      debugObstacles();

      if (!navTraj.addObstacles(newObstacles.getAll2D())) {
        driver.halt(Driver.HALT_REASON_NAVIGATION, "Cannot add new obstacles to navigation world model.");
        return;
      }

      TrajectorySequence plannedTraj = driver.plannedTrajectories();
      if (plannedTraj != null) {
        double[] collision = navTraj.trajectoryCollision(plannedTraj);

        if (collision != null) { // Die aktuelle Trajektorie oder die restliche Sequenz ist nicht frei
          driver.halt(Driver.HALT_REASON_NAVIGATION, "Planned route goes through obstacle");
        }
      }
    }
  }

  /**
   * Visualize the Lidar obstacles in the world.
   */
  private void debugObstacles() {
    double[][] obstacles = ObstacleContainer.getObstactles2D(obstaclesLidar);

    DebugPainterOverlay ovl = Robot.debugPainter.getOverlay("Obstacles");
    ovl.clear();
    for (int i = 0; i < obstacles.length; i++)
      ovl.fillCircle(obstacles[i][0], obstacles[i][1], 5, 0, 0, 0, 255);
    ovl.paint();
  }

  /**
   * True if the given coordinates are close enough to the current position.
   * 
   * @param x The x coordinate in world coordinates.
   * @param y The y coordinate in world coordinates.
   * @return True if the robot is within a small distance of the target.
   */
  private boolean targetReached(double x, double y) {
    return (Math.hypot(posXMSS - x, posYMSS - y) < 5);
  }

  /**
   * Plan a route to the given coordinates.
   * 
   * @param x The x coordinate in world coordinates.
   * @param y The y coordinate in world coordinates.
   * @return A TrajectorySequence that can be passed to the driver.
   */
  private TrajectorySequence computeRouteAndTrajectories(double x, double y) {
    try {
      LogUtils.disable();
      RouteTraj routeTraj = navTraj.getRouteTraj(posXMSS, posYMSS, RobotGeomUtil.mssAngle2NavAngle(posAngMSS),
          Double.NaN, x, y, Double.NaN, Double.NaN,
          ObstacleContainer.getObstactles2D(obstaclesLidar));

      if (routeTraj.getRoute() == null) {
        LogUtils.println("No route found!");
        return null;
      }

      TrajectorySequence trajSeq = routeTraj.getTrajectories();
      if (trajSeq == null) {
        LogUtils.println("No trajectories found");
      }
      return trajSeq;
    } catch (Exception e) {
      LogUtils.println("IGNORE:");
      e.printStackTrace();
      return null;
    } finally {
      LogUtils.enable();
    }
  }

  /**
   * Drive to the given coordinates and return success asap.
   * 
   * @param x The x coordinate in world coordinates.
   * @param y The y coordinate in world coordinates.
   * @return True if the robot reached the target.
   */
  private boolean driveStraight(double x, double y) {
    return driveStraight(x, y, false);
  }

  /**
   * Drive to the given coordinates in a straight line.
   * This only supports horizontal and vertical movement in known territory.
   * 
   * @param x               The x coordinate in world coordinates.
   * @param y               The y coordinate in world coordinates.
   * @param asFarAsPossible If true, the robot will drive as far as possible
   *                        before returning success.
   * @return True if the robot reached the target. If asFarAsPossible is true,
   *         this is always true.
   */
  private boolean driveStraight(double x, double y, boolean asFarAsPossible) {
    // Define a preferred target
    double xTarget = x;
    double yTarget = y;

    // Figure out the nature of this trip (up, down, right, left)
    double deltaX = xTarget - posXMSS;
    double deltaY = yTarget - posYMSS;
    boolean isVertical = Math.abs(deltaY) > Math.abs(deltaX);
    LogUtils.println("[" + asFarAsPossible + "] driveStraight() " + deltaX + " " + deltaY + " " + isVertical);

    // Figure out the offset when needing to correct the target
    double offsetYDelta = yTarget < posYMSS ? DRIVE_STRAIGHT_OFFSET : -DRIVE_STRAIGHT_OFFSET;
    double offsetXDelta = xTarget < posXMSS ? DRIVE_STRAIGHT_OFFSET : -DRIVE_STRAIGHT_OFFSET;
    if (isVertical) {
      offsetXDelta = 0;
    } else {
      offsetYDelta = 0;
    }
    LogUtils.println("[" + asFarAsPossible + "] driveStraight() " + offsetXDelta + " " + offsetYDelta);

    /**
     * The boundaries of the world with a bit of padding to prevent infinite loops.
     */
    Bounds rectWorldBounds = planner.bounds.getExpanded(5);

    /**
     * Set this to true if a failed drive attempt should be retried with an updated
     * target.
     */
    boolean tryAgain = false;
    do {
      LogUtils.println("[" + asFarAsPossible + "] driveStraight() | " + xTarget + " | " + yTarget);
      debugTarget(xTarget, yTarget);

      // Accurate position is needed or the trajectory planner will bitch about it
      double[] pos = Robot.motionSubsystem.estimateCurrentPosition();

      /** The angle between the current location and the target. */
      double angle = Math.atan2(yTarget - pos[1], xTarget - pos[0]);

      // Look ahead to calculate a potential route and fail quickly.
      // BUG: This isn't failsafe, there could still be obstacles in the way.
      // However, given the constraints of this function (only use in known
      // territory), this is fine.
      if (computeRouteAndTrajectories(xTarget, yTarget) == null) {
        tryAgain = true;
        xTarget += offsetXDelta;
        yTarget += offsetYDelta;
        continue;
      }

      TrajectorySequence sequence = new TrajectorySequence(
          TurnInPlaceTrajectory.createTrajectory(pos[0], pos[1],
              RobotGeomUtil.mssAngle2NavAngle(pos[2]),
              angle));
      // Adding a linear trajectory here drove into a wall way too often, so I'm using
      // routed navigation instead.

      try {
        driver.drive(sequence);

        // Make sure the driver has time to get in motion
        Time.sleep(200);
      } catch (Exception e) {
        LogUtils.println("IGNORE:");
        e.printStackTrace();

        tryAgain = true;
        xTarget += offsetXDelta;
        yTarget += offsetYDelta;
      }

      while (!driver.isHalted()) {
        Time.sleep(100);
        onObstacleFound();
      }

      boolean success = navigateTo(xTarget, yTarget);
      if (success) {
        return true;
      } else {
        tryAgain = true;
        xTarget += offsetXDelta;
        yTarget += offsetYDelta;
      }
    } while (asFarAsPossible && tryAgain && rectWorldBounds.isInBounds(xTarget, yTarget));

    boolean success = targetReached(xTarget, yTarget);
    if (!success) {
      LogUtils.println(
          "[" + asFarAsPossible
              + "] driveStraight() Target not reached, trying again with automatic pathing: "
              + posXMSS + "|" + posYMSS + " -> " + xTarget
              + " | " + yTarget);

      success = navigateTo(xTarget, yTarget);
    }

    return success;
  }

  /**
   * Draws an arrow from the current position to a target.
   * 
   * @param x     The x coordinate in world coordinates.
   * @param y     The y coordinate in world coordinates.
   * @param color The color of the arrow. Supported values: "blue", "red",
   *              "green", "magenta" (default)
   */
  private void debugArrow(double x, double y, String color) {
    var overlay = Robot.debugPainter.getOverlay("Next path");
    if (color == "blue") {
      overlay.drawArrow(posXMSS, posYMSS, x, y, 20, 0, 0, 255, 64);
    } else if (color == "red") {
      overlay.drawArrow(posXMSS, posYMSS, x, y, 20, 255, 0, 0, 64);
    } else if (color == "green") {
      overlay.drawArrow(posXMSS, posYMSS, x, y, 20, 0, 255, 0, 64);
    } else {
      overlay.drawArrow(posXMSS, posYMSS, x, y, 20, 0, 255, 255, 255);
    }
  }

  /**
   * Draws a black cross at the given coordinates after clearing the previous one.
   * 
   * @param x The x coordinate in world coordinates.
   * @param y The y coordinate in world coordinates.
   */
  private void debugTarget(double x, double y) {
    var overlay = Robot.debugPainter.getOverlay("Next target");
    overlay.clear();
    overlay.drawCross(x, y, 20, 0, 0, 0, 96);
  }
}

/**
 * A rectangle that works with the {@link CleaningPlanner} class.
 * All coordinates are grid coordinates by default unless otherwise noted.
 */
class MyRectangle {
  public int x;
  public int y;
  public int width;
  public int height;

  public MyRectangle(int x, int y, int width, int height) {
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
  }

  /**
   * A rectangle based on world coordinates.
   * 
   * @param x      x coordinate in world coordinates
   * @param y      y coordinate in world coordinates
   * @param width  width in world coordinates
   * @param height height in world coordinates
   */
  public MyRectangle(double x, double y, double width, double height) {
    this.x = GridUtils.getInGrid(x);
    this.y = GridUtils.getInGrid(y);
    this.width = GridUtils.getInGrid(width);
    this.height = GridUtils.getInGrid(height);
  }

  /**
   * A new rectangle based on bounds.
   * 
   * @param bounds The bounds to use, for example
   */
  public MyRectangle(Bounds bounds) {
    this.x = bounds.left;
    this.y = bounds.bottom;
    this.width = bounds.right - bounds.left;
    this.height = bounds.top - bounds.bottom;
  }

  public double getXWorld() {
    return x * CleaningPlanner.GRID_SIZE;
  }

  public double getYWorld() {
    return y * CleaningPlanner.GRID_SIZE;
  }

  public double getWidthWorld() {
    return width * CleaningPlanner.GRID_SIZE;
  }

  public double getHeightWorld() {
    return height * CleaningPlanner.GRID_SIZE;
  }

  @Override
  public String toString() {
    return "Rectangle [x=" + x + ", y=" + y + ", width=" + width + ", height=" + height + "]";
  }

  public Bounds getBounds() {
    int minX = (int) Math.floor(x);
    int minY = (int) Math.floor(y);
    int maxX = (int) Math.ceil(x + width) - 1;
    int maxY = (int) Math.ceil(y + height) - 1;
    return new Bounds(minX, minY, maxX, maxY);
  }
}

/**
 * A {@link MyRectangle} with an additional id.
 */
class CleanableRectangle extends MyRectangle {
  public static long nextId = CleaningPlanner.VALUE_RECT;
  public long id;

  public CleanableRectangle(int x, int y, int width, int height) {
    super(x, y, width, height);
    this.id = nextId++;

    // Handle overflow gracefully.
    // Only leads to undefined behavior if more than Long.MAX_VALUE
    // rectangles are on-screen at once.
    if (nextId < CleaningPlanner.VALUE_RECT) {
      nextId = CleaningPlanner.VALUE_RECT;
    }
  }

  /**
   * Evaluates if this rectangle is worth cleaning.
   * 
   * @return True if the Robot should attempt to clean it.
   */
  public boolean isWorthCleaning() {
    return width > 1 && height > 1;
  }
}

/**
 * Manages everything related to the grid and partitioning strategy.
 * 
 * Protected fields and methods are only to be used from within
 * {@link CleaningPlanner} and {@link CleaningPlannerTest}.
 */
class CleaningPlanner {
  public static final int GRID_SIZE = 10;

  public static final long VALUE_DIRTY = 0;
  public static final long VALUE_OBSTACLE = 1;
  public static final long VALUE_CLEANED = 2;
  public static final long VALUE_DONE = 3;
  public static final long VALUE_RECT = 4;

  /** The known boundaries of this world. */
  Bounds bounds = new Bounds();

  /**
   * A high level discrete world.
   * Only access this directly in {@link CleaningPlanner} &
   * {@link CleaningPlannerTest}
   */
  protected LongGrid grid = new LongGrid(LongGrid.X_POSNEG | LongGrid.Y_POSNEG);

  /** The rectangle which is currently cleaned. */
  private CleanableRectangle currentRectangle;

  CleaningPlanner() {
  }

  /**
   * Checks if a given coordinate is inside the rectangle.
   * 
   * @param xWorld
   * @param yWorld
   * @param rect
   * @return
   */
  public boolean isInRectBounds(double xWorld, double yWorld, CleanableRectangle rect) {
    int x = GridUtils.getInGrid(xWorld);
    int y = GridUtils.getInGrid(yWorld);
    return grid.getLong(x, y) == rect.id;
  }

  /**
   * Searches the remaining space for the next rectangle to be cleaned.
   * 
   * @return A cleanable rectangle.
   */
  public CleanableRectangle nextRectangle() {
    LogUtils.println("nextRectangle()");

    wipeGrid();

    ArrayList<CleanableRectangle> rects = partitionGrid();

    if (rects.size() == 0)
      return null;

    // NOTE: Sort rectangles based on planned route distance.
    // This could be done by passing a reference to the RobotController.

    // Return the first rectangle worth cleaning
    for (CleanableRectangle rect : rects) {
      if (rect.isWorthCleaning()) {
        currentRectangle = rect;

        printGrid();
        return rect;
      }
    }

    return null;
  }

  /**
   * Marks the given rectangle as done in the grid.
   * This means that the rectangle is no longer considered for further cleaning
   * and is equivalent to a wall.
   * 
   * @param rect The rectangle to mark as done.
   */
  public void setDone(MyRectangle rect) {
    setRect(rect, VALUE_DONE);
  }

  private void setRect(MyRectangle rect, long value) {
    fillGrid(rect.x, rect.y, rect.x + rect.width - 1, rect.y + rect.height - 1, value);
  }

  private void wipeGrid() {
    for (int x = bounds.left; x <= bounds.right; x++) {
      for (int y = bounds.bottom; y <= bounds.top; y++) {
        if (grid.getLong(x, y) == VALUE_OBSTACLE)
          continue;

        if (grid.getLong(x, y) == VALUE_DONE)
          continue;

        grid.setLong(x, y, VALUE_DIRTY);
      }
    }
  }

  /**
   * Mark a given coordinate as an obstacle.
   * 
   * @param x The x coordinate in grid space.
   * @param y The y coordinate in grid space.
   */
  public void handleObstacle(int x, int y) {
    bounds.expand(x, y);

    long byte_before = grid.getLong(x, y);
    grid.setLong(x, y, VALUE_OBSTACLE);
    long byte_after = grid.getLong(x, y);

    if (currentRectangle == null)
      return;

    if (byte_before != VALUE_OBSTACLE && byte_after == VALUE_OBSTACLE) {
      if (byte_before == currentRectangle.id) {
        wipeGrid();
      }
    }
  }

  /**
   * Sets the given area to a value.
   * 
   * @param left   The leftmost coordinate.
   * @param bottom The bottommost coordinate.
   * @param right  The rightmost coordinate.
   * @param top    The topmost coordinate.
   * @param value  The value to set the area to.
   */
  protected void fillGrid(int left, int bottom, int right, int top, long value) {
    for (int x = left; x <= right; x++) {
      for (int y = bottom; y <= top; y++) {
        long valuePrevious = grid.getLong(x, y);
        if (valuePrevious != VALUE_OBSTACLE && valuePrevious != VALUE_DONE) {
          grid.setLong(x, y, value);
        }
      }
    }
  }

  /**
   * Partitions the whole grid into a list of rectangles.
   * 
   * @return A list of cleanable rectangles.
   */
  private ArrayList<CleanableRectangle> partitionGrid() {
    fixGridObstacles();

    ArrayList<CleanableRectangle> rects = new ArrayList<CleanableRectangle>();

    CleanableRectangle curRect;
    do {
      curRect = findRectangle();
      if (curRect != null) {
        rects.add(curRect);
      }
    } while (curRect != null);

    // printGrid();
    return rects;
  }

  /**
   * Searches the grid for the next rectangle and marks it.
   * This should only be used during partitioning, the returned rectangle isn't
   * necessarily worth cleaning.
   * 
   * @param rect The boundaries of the area to search.
   * @return The next rectangle in the grid.
   */
  private CleanableRectangle findRectangle() {
    LogUtils.println("findRectangle()");

    int[] posStart = getStartingCoords();

    if (posStart == null)
      return null;

    int x = posStart[0];
    int y = posStart[1];

    /** The height of the detected rectangle. */
    int height = getSpaceInDirection(x, y, 0, 1);

    /** The current width of the detected rectangle. */
    int width = 0;
    for (; x + width <= bounds.right; ++width) {
      int spaceUp = getSpaceInDirection(x + width, y, 0, 1);

      // This means that the current column has an obstacle that prevents the
      // rectangle from being extended further horizontally
      if (spaceUp < height)
        break;
    }

    CleanableRectangle retRect = new CleanableRectangle(x, y, width, height);
    setRect(retRect, retRect.id);

    // printGrid();
    return retRect;
  }

  /**
   * Returns the space in the given direction.
   * Space is defined as dirty cells in a straight line.
   * 
   * @param x          X coordinate
   * @param y          Y coordinate
   * @param directionX -1,0,1 depending the horizontal direction
   * @param directionY -1,0,1 depending the vertical direction
   * @return -1 if the x and y are out of bounds, otherwise the number of dirty
   *         tiles in the given direction.
   */
  private int getSpaceInDirection(int x, int y, int directionX, int directionY) {
    int space = 0;
    int xCur = x;
    int yCur = y;

    while (grid.getLong(xCur, yCur) == VALUE_DIRTY) {
      if (xCur < bounds.left)
        return -1;

      if (xCur > bounds.right)
        return -1;

      if (yCur < bounds.bottom)
        return -1;

      if (yCur > bounds.top)
        return -1;

      space++;

      xCur += directionX;
      yCur += directionY;
    }
    return space;
  }

  /**
   * Searches the given bounds for the next valid starting coordinate.
   * 
   * @param rect The boundaries of the area to search.
   * @return The starting coordinates of the next rectangle within the bounds.
   */
  protected int[] getStartingCoords() {
    // LogUtils.println("getStartingCoords()");

    // Start in the bottom left corner
    int x = bounds.left;
    int y = bounds.bottom;

    while (true) {
      // No more vertical space left
      if (y > bounds.top)
        return null;

      if (grid.getLong(x, y) == VALUE_DIRTY && !isOutOfWorld(x, y)) {
        LogUtils.println("getStartingCoords() returned: " + x + ", " + y);
        return new int[] { x, y };
      }

      // Get the next cell
      // 6 7 -->
      // 1 2 3 4 5 (out of bounds)
      x++;
      if (x > bounds.right) {
        x = bounds.left;
        y++;
      }
    }
  }

  /**
   * Checks if the given coordinates are out of bounds.
   * 
   * @param x The x coordinate in grid space.
   * @param y The y coordinate in grid space.
   * @return True if the coordinates are out of bounds.
   */
  public boolean isOutOfWorld(int x, int y) {
    return getSpaceInDirection(x, y, 0, 1) <= 0
        || getSpaceInDirection(x, y, 1, 0) <= 0
        || getSpaceInDirection(x, y, 0, -1) <= 0
        || getSpaceInDirection(x, y, -1, 0) <= 0;
  }

  /**
   * Print the grid to the console.
   */
  public void printGrid() {
    printGrid(-100000, -100000);
  }

  /**
   * Print the grid to the console.
   * The provided cell at the coordinate will be colored green.
   * 
   * @param x
   * @param y
   */
  public void printGrid(int x, int y) {
    System.out.println("\n---------------------------");
    System.out.println("Bottom left corner: " + bounds.left + "," + bounds.bottom);
    System.out.println("---------------------------");

    for (int yCurrent = bounds.top; yCurrent >= bounds.bottom; yCurrent--) {
      for (int xCurrent = bounds.left; xCurrent <= bounds.right; xCurrent++) {
        boolean shouldBeColored = xCurrent == x && yCurrent == y;

        String color = shouldBeColored ? LogUtils.ANSI_RED : null;

        long value = grid.getLong(xCurrent, yCurrent);
        if (value == VALUE_DIRTY) {
          LogUtils.print("·", color);
        } else if (value == VALUE_OBSTACLE) {
          LogUtils.print("█", color);
        } else if (value == VALUE_CLEANED) {
          LogUtils.print("░", color);
        } else if (value == VALUE_DONE) {
          LogUtils.print("▒", color);
        } else {
          // %9 is used to limit the grid indicators to a single digit
          long id = grid.getLong(xCurrent, yCurrent);
          int num = (int) (id - VALUE_RECT) % 9;
          if (id == currentRectangle.id) {
            LogUtils.print(num + "", LogUtils.ANSI_BLUE);
          } else {
            LogUtils.print(num + "", color);
          }
        }
      }
      System.out.println();
    }
  }

  // @formatter:off
  /**
   * Try to prepare the grid for partitioning by removing minor flaws.
   * 
   * Currently, this method fills in walls:
   * 
   * █·█ -> ███
   * 
   * █    █
   * · -> █
   * █    █
   * 
   * and corners of obstacles:
   * 
   *  █     ███
   * █·█ -> █·█
   *  █     ███
   */
  // @formatter:on
  protected void fixGridObstacles() {
    for (int y = bounds.top; y >= bounds.bottom; y--) {
      for (int x = bounds.left; x <= bounds.right; x++) {
        // Get near cells
        boolean middle = grid.getLong(x, y) == VALUE_OBSTACLE;

        boolean up = grid.getLong(x, y + 1) == VALUE_OBSTACLE;
        boolean down = grid.getLong(x, y - 1) == VALUE_OBSTACLE;
        boolean left = grid.getLong(x - 1, y) == VALUE_OBSTACLE;
        boolean right = grid.getLong(x + 1, y) == VALUE_OBSTACLE;

        boolean upright = grid.getLong(x + 1, y + 1) == VALUE_OBSTACLE;
        boolean upleft = grid.getLong(x - 1, y + 1) == VALUE_OBSTACLE;
        boolean downright = grid.getLong(x + 1, y - 1) == VALUE_OBSTACLE;
        boolean downleft = grid.getLong(x - 1, y - 1) == VALUE_OBSTACLE;

        if (!middle) {
          // Fill vertical wall gaps
          if (up && down) {
            grid.setLong(x, y, VALUE_OBSTACLE);
            continue;
          }

          // Fill horizontal wall gaps
          if (left && right) {
            grid.setLong(x, y, VALUE_OBSTACLE);
            continue;
          }

          // Fill outer corners
          // ·····
          // ·o█o·
          // ·███·
          // ·o█o·
          // ·····
          boolean isTopRightCorner = left
              && down
              && !upleft
              && !downright;
          boolean isTopLeftCorner = right
              && down
              && !upright
              && !downleft;
          boolean isBottomRight = left
              && up
              && !upright
              && !downleft;
          boolean isBottomLeft = right
              && up
              && !upleft
              && !downright;

          if (isTopRightCorner || isTopLeftCorner || isBottomRight || isBottomLeft) {
            grid.setLong(x, y, VALUE_OBSTACLE);
            continue;
          }
        }
      }
    }
  }
}

/**
 * Generic boundaries in grid space.
 */
class Bounds {
  public int bottom;
  public int top;
  public int left;
  public int right;

  public Bounds() {
    this.bottom = 0;
    this.top = 0;
    this.left = 0;
    this.right = 0;
  }

  public Bounds(int left, int bottom, int right, int top) {
    this.left = left;
    this.bottom = bottom;
    this.right = right;
    this.top = top;
  }

  /**
   * Expands the boundaries to fit the given point.
   * 
   * @param x x-coordinate
   * @param y y-coordinate
   */
  public void expand(int x, int y) {
    left = Math.min(left, x);
    right = Math.max(right, x);
    bottom = Math.min(bottom, y);
    top = Math.max(top, y);
  }

  /**
   * Check if the given world coordinate is within the bounds.
   * 
   * @param x x-coordinate in world space
   * @param y y-coordinate in world space
   * @return True if the given coordinate is within the bounds.
   */
  public boolean isInBounds(double x, double y) {
    int xGrid = GridUtils.getInGrid(x);
    int yGrid = GridUtils.getInGrid(y);
    return xGrid >= left && xGrid <= right && yGrid >= bottom && yGrid <= top;
  }

  /**
   * Gets a new WorldBoundaries object that is expanded by the given amount in
   * all directions.
   * 
   * @param distance The amount to expand the boundaries by in grid space.
   * @return A new Bounds instance.
   */
  public Bounds getExpanded(int distance) {
    return new Bounds(left - distance, bottom - distance, right + distance, top + distance);
  }
}

/**
 * Utility class to handle grid <-> world space conversions.
 */
class GridUtils {
  /**
   * Converts a grid coordinate to a world coordinate.
   * 
   * @param coord The coordinate to convert.
   * @return The world coordinate.
   */
  public static double getInWorld(int coord) {
    return coord * CleaningPlanner.GRID_SIZE;
  }

  /**
   * Converts a world coordinate to a grid coordinate.
   * 
   * @param coord The coordinate to convert.
   * @return The grid coordinate.
   */
  public static int getInGrid(double coord) {
    return (int) Math.rint(coord / CleaningPlanner.GRID_SIZE);
  }
}

/**
 * Utility class to improve logging.
 */
class LogUtils {
  // Used for displaying color during terminal output
  public static final String ANSI_RESET = "\u001B[0m";
  public static final String ANSI_BLACK = "\u001B[30m";
  public static final String ANSI_RED = "\u001B[31m";
  public static final String ANSI_GREEN = "\u001B[32m";
  public static final String ANSI_YELLOW = "\u001B[33m";
  public static final String ANSI_BLUE = "\u001B[34m";
  public static final String ANSI_PURPLE = "\u001B[35m";
  public static final String ANSI_CYAN = "\u001B[36m";
  public static final String ANSI_WHITE = "\u001B[37m";

  /**
   * Proxy for println()
   * 
   * @param message The message to print.
   */
  public static void println(String message) {
    try {
      Robot.debugOut.println(message);
    } catch (Exception e) {
      System.out.println(message);
    }
  }

  /** Reset log color after disable() */
  public static void enable() {
    print(ANSI_RESET);
  }

  /** Force-hide log messages by printing them in black. */
  public static void disable() {
    // print(ANSI_BLACK);
  }

  /**
   * Proxy for println() with color
   * 
   * @param message The message to print.
   * @param color   Color to print. Values: LogUtils.ANSI_RED, ...
   */
  public static void println(String message, String color) {
    if (color != null)
      print(color);

    println(message);

    if (color != null)
      print(ANSI_RESET);
  }

  /**
   * Proxy for print()
   * 
   * @param msg
   */
  public static void print(String message) {
    System.out.print(message);
  }

  /**
   * Proxy for print() with color
   * 
   * @param message Message to print
   * @param color   Color to print. Values: LogUtils.ANSI_RED, ...
   */
  public static void print(String message, String color) {
    if (color != null)
      print(color);

    print(message);

    if (color != null)
      print(ANSI_RESET);
  }
}

class CleaningPlannerTest extends CleaningPlanner {
  public static void test() {
    new CleaningPlannerTest().testFixGrid();
    new CleaningPlannerTest().testPartitioning();
    new CleaningPlannerTest().testRepartitionOnObstacle();
    new CleaningPlannerTest().testGetStartingCoords();
    new CleaningPlannerTest().testSimulatedCleaning();
  }

  public void testFixGrid() {
    loadCrackedWorld();

    before();

    fixGridObstacles();

    after();
  }

  public void testPartitioning() {
    loadCrackedWorld();

    before();

    nextRectangle();

    after();
  }

  public void testRepartitionOnObstacle() {
    loadCrackedWorld();
    nextRectangle();

    before();

    handleObstacle(-8, 4);
    nextRectangle();

    after();
  }

  public void testGetStartingCoords() {
    loadCrackedWorld();

    before();

    int[] pos = getStartingCoords();

    after();
    printGrid(pos[0], pos[1]);

    LogUtils.println(pos[0] + " " + pos[1], LogUtils.ANSI_RED);
  }

  public void testSimulatedCleaning() {
    loadCrackedWorld();

    while (true) {
      CleanableRectangle rect = nextRectangle();
      printGrid();

      if (rect == null) {
        LogUtils.println("Done.");
        break;
      }

      for (int i = 0; i < rect.width; i++) {
        setDone(new MyRectangle(rect.x + i, rect.y, 1, rect.height));
        printGrid();
        Time.sleep(100);
      }

      printGrid();
    }
  }

  /**
   * A world with irregular obstacles and wall cracks.
   */
  private void loadCrackedWorld() {
    loadSimpleWorld();
    addObstacleSmallIrregular(-5, -4);
    addObstacleMediumIrregular(0, 0);
    addWallCracks();
  }

  /**
   * Loads a simple 20x20 world:
   * 
   * █████████████████
   * █···············█
   * █···············█
   * █···············█
   * █···············█
   * █···············█
   * █···············█
   * █···············█
   * █████████████████
   */
  private void loadSimpleWorld() {
    handleObstacle(-10, -10);
    handleObstacle(-10, 10);
    handleObstacle(10, 10);
    handleObstacle(10, -10);

    fillGrid(bounds.left, bounds.bottom, bounds.left, bounds.top, VALUE_OBSTACLE);
    fillGrid(bounds.left, bounds.bottom, bounds.right, bounds.bottom, VALUE_OBSTACLE);
    fillGrid(bounds.right, bounds.bottom, bounds.right, bounds.top, VALUE_OBSTACLE);
    fillGrid(bounds.left, bounds.top, bounds.right, bounds.top, VALUE_OBSTACLE);
  }

  /**
   * Adds an irregular obstacle to the grid:
   * 
   * ·█·
   * █·█
   * ·█·
   */
  private void addObstacleMediumIrregular(int x, int y) {
    fillGrid(x - 1, y, x - 1, y, VALUE_OBSTACLE);
    fillGrid(x + 1, y, x + 1, y, VALUE_OBSTACLE);
    fillGrid(x, 1, x, y + 1, VALUE_OBSTACLE);
    fillGrid(x, -1, x, y - 1, VALUE_OBSTACLE);
  }

  private void addObstacleSmallIrregular(int x, int y) {
    fillGrid(x - 1, y, x - 1, y, VALUE_OBSTACLE);
    fillGrid(x + 1, y, x + 1, y, VALUE_OBSTACLE);
  }

  private void addWallCracks() {
    grid.setLong(bounds.left, bounds.bottom + 4, VALUE_DIRTY);
    grid.setLong(bounds.left + 3, bounds.top, VALUE_DIRTY);
  }

  private void before() {
    LogUtils.println("\n\nBEFORE:\n");
    printGrid();
  }

  private void after() {
    LogUtils.println("\n\nAFTER:\n");
    printGrid();
  }
}