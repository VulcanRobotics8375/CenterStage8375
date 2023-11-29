package org.firstinspires.ftc.teamcode.robotcorelib.motion.path;

import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;

import java.util.ArrayList;
import java.util.HashMap;

public class Path {

    private final PathPoint start;
    private final ArrayList<PathPoint> guidePoints;
    private final PathPoint end;
    private boolean precise = true;

    private final HashMap<PathPoint, Runnable> runnableTasks;

    public Path(PathBuilder builder) {
        start = new PathPoint();
        start.setPathPoint(builder.getStartPoint());
        end = new PathPoint();
        end.setPathPoint(builder.getEndPoint());
        guidePoints = new ArrayList<>(builder.getGuidePoints());
        runnableTasks = new HashMap<>(builder.getTasks());
    }


    public Path(Path path) {
        this.start = new PathPoint();
        this.start.setPathPoint(path.start);
        this.guidePoints = new ArrayList<>(path.guidePoints);
        this.end = new PathPoint();
        end.setPathPoint(path.end);
        this.precise = path.precise;

        this.runnableTasks = path.runnableTasks;
    }

    public PathPoint getStart() {
        return start;
    }

    public ArrayList<PathPoint> getGuidePoints() {
        return guidePoints;
    }

    public PathPoint get(int index) {
        if(index == 0) {
            return start;
        } else if(index < guidePoints.size() + 1) {
           return guidePoints.get(index - 1);
        } else if(index == guidePoints.size() + 1) {
            return end;
        } else {
            return null;
        }
    }

    public PathPoint getEnd() {
        return end;
    }

    public HashMap<PathPoint, Runnable> getRunnableTasks() {
        return runnableTasks;
    }

    public ArrayList<PathPoint> asList() {
        ArrayList<PathPoint> pathPoints = new ArrayList<>(guidePoints);
        pathPoints.add(0, start);
        pathPoints.add(end);

        return pathPoints;
    }

    public void setPrecise(boolean precise) {
        this.precise = precise;
    }

    public boolean isPrecise() {
        return precise;
    }


}
