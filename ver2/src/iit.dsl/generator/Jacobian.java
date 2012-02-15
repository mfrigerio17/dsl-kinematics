package iit.dsl.generator;

import iit.dsl.kinDsl.AbstractLink;
import iit.dsl.kinDsl.Joint;
import iit.dsl.kinDsl.RefFrame;
import iit.dsl.kinDsl.Robot;

import java.util.Iterator;
import java.util.List;

public class Jacobian {
    private final Robot robot;
    private final RefFrame baseFrame;
    private final RefFrame movingFrame;
    private final AbstractLink baseLink;
    private final AbstractLink movingLink;
    private final List<AbstractLink> linksChain;
    private final List<Joint> jointsChain;
    private final int cols;

    private final Common common;

    public Jacobian(final Robot robot, final iit.dsl.transspecs.transSpecs.FramePair jspec) {
        this.common = new Common();
        this.robot = robot;
        this.baseFrame   = common.getFrameByName(robot, jspec.getBase().getName());
        this.movingFrame = common.getFrameByName(robot, jspec.getTarget().getName());
        this.baseLink = common.getContainingLink(robot, baseFrame);
        this.movingLink = common.getContainingLink(robot, movingFrame);
        this.linksChain = common.buildChain(baseLink, movingLink);
        this.jointsChain = common.getChainJoints(linksChain);
        this.cols = jointsChain.size();
    }

    public Robot getRobot() {
        return robot;
    }

    public RefFrame getBaseFrame() {
        return baseFrame;
    }

    public RefFrame getMovingFrame() {
        return movingFrame;
    }

    public AbstractLink getBaseLink() {
        return baseLink;
    }

    public AbstractLink getMovingLink() {
        return movingLink;
    }

    public List<AbstractLink> getLinksChain() {
        return linksChain;
    }

    public int getCols() {
        return cols;
    }

    public List<Joint> getJointsChain() {
        return jointsChain;
    }

    /**
     * Creates a default name for this Jacobian */
    public String getName() {
        return baseFrame.getName() + "_J_" + movingFrame.getName();
    }

    /**
     * Returns a comma-separated list of the names of the variables of this Jacobian
     * @return
     */
    public String getArgsList() {
        if(jointsChain.size() == 0) return "";

        StringBuffer argsList = new StringBuffer();
        Iterator<Joint> iter = jointsChain.iterator();
        argsList.append( common.getVariableName( iter.next() ) );
        while( iter.hasNext() ) {
            argsList.append( ", " + common.getVariableName( iter.next() ) );
        }
        return argsList.toString();
    }
}
