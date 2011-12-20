package iit.dsl.generator

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Link
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.RobotBase
import iit.dsl.kinDsl.FloatingRobotBase
import iit.dsl.kinDsl.ChildrenList

import java.util.ArrayList
import java.util.List
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.kinDsl.FixedRobotBase
import iit.dsl.kinDsl.Vector3
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint


class Common {

List<AbstractLink> allLinks

def void init(Robot robot) {
    allLinks = abstractLinks(robot)
}

def inertiaMxName(Link link) {
    link.name + "_Imx"
}

def coordTransformName(Joint j) {
    j.name + "_X"
}

def subspaceMxName(Joint par) {
    par.name + "_S"
}

def childToParentMxName(Link child) {
    child.name + "_X_" + child.getParent().name
}

def velocityName(AbstractLink par) {
    par.name + "_v"
}
def accelerationName(AbstractLink par) {
    par.name + "_a"
}
def forceName(AbstractLink par) {
    par.name + "_f"
}

def Boolean isFloating(RobotBase par) {
    false
}
def Boolean isFloating(FloatingRobotBase par) {
    true
}

def List<AbstractLink> abstractLinks(Robot robot) {
    val list = new ArrayList<AbstractLink>()
    list.add(robot.base)
    list.addAll(robot.links)
    return list
}

/** Returns whether the ChildrenList contains the specified link */
def boolean contains(ChildrenList list, AbstractLink link) {
    for(ChildSpec c : list.children) {
        if(c.link.equals(link)) return true
    }
    return false
}
/** Returns whether the ChildrenList contains the specified joint */
def boolean contains(ChildrenList list, Joint joint) {
    for(ChildSpec c : list.children) {
        if(c.joint.equals(joint)) return true
    }
    return false
}
/** Returns the parent of the specified link, null if it does not exist */
def AbstractLink getParent(AbstractLink link) {
    for(AbstractLink l : allLinks) {
        if(contains(l.childrenList, link)) return l
    }
   return null
}

def int movingBodiesCount(Robot robot) {
    if(robot.base.isFloating()) {
        return robot.links.size + 1;
    } else {
        return robot.links.size
    }
}

def int getID(Link l) {
    return l.num;
}
def int getID(FixedRobotBase base) {
    return 0;
}
def int getID(FloatingRobotBase base) {
    return 1;
}
def int getID(Joint j) {
    j.num;
}

/**
 * Returns the successor link of the joint, searching in the given
 * children list. Null, if it does not find it.
 */
def AbstractLink getSuccessor(ChildrenList list, Joint joint) {
    for(ChildSpec c : list.children) {
        if(c.joint.equals(joint)) {
            return c.link
        }
    }
    return null
}
/** Returns the successor link of a joint */
def AbstractLink getSuccessorLink(Joint joint) {
    var AbstractLink found = null
    for(AbstractLink l : allLinks) {
        found = getSuccessor(l.childrenList, joint);
        if(found != null) return found
    }
    //should never get here
    throw new RuntimeException("Joint " + joint.name + " has no successor?!?")
}
/** Returns the predecessor link of a joint */
def AbstractLink getPredecessorLink(Joint joint) {
    for(AbstractLink l : allLinks) {
        if(contains(l.childrenList, joint)){
            return l
        }
    }
    //should never get here
    throw new RuntimeException("Joint " + joint.name + " has no predecessor?!?")
}

/** Returns the Joints which "moves" the link in the argument */
def dispatch Joint getConnectingJoint(RobotBase base) {//specialized funtion for RobotBase
    if(base.floating) {
        throw new RuntimeException("The floating base 6DOF virtual joint is not yet implemented, cannot return it")
    } else {
        return null
    }
}
def dispatch Joint getConnectingJoint(AbstractLink link) {
    for(ChildSpec c : link.getParent().childrenList.children) {
        if(c.link.equals(link)) {
            return c.joint
        }
    }
    //should never get here
    throw new RuntimeException("Link " + link.name + " is not connected via any joint?!?")
}

def listCoordinates(Vector3 vector) {
    '''«vector.items.get(0)» «vector.items.get(1)» «vector.items.get(2)»'''
}

def dispatch getType(RevoluteJoint joint) {
    '''revolute'''
}
def dispatch getType(PrismaticJoint joint) {
    '''revolute'''
}

def getVariableName(Joint joint) '''q_«joint.name»'''
def getFrameName(Joint joint) '''fr_«joint.name»'''
def getFrameName(AbstractLink link) '''fr_«link.name»'''

def dispatch motionTransform(RevoluteJoint  joint)
    '''Rz(«joint.variableName»)'''
def dispatch motionTransform(PrismaticJoint joint)
    '''Tz(«joint.variableName»)'''

def CharSequence link2jointTransform(Joint joint) {
    var StringBuffer text = new StringBuffer();
    text.append('''Tx(«joint.refFrame.translation.items.get(0)») Ty(«joint.refFrame.translation.items.get(1)») Tz(«joint.refFrame.translation.items.get(2)») Rx(«joint.refFrame.rotation.items.get(0)») Ry(«joint.refFrame.rotation.items.get(1)») Rz(«joint.refFrame.rotation.items.get(2)»)''')

    return text
}



}//end of class
