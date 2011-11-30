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


class Common {
    
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

/* Returns whether the ChildrenList contains the specified link */
def boolean contains(ChildrenList list, AbstractLink link) {
    for(ChildSpec c : list.children) {
        if(c.link.equals(link)) return true
    }
    return false
}
/* Returns whether the ChildrenList contains the specified joint */
def boolean contains(ChildrenList list, Joint joint) {
    for(ChildSpec c : list.children) {
        if(c.joint.equals(joint)) return true
    }
    return false
}
/* Returns the parent of the specified link, null if it does not exist */
def AbstractLink getParent(AbstractLink link) {
    val links = abstractLinks(link.eContainer() as Robot)
    for(AbstractLink l : links) {
        if(contains(l.childrenList, link)) return l
    }
   return null
}


}
