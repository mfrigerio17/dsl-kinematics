package iit.dsl.generator

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Link
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.RobotBase
import iit.dsl.kinDsl.FloatingRobotBase

import java.util.ArrayList
import java.util.List

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
    //child.name + "_X_" + child.getParent().name
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


}
