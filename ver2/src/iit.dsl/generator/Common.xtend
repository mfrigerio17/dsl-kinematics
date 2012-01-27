package iit.dsl.generator

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.kinDsl.ChildrenList
import iit.dsl.kinDsl.DivExpr
import iit.dsl.kinDsl.Expr
import iit.dsl.kinDsl.FixedRobotBase
import iit.dsl.kinDsl.FloatLiteral
import iit.dsl.kinDsl.FloatingRobotBase
import iit.dsl.kinDsl.Identifier
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.Link
import iit.dsl.kinDsl.MultExpr
import iit.dsl.kinDsl.PlainExpr
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.RobotBase
import iit.dsl.kinDsl.Vector3
import java.util.ArrayList
import java.util.List
import org.eclipse.xtext.EcoreUtil2
import java.util.Locale
import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.impl.KinDslFactoryImpl
import iit.dsl.kinDsl.RotoTrasl
import iit.dsl.kinDsl.KinDslFactory
import org.eclipse.emf.ecore.util.EcoreUtil


class Common {

    static KinDslFactory kinDSLFactory = KinDslFactoryImpl::init()
    static FloatLiteral zeroFloat = kinDSLFactory.createFloatLiteral()//relies on default constructor setting it to zero

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
    val links = abstractLinks(link.eContainer() as Robot)
    for(AbstractLink l : links) {
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

def dispatch int getID(Link l) {
    return l.num;
}
def dispatch int getID(FixedRobotBase base) {
    return 0;
}
def dispatch int getID(FloatingRobotBase base) {
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
    val links = abstractLinks(joint.eContainer() as Robot)
    var AbstractLink found = null
    for(AbstractLink l : links) {
        found = getSuccessor(l.childrenList, joint);
        if(found != null) return found
    }
    //should never get here
    throw new RuntimeException("Joint " + joint.name + " has no successor?!?")
}
/** Returns the predecessor link of a joint */
def AbstractLink getPredecessorLink(Joint joint) {
    val links = abstractLinks(joint.eContainer() as Robot)
    for(AbstractLink l : links) {
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
    '''«vector.x.str» «vector.y.str» «vector.z.str»'''
}

def dispatch getTypeString(RevoluteJoint joint) '''revolute'''
def dispatch getTypeString(PrismaticJoint joint)'''prismatic'''

def getVariableName(Joint joint) '''q_«joint.name»'''
def getFrameName(Joint joint) '''fr_«joint.name»'''
def getFrameName(AbstractLink link) '''fr_«link.name»'''

def str(Float num) {
    String::format(Locale::US,"% 06.5f", num)
}

def dispatch str(FloatLiteral id)'''«str(id.value)»'''
def dispatch str(PlainExpr expr) '''«str(expr.identifier)»'''
def dispatch str(MultExpr expr)  '''«str(expr.mult)» «str(expr.identifier)»'''
def dispatch str(DivExpr expr)   '''«str(expr.identifier)»/«expr.div»'''
def str(Identifier id)           '''«IF id.minus»-«ENDIF»«id.varname»'''

def dispatch FloatLiteral invert(FloatLiteral f) {
    var FloatLiteral newf =  EcoreUtil2::copy(f)
    newf.value = Utilities::invert(f.value)
    return newf
}
def dispatch Expr invert(Expr expr) {
    var Expr newExpr = EcoreUtil2::copy(expr)
    newExpr.identifier.minus = !expr.identifier.minus //the actual inversion
    return newExpr
}

def dispatch boolean isZero(FloatLiteral f) {
    return Utilities::isZero(f.value)
}
def dispatch boolean isZero(Expr expr) {
    return false
}

def boolean isDescendant(AbstractLink candidate, AbstractLink start) {
    if(start.childrenList.contains(candidate)) {
        return true
    }
    for(ChildSpec child : start.childrenList.children) {
        if(candidate.isDescendant(child.link)) {
            return true
        }
    }
    return false
}

def AbstractLink commonAncestor(AbstractLink l1, AbstractLink l2) {
    if(l1.equals(l2)) return l1;
    if(l1.isDescendant(l2)) return l2;
    if(l2.isDescendant(l1)) return l1;
    var AbstractLink child1  = l1
    var AbstractLink child2  = l2
    var AbstractLink parent1 = l1.parent
    var AbstractLink parent2 = l2.parent
    while(parent1 != null && parent2 != null) {
        if(parent1.equals(parent2)) return parent1;
        // follow the branches up in the hierarchy
        child1  = parent1
        child2  = parent2
        parent1 = parent1.parent
        parent2 = parent2.parent
    }
    while(parent1 != null) {
        if(parent1.equals(child2)) return parent1;
        // follow the branch 1 up in the hierarchy
        child1  = parent1
        parent1 = parent1.parent
    }
    while(parent2 != null) {
        if(parent2.equals(child1)) return parent2;
        // follow the branch 2 up in the hierarchy
        child2  = parent2
        parent2 = parent2.parent
    }
    //should never get here
    throw(new RuntimeException("looks like these two links do not have a common ancestor!!"))
}


def AbstractLink getLinkByName(Robot robot, String linkName) {
    for(AbstractLink l : robot.abstractLinks) {
        if(l.name.equals(linkName)) {
            return l
        }
    }
    return null
}

def RefFrame getFrameByName(AbstractLink link, String frameName) {
    for(RefFrame f : link.frames) {
        if(f.name.equals(frameName)) {
            return f
        }
    }
    return null
}

def List<AbstractLink> buildChain(AbstractLink first, AbstractLink last) {
    val List<AbstractLink> chain = new ArrayList<AbstractLink>()
    if(first.equals(last)) {
        chain.add(first)
        return chain
    }
    val AbstractLink ancestor = commonAncestor(first, last);
    var AbstractLink parent

    if(last.equals(ancestor)) {
        chain.add(first)
        parent = first.parent
        while(! parent.equals(last)){
            chain.add(parent)
            parent = parent.parent
        }
        chain.add(last)
        return chain
    }
    if(first.equals(ancestor)) {
        chain.add(last)
        parent = last.parent
        while(! parent.equals(first)) {
            chain.add(parent)
            parent = parent.parent
        }
        chain.add(first)
        return chain.reverse
    }

    // The two links belong to different branches starting from the common ancestor
    val List<AbstractLink> head = buildChain(first, ancestor);
    val List<AbstractLink> tail = buildChain(ancestor, last);

    head.addAll( tail.drop(1) ) //drop the first because it is the second copy of the ancestor
    return head
}

/**
 * The default frame of a link is simply a named frame whose transform should be
 * the identity
 */
def RefFrame getDefaultFrame(AbstractLink link) {
    val RefFrame ret = createDefaultFrame()
    ret.setName(link.frameName.toString())

    return ret
}

def createDefaultFrame() {
    val RefFrame ret = kinDSLFactory.createRefFrame()
    val RotoTrasl roto = kinDSLFactory.createRotoTrasl()
    // default transformation is the identity
    roto.setTranslation(zeroVector())
    roto.setRotation(zeroVector())
    ret.setTransform(roto)
    ret.name = ""
    return ret
}

def Vector3 zeroVector() {
    val ret = kinDSLFactory.createVector3()
    ret.setX(EcoreUtil::copy(zeroFloat))
    ret.setY(EcoreUtil::copy(zeroFloat))
    ret.setZ(EcoreUtil::copy(zeroFloat))
    return ret
}

def AbstractLink getContainingLink(Robot robot, RefFrame frame) {
    for(l : robot.abstractLinks) {
        if(l.frames.contains(frame)) return l
        if(frame.name.equals(l.frameName.toString())) return l
    }
    return null
}

}//end of class

