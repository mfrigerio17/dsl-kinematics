package iit.dsl.generator

import java.util.ArrayList
import java.util.List
import org.eclipse.xtext.EcoreUtil2
import org.eclipse.emf.ecore.util.EcoreUtil
import java.util.Locale

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
import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.impl.KinDslFactoryImpl
import iit.dsl.kinDsl.RotoTrasl
import iit.dsl.kinDsl.KinDslFactory
import iit.dsl.kinDsl.PILiteral
import iit.dsl.kinDsl.InertiaParams


class Common {
    private static Common instance = new Common()

    def public static Common getInstance() {
        return instance
    }
    static KinDslFactory kinDSLFactory = KinDslFactoryImpl::init()
    static FloatLiteral zeroFloat = kinDSLFactory.createFloatLiteral()//relies on default constructor setting it to zero


def dispatch boolean isFloating(RobotBase par) {
    false
}
def dispatch boolean isFloating(FloatingRobotBase par) {
    true
}

def List<AbstractLink> abstractLinks(Robot robot) {
    val list = new ArrayList<AbstractLink>()
    list.add(robot.base)
    list.addAll(robot.links)
    return list
}
// an alias of the above, as I don't like the name that much
def List<AbstractLink> linksAndBase(Robot robot) {
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

/**
 * The total number of degrees of freedom of this robot.
 * This functions considers the degrees of freedom due to the joints of the robot,
 * plus 6 additional dofs if the robot is a floating-base one.
 */
def int getDOFs(Robot robot) {
    if(robot.base.floating) {
        return getJointDOFs(robot) + 6
    } else {
        return getJointDOFs(robot)
    }
}

/**
 * The number of degrees of freedom of this robot due to its joints
 */
def int getJointDOFs(Robot robot) {
    return robot.joints.size
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
def dispatch int getID(Joint j) {
    //According to the convention, the integer identifier of a joint is the same
    // as the identifier of the link which is supported by that joint
    j.successorLink.ID;
}
def int getArrayIdx(Joint j) {
    j.ID - 1
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

/** Returns the joint that connects two links, the parent and the child */
def getJoint(AbstractLink parent, AbstractLink child) {
    if(parent == null  ||  child == null) return null
    for(ChildSpec c : parent.childrenList.children) {
        if(c.link.equals(child)) {
            return c.joint
        }
    }
    //should never get here
    throw new RuntimeException("Link " + parent.name  + " does not seem to be the parent of " + child.name)
}

def listCoordinates(Vector3 vector) {
    '''«vector.x.str» «vector.y.str» «vector.z.str»'''
}

def dispatch getTypeString(RevoluteJoint joint) '''revolute'''
def dispatch getTypeString(PrismaticJoint joint)'''prismatic'''

def dispatch boolean isPrismatic(RevoluteJoint joint) { return false }
def dispatch boolean isPrismatic(PrismaticJoint joint){ return true }

def boolean anyPrismaticJoint(Robot robot) {
    for(Joint j : robot.joints) {
        if(j.isPrismatic) return true;
    }
    return false;
}
def boolean anyRevoluteJoint(Robot robot) {
    for(Joint j : robot.joints) {
        if(j instanceof RevoluteJoint) return true;
    }
    return false;
}

def getVariableName(Joint joint) '''q_«joint.name»'''
def Joint getJointFromVariableName(Robot robot, String varname) {
    return getJointByName(robot, varname.replaceAll("q_", ""))
}
def getFrameName(Joint joint) '''fr_«joint.name»'''
def getFrameName(AbstractLink link) '''fr_«link.name»'''

def getItemNameByFrameName(String frameName) {
    if(! frameName.startsWith("fr_")) return null
    return frameName.replaceFirst("fr_", "")
}

def dispatch CharSequence str(Float num) {
    String::format(Locale::US,"%06.5f", num)
}
def dispatch CharSequence str(FloatLiteral id)'''«str(id.value)»'''
def dispatch CharSequence str(PlainExpr expr) '''«str(expr.identifier)»'''
def dispatch CharSequence str(MultExpr expr)  '''«str(expr.mult)» «str(expr.identifier)»'''
def dispatch CharSequence str(DivExpr expr)   '''«str(expr.identifier)»/«expr.div»'''
def dispatch CharSequence str(Identifier id)  '''«IF id.minus»-«ENDIF»«id.varname»'''


/**
 * Return the float value of an expression.
 * Will throw an exception if the expression is a variable literal, which of
 * course cannot be converted arbitrarily to a floating point value
 */
def dispatch float asFloat(FloatLiteral id) {
    id.value
}
def dispatch float asFloat(PlainExpr id) {
    id.identifier.asFloat
}
def dispatch float asFloat(MultExpr expr) {
    return Utilities::mult(expr.mult, expr.identifier.asFloat)
}
def dispatch float asFloat(DivExpr expr) {
    return Utilities::div(expr.identifier.asFloat, expr.div)
}
def dispatch float asFloat(PILiteral pi) {
    if(pi.minus) {
        return Utilities::invert(java::lang::Math::PI as float)
     }
     else {
         return java::lang::Math::PI as float
     }
}
//def dispatch asFloat(ConstantLiteral c) {
//    throw new RuntimeException("cannot convert to a float a variable literal")
//}





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

def RefFrame getFrameByName(Robot robot, String frameName) {
    for(AbstractLink l : robot.abstractLinks) {
        // The link does not have an actual RefFrame member to model its default frame,
        //  so if the name matches return a default token that represents it
        if(frameName.equals(l.frameName.toString())) {
            return l.defaultFrame
        }
        var frame = getFrameByName(l, frameName) // search the list of local frames on the link
        if(frame != null) {
            return frame
        }
    }
    //Search among the joint-frames. Same comment as above
    for(Joint j : robot.joints) {
        if(frameName.equals(j.frameName.toString())) {
            return j.defaultFrame
        }
    }

    // The reference frame was not found on the given robot
    return null
}

def Joint getJointByName(Robot robot, String jointName) {
    for(Joint j : robot.joints) {
        if(j.name.equals(jointName)) {
            return j
        }
    }
    return null
}
def Joint getJointByName(AbstractLink link, String jointName) {
    for(ChildSpec ch : link.childrenList.children) {
        if( ch.joint.name.equals(jointName) ) {
            return ch.joint
        }
    }
    return null
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
/**
 * Creates a RefFrame instance that represents the default reference frame of
 * the given joint. This method is required since in the model, joints do not
 * have an explicit property of type RefFrame (they only have rototranslation
 * parameters).
 */
def RefFrame getDefaultFrame(Joint j) {
    val RefFrame ret = kinDSLFactory.createRefFrame()
    ret.setTransform(EcoreUtil::copy(j.refFrame))
    ret.setName(j.frameName.toString())
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

/**
 * Returns the link of the given robot which supports the given frame.
 * A frame is supported by a link if its origin belongs to the link; this also
 * includes the frame of the joint which is supported by the link.
 */
def AbstractLink getContainingLink(Robot robot, RefFrame frame) {
    for(l : robot.abstractLinks) {
        if(l.frames.contains(frame)) return l
        if(frame.name.equals(l.frameName.toString())) return l
        for(ChildSpec child : l.childrenList.children) {
            if(frame.name.equals(child.joint.frameName.toString())) {
                return l
            }
        }
    }
    return null
}

def List<Joint> getChainJoints(List<AbstractLink> kinChain) {
    if(kinChain == null) return null
    if(kinChain.size() == 1) return new ArrayList<Joint>() // empty list

    val ret = new ArrayList<Joint>()
    val it1 = kinChain.iterator // points to first element
    val it2 = kinChain.listIterator(1) // points to second element
    var AbstractLink current
    var AbstractLink next
    while(it1.hasNext() && it2.hasNext()) {
        current = it1.next()
        next    = it2.next()
        if(next.equals(current.parent)) { // the i+1th element is the parent of the i-th
            ret.add(getJoint(next, current))
        } else if(current.equals(next.parent)) {// the other way round
            ret.add(getJoint(current, next))
        } else {
            throw new RuntimeException("Seems that the specified list is not a connected kinematic chain")
        }
    }
    return ret
}

def List<AbstractLink> chainEndLinks(Robot robot) {
    val ret = new ArrayList<AbstractLink>()
    for(l : robot.abstractLinks) {
        if(l.childrenList.children.empty) {
            ret.add(l)
        }
    }
    return ret;
}

/**
 * Returns the inertia parameters of the link in the argument, expressed
 * in the default frame of the link.
 */
def InertiaParams getLinkFrameInertiaParams(AbstractLink link) {
    // If no frame is specified, then the parameters are already expressed in the link frame
    if(link.inertiaParams.frame === null) return link.inertiaParams;

    val Vector3 trasl = link.inertiaParams.frame.transform.translation;
    val Vector3 rotat = link.inertiaParams.frame.transform.rotation;
    val InertiaParams params =
        Utilities::rototranslate(link.inertiaParams,
            trasl.x.asFloat, trasl.y.asFloat, trasl.z.asFloat,
            rotat.x.asFloat, rotat.y.asFloat, rotat.z.asFloat,
            //these numbers specify the pose of the frame in which the inertia-params
            // are currently expressed, with respect to the link frame. The link frame
            // is the one we want inertia to be expressed. According to docs, the next
            // argument must then be 'true'
            true)
    return params
}

}//end of class

