/*
 * generated by Xtext
 */
package iit.dsl.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink


class KinDslGenerator implements IGenerator {
    @Inject extension Common common

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        //fsa.generateFile(robot.name+".temp", temp(robot))
        //fsa.generateFile(robot.name+".urdf", generateURDFmodel(robot))
        fsa.generateFile(robot.name+".ctdsl", generateCoordinateTransforms(robot))
        //test(robot)
    }

    /**
     * Generates and xml URDF description of the robot, as specified in the ROS documentation
     */
    def generateURDFmodel(Robot robot) '''
    <robot name="«robot.name»">
    «FOR link : robot.links»
        «val inertia = link.inertiaParams»
        «val inertia_trans = Utilities::translate(inertia, inertia.com)»
        <link name="«link.name»">
            <inertial>
                <origin xyz="«inertia.com.x.str» «inertia.com.y.str» «inertia.com.z.str»"/>
                <mass value="«inertia.mass»"/>
                <inertia ixx="«inertia_trans.ix»" iyy="«inertia_trans.iy»" izz="«inertia_trans.iz»" ixy="«inertia_trans.ixy»" ixz="«inertia_trans.ixz»" iyz="«inertia_trans.iyz»"/>
            </inertial>
        </link>
    «ENDFOR»
    «FOR joint : robot.joints»
        «val frame = joint.refFrame»
        <joint name="«joint.name»" type="«joint.typeString»"/>
            <origin xyz="«frame.translation.listCoordinates()»" rpy="«frame.rotation.listCoordinates()»"/>
            <parent link="«joint.predecessorLink.name»"/>
            <child  link="«joint.successorLink.name»"/>
        </joint>
    «ENDFOR»
    </robot>
    '''

    def test(Robot robot) {
        var AbstractLink found;
        for(AbstractLink l: robot.abstractLinks) {
            found = common.getLinkByName(robot, l.name)
            if(! found.equals(l)) {
                throw new RuntimeException("test failed!")
            }
        }
        //throw new RuntimeException("OK!!")
    }
    /*
    '''
        «FOR link : robot.abstractLinks»
        Link «link.name» moved by  «link.connectingJoint»
        «ENDFOR»
        «FOR link : robot.links»
        Link «link.name» connected from  «link.parent.name»  via  «link.connectingJoint.name»
        «ENDFOR»
        «FOR joint : robot.joints»
        Joint «joint.name» connecting  «joint.successorLink.name»
        «ENDFOR»
    '''
    //*/
    /*
    '''
        «FOR link : robot.links»
        Link «link.name» connected from  «link.parent.name»  via  «link.connectingJoint.name»
        «ENDFOR»
        «FOR joint : robot.joints»
        Joint «joint.name» connecting  «joint.successorLink.name»
        «ENDFOR»
    '''
    //*/



//    def temp(Robot robot) '''
//    «val inertia = Utilities::translate(robot.links.get(1).inertiaParams,robot.links.get(1).inertiaParams.com)»
//    «inertia.com»
//    '''
    def temp(Robot robot) {
        val StringBuilder builder = new StringBuilder()
        val links = robot.abstractLinks
        for(AbstractLink l1 : links) {
            for(AbstractLink l2 : links.reverseView) {
                //builder.append(
                //'''«l1.name» - «l2.name»   Ancestor: «commonAncestor(l1,l2).name»
                //''')
                builder.append('''«l1.name» - «l2.name»  :  «FramesTransforms::dest_X_source(l1,l2)»
                ''')
            }
        }
        return builder
    }

    def generateCoordinateTransforms(Robot robot) '''
    Model «robot.name»
    Frames {
        «robot.base.frameName»
        «FOR link : robot.links»
            , «link.frameName»
        «ENDFOR»
        «FOR joint : robot.joints»
            , «joint.frameName»
        «ENDFOR»
    }

    TransformedFramePos = right

    «FOR link : robot.links»
        «val joint  = link.connectingJoint»
        «val parent = link.parent»
        {«parent.frameName»}_X_{«link.frameName»} = «FramesTransforms::predecessor_X_successor(joint)»
        {«link.frameName»}_X_{«parent.frameName»} = «FramesTransforms::successor_X_predecessor(joint)»

    «ENDFOR»
    '''
}
