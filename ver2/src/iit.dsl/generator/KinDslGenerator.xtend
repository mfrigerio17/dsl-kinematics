/*
 * generated by Xtext
 */
package iit.dsl.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import org.eclipse.xtext.generator.IFileSystemAccess

import com.google.inject.Inject
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.InertiaParams
import iit.dsl.kinDsk.generator.Utilities


class KinDslGenerator implements IGenerator {
    @Inject extension Common common

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        common.init(robot)
        //fsa.generateFile(robot.name+".txt", test(robot))
        fsa.generateFile(robot.name+".urdf", generateURDFmodel(robot))
        //fsa.generateFile(robot.name+".temp", temp(robot))
    }

    def generateCode(Robot robot) '''
        «FOR link : robot.links»
        Link «link.name» connected from  «link.getParent().name»  via  «link.connectingJoint().name»
        «ENDFOR»
        «FOR joint : robot.joints»
        Joint «joint.name» connecting  «joint.successorLink().name»
        «ENDFOR»
    '''
//    «var InertiaParams inertia»
//    «var InertiaParams inertia_trans»
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
                <origin xyz="«inertia.com.items.get(0)» «inertia.com.items.get(1)» «inertia.com.items.get(2)»"/>
                <mass value="«inertia.mass»"/>
                <inertia ixx="«inertia_trans.ix»" iyy="«inertia_trans.iy»" izz="«inertia_trans.iz»" ixy="«inertia_trans.ixy»" ixz="«inertia_trans.ixz»" iyz="«inertia_trans.iyz»"/>
            </inertial>
        </link>
    «ENDFOR»
    «FOR joint : robot.joints»
        «val frame = joint.refFrame»
        <joint name="«joint.name»" type="«joint.type»"/>
            <origin xyz="«frame.translation.listCoordinates()»" rpy="«frame.rotation.listCoordinates()»"/>
            <parent link="«joint.predecessorLink.name»"/>
            <child  link="«joint.successorLink.name»"/>
        </joint>
    «ENDFOR»
    </robot>
    '''

    def test(Robot robot) '''
        «FOR link : robot.abstractLinks»
        Link «link.name» moved by  «link.connectingJoint»
        «ENDFOR»
        «FOR link : robot.links»
        Link «link.name» connected from  «link.parent.name»  via  «link.connectingJoint().name»
        «ENDFOR»
        «FOR joint : robot.joints»
        Joint «joint.name» connecting  «joint.successorLink().name»
        «ENDFOR»
    '''

    def temp(Robot robot) '''
    «val inertia = Utilities::translate(robot.links.get(1).inertiaParams,robot.links.get(1).inertiaParams.com)»
    «inertia.com»
    '''
}
