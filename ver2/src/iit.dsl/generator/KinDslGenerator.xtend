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
import iit.dsl.kinDsl.RefFrame


class KinDslGenerator implements IGenerator {
    @Inject extension Common common
    @Inject FramesTransforms frTransforms

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        //fsa.generateFile(robot.name+".temp", test(robot))
        fsa.generateFile(robot.name+".urdf", generateURDFmodel(robot))
        //fsa.generateFile(robot.name+".ctdsl", frTransforms.coordinateTransformsDSLDocument(robot))
        //fsa.generateFile("blabla", temp(resource))
        //test_getJoint(robot)
    }

    /**
     * Generates and xml URDF description of the robot, as specified in the ROS documentation
     */
    def generateURDFmodel(Robot robot) '''
    <robot name="«robot.name»">
    «FOR link : robot.abstractLinks»
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
        <joint name="«joint.name»" type="«joint.typeString»">
            <origin xyz="«frame.translation.listCoordinates()»" rpy="«frame.rotation.x.asFloat» «frame.rotation.y.asFloat» «frame.rotation.z.asFloat»"/>
            <parent link="«joint.predecessorLink.name»"/>
            <child  link="«joint.successorLink.name»"/>
            <limit effort="30" velocity="1.0"/>
            <axis xyz="0 0 1"/>
        </joint>
    «ENDFOR»
    </robot>
    '''

    def test_getJoint(Robot robot) {
        System::out.println('''
        «FOR link : robot.links»
        Parent «link.parent.name», child: «link.name»  -  Joint: «getJoint(link.parent, link).name»
        «ENDFOR»'''.toString() );
    }

    def test_jointFromVariableName(Robot robot) {
        System::out.println('''
        «FOR joint : robot.joints»
        Joint «joint.name», variable: «joint.variableName», joint again: «(robot.getJointFromVariableName(joint.variableName.toString())).name»
        «ENDFOR»'''.toString() );
    }
    /*
    '''
        «FOR link : robot.abstractLinks»
            «FOR link2 : robot.abstractLinks»
                «val chain = common.buildChain(link, link2)»
                «link.name» - «link2.name»   :   «FOR AbstractLink el : chain» «el.name»«ENDFOR»
            «ENDFOR»
        «ENDFOR»

    '''
    //*/
    /*
    var AbstractLink found;
        for(AbstractLink l: robot.abstractLinks) {
            found = common.getLinkByName(robot, l.name)
            if(! found.equals(l)) {
                throw new RuntimeException("test failed!")
            }
        }
    //throw new RuntimeException("OK!!")
    //*/
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


    def temp(Resource resource) '''
        «FOR r : resource.contents»
        «r.eClass.name»
        «ENDFOR»'''

}
