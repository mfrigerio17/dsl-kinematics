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
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.InertiaParams


class KinDslGenerator implements IGenerator {
    @Inject extension Common common
    @Inject FramesTransforms frTransforms

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robot.name+".urdf", generateURDFmodel(robot))
        fsa.generateFile(FramesTransforms::fileName(robot), frTransforms.coordinateTransformsDSLDocument(robot))
        fsa.generateFile(robot.name + ".m", featherstoneMatlabModel(robot))
    }

    /**
     * Generates and xml URDF description of the robot, as specified in the ROS documentation.
     * Leaves the inertia tensor as it is, according to the assumption that both the tensor
     * in the model and the one required by the URDF file format are expressed in the reference
     * frame with origin in the center of mass
     */
    def generateURDFmodel(Robot robot) '''
    <robot name="«robot.name»">
    «FOR link : robot.abstractLinks»
        «val inertia = link.inertiaParams»
        <link name="«link.name»">
            <inertial>
                <origin xyz="«inertia.com.x.str» «inertia.com.y.str» «inertia.com.z.str»"/>
                <mass value="«inertia.mass»"/>
                <inertia ixx="«inertia.ix»" iyy="«inertia.iy»" izz="«inertia.iz»" ixy="«inertia.ixy»" ixz="«inertia.ixz»" iyz="«inertia.iyz»"/>
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

    /**
     * This template generates a matlab file with the initialization of
     * a structure describing the robot kinematics, according to the model described
     * in "A beginner's guide to &-D vectors (part 2)" by Roy Featherstone.
     */
    def featherstoneMatlabModel(Robot robot) '''
        «val structName = robot.name.toLowerCase + "model"»
        «structName».robotname = '«robot.name»';
        «structName».NB = «robot.movingBodiesCount»;
        «structName».parent = zeros(1, «robot.links.size»);

        «IF robot.base.isFloating»
            «structName».parent(«robot.base.ID») = 0;
        «ENDIF»
        «FOR l : robot.links»
            «structName».parent(«l.ID») = «l.parent.ID»;
        «ENDFOR»

        «FOR j : robot.joints»
            «structName».pitch(«j.num») = «jointPitch(j)»;
        «ENDFOR»

        «FOR j : robot.joints»
            «structName».Xtree{«j.num»} = «jointTransform(j)»;
        «ENDFOR»

        «FOR l : robot.links»
            «structName».«inertiaParams(l)»
        «ENDFOR»
    '''


    def private dispatch jointPitch(RevoluteJoint j) '''0.0'''
    def private dispatch jointPitch(PrismaticJoint j) '''inf'''
    def private jointTransform(Joint j) '''
        Xrotz(«j.refFrame.rotation.z.asFloat» ) * ...
        Xroty(«j.refFrame.rotation.y.asFloat» ) * ...
        Xrotx(«j.refFrame.rotation.x.asFloat» ) * ...
        Xtrans([«j.refFrame.translation.listCoordinates()»]);
    '''

    def private inertiaParams(AbstractLink l) '''
        I{«l.ID»} = mcI(«l.inertiaParams.mass», [«l.inertiaParams.com.listCoordinates»], ...
        «inertiaTensor(l.inertiaParams)» );
    '''
    def private inertiaTensor(InertiaParams ip)  '''
        [ [ «ip.ix»    -(«ip.ixy») -(«ip.ixz»)]; ...
          [-(«ip.ixy»)   «ip.iy»   -(«ip.iyz»)]; ...
          [-(«ip.ixz») -(«ip.iyz»)   «ip.iz»] ] ...
    '''

}
