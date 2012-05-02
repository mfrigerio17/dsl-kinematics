package iit.dsl.generator.matlab

import org.eclipse.xtext.generator.IFileSystemAccess
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IGenerator
import iit.dsl.kinDsl.Robot

class Generator implements IGenerator {

    override void doGenerate(Resource resource, IFileSystemAccess fsa) {
        val robot = resource.contents.head as Robot;
        fsa.generateFile(robot.name.toLowerCase() + "inertia", inertiaParams(robot))
    }

    def inertiaParams(Robot robot) '''
         «val bp = robot.base.inertiaParams»
        I_«robot.base.name» = [[ «bp.ix», -(«bp.ixy»), -(«bp.ixz»)];
                               [-(«bp.ixy»), «bp.iy» , -(«bp.iyz»)];
                               [-(«bp.ixz»),-(«bp.iyz»),  «bp.iz»]];
        «FOR l : robot.links»
            I_«l.name» = [[ «l.inertiaParams.ix», -(«l.inertiaParams.ixy»), -(«l.inertiaParams.ixz»)];
                          [-(«l.inertiaParams.ixy»), «l.inertiaParams.iy» , -(«l.inertiaParams.iyz»)];
                          [-(«l.inertiaParams.ixz»),-(«l.inertiaParams.iyz»),  («l.inertiaParams.iz»)]];
        «ENDFOR»
    '''
}