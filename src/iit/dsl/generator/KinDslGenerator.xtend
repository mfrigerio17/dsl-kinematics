package iit.dsl.generator

//import iit.dsl.kinDsl.Robot

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGenerator2
import org.eclipse.xtext.generator.IGeneratorContext


class KinDslGenerator implements IGenerator2
{
    override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context)
    {
        //val robot = resource.contents.head as Robot;
    }

    override afterGenerate(Resource input, IFileSystemAccess2 fsa, IGeneratorContext context) {
    }
    override beforeGenerate(Resource input, IFileSystemAccess2 fsa, IGeneratorContext context) {
    }



}
