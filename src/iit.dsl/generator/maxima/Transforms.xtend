package iit.dsl.generator.maxima

import iit.dsl.kinDsl.Robot

import org.eclipse.xtext.generator.IFileSystemAccess


class Transforms {

    /* The generator functions of this file completely rely on the code
     * generation available in the iit.dsl.coordTransDsl package
     * (the Transforms-DSL project).
     */
    def public generate(
        Robot robot,
        IFileSystemAccess fsa,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        fsa.generateFile(
            iit::dsl::coord::generator::maxima::Maxima::transformsFileName(transformsModel),
            iit::dsl::coord::generator::maxima::Maxima::sourceCode(transformsModel,
                new iit.dsl.coord.generator.maxima.Maxima()
            )
        );
        fsa.generateFile(
            iit::dsl::coord::generator::maxima::Maxima::transforms6DFileName(transformsModel),
            iit::dsl::coord::generator::maxima::Maxima::sourceCode(transformsModel,
                new iit.dsl.coord.generator.maxima.Maxima6D()
            )
        );
        fsa.generateFile(
            iit::dsl::coord::generator::maxima::Maxima::transforms6DForceFileName(transformsModel),
            iit::dsl::coord::generator::maxima::Maxima::sourceCode(transformsModel,
                new iit.dsl.coord.generator.maxima.Maxima6DForce()
            )
        );
    }

}