package iit.dsl.generator.maxima

import iit.dsl.kinDsl.Robot

import org.eclipse.xtext.generator.IFileSystemAccess


class Transforms {

    /* This function completely relies on the code generation available in the
     * iit.dsl.coordTransDsl package.
     */
    def public generate(Robot robot, IFileSystemAccess fsa) {
        val iit.dsl.coord.coordTransDsl.Model transformsModel =
           iit::dsl::generator::common::Transforms::getTransformsModel(robot)

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