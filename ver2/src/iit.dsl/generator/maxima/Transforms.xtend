package iit.dsl.generator.maxima

import iit.dsl.kinDsl.Robot

import org.eclipse.xtext.generator.IFileSystemAccess


class Transforms {
    private iit.dsl.coord.generator.CoordTransDslGenerator gen =
            new iit.dsl.coord.generator.CoordTransDslGenerator()

    def public generate(Robot robot, IFileSystemAccess fsa) {
        val iit.dsl.coord.coordTransDsl.Model transformsModel =
           iit::dsl::generator::common::Transforms::getTransformsModel(robot)

        fsa.generateFile(
            iit::dsl::coord::generator::Maxima::transformsFileName(transformsModel),
            gen.generateMaxima(transformsModel, new iit.dsl.coord.generator.Maxima())
        );
        fsa.generateFile(
            iit::dsl::coord::generator::Maxima::transforms6DFileName(transformsModel),
            gen.generateMaxima(transformsModel, new iit.dsl.coord.generator.Maxima6D())
        );
        fsa.generateFile(
            iit::dsl::coord::generator::Maxima::transforms6DForceFileName(transformsModel),
            gen.generateMaxima(transformsModel, new iit.dsl.coord.generator.Maxima6DForce())
        );
    }
}