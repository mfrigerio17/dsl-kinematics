package iit.dsl.generator.matlab


import iit.dsl.kinDsl.Robot

class Transforms {
    private iit.dsl.coord.generator.matlab.Generator matlabGen = null
    private iit.dsl.coord.coordTransDsl.Model transModel = null

    new(Robot robot) {
        // Configure the Maxima converter that will be used by the generator
        iit::dsl::coord::generator::MaximaConverter::setGenMaximaCodeFolder(
            iit::dsl::generator::common::Transforms::getPath_transformsMaxima());
        transModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        matlabGen  =  new iit.dsl.coord.generator.matlab.Generator()
        matlabGen.setMaximaReplSpecs(new MaximaReplSpecs(robot))
    }

	def public homogeneous_init_fileContent(Robot robot) {
	    matlabGen.homogeneous_init_fileContent(transModel)
	}

	def public homogeneous_update_fileContent(Robot robot) {
        matlabGen.homogeneous_update_fileContent(transModel)
    }
}