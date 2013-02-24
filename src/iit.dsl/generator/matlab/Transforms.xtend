package iit.dsl.generator.matlab


import iit.dsl.kinDsl.Robot

class Transforms {
    private iit.dsl.coord.generator.matlab.Generator matlabGen = null
    private iit.dsl.coord.coordTransDsl.Model transModel = null

    new(Robot robot, iit.dsl.coord.generator.MaximaConverter$IConfigurator maximaCfg) {
        transModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        matlabGen =  new iit.dsl.coord.generator.matlab.Generator()
        matlabGen.setMaximaReplSpecs(new MaximaReplSpecs(robot))
        matlabGen.setMaximaConverterConfigurator(maximaCfg)
    }

	def public homogeneous_init_fileContent(Robot robot) {
	    matlabGen.homogeneous_init_fileContent(transModel)
	}

	def public homogeneous_update_fileContent(Robot robot) {
        matlabGen.homogeneous_update_fileContent(transModel)
    }

    def public motion6D_init_fileContent(Robot robot) {
        matlabGen.motion6D_init_fileContent(transModel)
    }

    def public motion6D_update_fileContent(Robot robot) {
        matlabGen.motion6D_update_fileContent(transModel)
    }

    def public force6D_init_fileContent(Robot robot) {
        matlabGen.force6D_init_fileContent(transModel)
    }

    def public force6D_update_fileContent(Robot robot) {
        matlabGen.force6D_update_fileContent(transModel)
    }
}