package iit.dsl.generator.matlab


import iit.dsl.kinDsl.Robot
import iit.dsl.generator.matlab.config.IConfigurator

class Transforms
{
    private iit.dsl.coord.generator.matlab.Generator matlabGen = null
    private iit.dsl.coord.coordTransDsl.Model       transModel = null

    new(Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel,
        IConfigurator configurator)
    {
        transModel = transformsModel
        matlabGen  =  new iit.dsl.coord.generator.matlab.Generator(configurator)
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