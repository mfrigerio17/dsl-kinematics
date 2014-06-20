package iit.dsl.generator.maxima

import org.eclipse.xtend2.lib.StringConcatenation


import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint

import iit.dsl.generator.Jacobian


class Jacobians {


    def static fileName(Robot robot) '''«robot.name»_jacobians'''

    /**
     * This function returns the CharSequence with the Maxima code that
     * should evaluate to the given Jacobian matrix.
     *
     * @param J the token representing the Jacobian of interest
     * @param transforms the model of the Transforms-DSL containing the
     *        coordinate transforms (ie the forward kinematics) required to
     *        compute the Jacobian
     */
    def jacobian(Jacobian J, iit.dsl.coord.coordTransDsl.Model transforms)
    {
        val baseFrameName = J.baseFrame.name;
        if(baseFrameName.equals(J.movingFrame.name)) {
            System::err.println("Skipping useless Jacobian " +
                baseFrameName + "_J_" + baseFrameName)
            //TODO log warning instead
            return '''''';
        }

        // The "longest" transform, between the base link and the moving link
        val base_X_ee = coordsUtils.getTransform(transforms, baseFrameName, J.movingFrame.name)
        if(base_X_ee == null) {
            throw new RuntimeException("Cannot generate the jacobian file: could not find the transform " +
                J.baseFrame.name + "_X_" + J.movingFrame.name + " (robot " + J.robot.name + ")");
        }
        var maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(base_X_ee)
        // Some variables of help for the code generation
        val variablesList = coordsUtils.argsListText(base_X_ee)
        val jacName   = '''«J.getName()»(«variablesList»)'''
        val eePosName = '''«J.movingFrame.name»_pos_wrt_«J.baseFrame.name»(«variablesList»)'''

        var iit.dsl.coord.coordTransDsl.Transform transform
        val StringConcatenation strBuff = new StringConcatenation();
        strBuff.append('''
            «eePosName» := posVector(«maximaTransformLiteral»);
            «jacName» := addcol(matrix()
                ''');

        for(Joint el : J.jointsChain) {
            val tmpFrameName = common.getFrameName(el).toString();
            var CharSequence maximaCodeLine

            // Check for the special case in which the frame of the first joint
            //  is also the base frame for the Jacobian. In this case the
            //  parameters required by GJacobianColumn are trivial
            if(baseFrameName.equals(tmpFrameName)) {

                if(el instanceof RevoluteJoint) {
                    maximaCodeLine = '''GJacobianColumn(«eePosName», matrix([0],[0],[1]), matrix([0],[0],[0]) )'''
                } else if (el instanceof PrismaticJoint) {
                    maximaCodeLine = '''GJacobianColumn_prism(matrix([0],[0],[1])'''
                } else {
                    throw new RuntimeException("Unknown joint type")
                }

            } else {

                // Get the transform 'base_X_<current link>', which will be
                //  referenced in the generated code to retrieve the joint
                //  position and joint axis required to compute the Jacobian column
                transform = coordsUtils.getTransform(transforms, baseFrameName, tmpFrameName)
                if(transform == null) {
                    throw new RuntimeException("Cannot generate the jacobian file: could not find the transform " +
                        baseFrameName + "_X_" + tmpFrameName + " (robot " + J.robot.name + ")");
                }
                maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(transform)

                if(el instanceof RevoluteJoint) {
                    maximaCodeLine = '''GJacobianColumn(«eePosName», zaxis(«maximaTransformLiteral»), posVector(«maximaTransformLiteral»))'''
                } else if (el instanceof PrismaticJoint) {
                    maximaCodeLine = '''GJacobianColumn_prism(zaxis(«maximaTransformLiteral»))'''
                } else {
                    throw new RuntimeException("Unknown joint type")
                }

            }
            strBuff.append("\t, ");
            strBuff.append(maximaCodeLine);
            strBuff.append("\n");

        }
        strBuff.append(''');''');
        strBuff.append("\n");

        return strBuff
    }


    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    private iit.dsl.coord.generator.maxima.Maxima coordsMaxima = new iit.dsl.coord.generator.maxima.Maxima()
    private iit.dsl.coord.generator.Common coordsUtils = new iit.dsl.coord.generator.Common()
}
