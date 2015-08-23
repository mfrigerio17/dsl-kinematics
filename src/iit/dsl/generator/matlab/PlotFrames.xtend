package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Common
import iit.dsl.kinDsl.Link
import iit.dsl.generator.Utilities


class PlotFrames {

    public static CharSequence functionName = '''plotFrames'''

    extension Common common = new Common()
    private Robot robot
    private iit.dsl.coord.coordTransDsl.Model transforms
    private iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()

    new(Robot rob, iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        robot = rob
        transforms = transformsModel
    }

	def plotFramesCode() '''
        function «functionName»(xh)

        baseColor = [0,0,1];
        ellips.faceColor = [0.94, 0.94, 0.94];
        ellips.edgeColor = [0.7 , 0.9, 1];
        ellips.faceAlpha = 0.4;
        ellips.edgeAlpha = 0.6;

        % Plot the reference frame of the base.
        % Use arbitrary scaling.
        scaling = 0.1;
        x = scaling * [1;0;0];
        y = scaling * [0;1;0];

        plotRefFrame(x, y, [0;0;0], baseColor);

        % Now plot the reference frames of the other links
        % Scaling factor of 33% of the estimate of the link length

        H = eye(4);
        «val count = robot.links.size + 1»
        «FOR l : robot.links»
            %%% Link «l.name» %%%
            «val T = coordTransCommon.getTransform(transforms, l.parent.frameName.toString(), l.frameName.toString())»
            «IF T == null»
                % I could not find the transform from link «l.name» to its parent; are you generating it?
            «ELSE»
                «val tname = iit::dsl::coord::generator::matlab::Generator::identifier(T,
                    iit::dsl::coord::generator::Utilities$MatrixType::HOMOGENEOUS)»
                «val jointDist = jointDistance(l)»
                «IF l.childrenList.children.size > 0»
                    scaling = 0.33 * «jointDist»;
                «ENDIF»
                H = H * xh.«tname»;
                x = scaling * H(1:3,1);
                y = scaling * H(1:3,2);
                pos = H(1:3,4);  % no scaling

                plotRefFrame(x, y, pos, baseColor * («count»-«l.ID»)/«count»);

                %% [X,Y,Z] = inertia_ellipsoid(inertia_lf_«l.name», H, «jointDist»);
                %% h_ell = surf(X,Y,Z);
                %% set(h_ell, 'FaceAlpha', ellips.faceAlpha, 'EdgeAlpha', ellips.edgeAlpha);
                %% set(h_ell, 'FaceColor', ellips.faceColor, 'EdgeColor', ellips.edgeColor);
            «ENDIF»


        «ENDFOR»

        axis equal;
        '''

     def private double jointDistance(Link link) {
         if(link.childrenList.children.size() == 0) {
             return 1 // default??
         } else {
             val j = link.childrenList.children.get(0).joint
             return Utilities::length(j.refFrame.translation)
         }
     }

}