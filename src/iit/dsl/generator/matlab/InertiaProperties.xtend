package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Var
import iit.dsl.kinDsl.InertiaParams
import iit.dsl.kinDsl.AbstractLink

import iit.dsl.generator.Common
import iit.dsl.generator.Utilities

import java.util.List
import java.util.ArrayList


class InertiaProperties {
    public static CharSequence inertiaParamsStruct = '''massParams'''
    public static CharSequence functionName = '''inertiaProperties'''
    public static CharSequence fieldName_spatialInertia = '''tensor6D'''

    def public static fieldName(AbstractLink l) {
        return "lf_" + l.name
    }

    /**
     * The expression that should evaluate to the runtime value of the given
     * inertia property.
     *
     * Such expression is either just a floating point number, or a
     * field of the struct with the inertia parameters
     */
    def public static value(Var inertiaProperty) {
        return Parameters::value(inertiaProperty, inertiaParamsStruct)
    }

    /**
     * Matlab code that evaluates to the 3x3 tensor of the given inertia properties.
     */
    def public static CharSequence tensor(InertiaParams params) '''
        [[  «value(params.ix)»,«TAB»-(«value(params.ixy)»),«TAB»-(«value(params.ixz)»)];
         [-(«value(params.ixy)»),«TAB»  «value(params.iy)» ,«TAB»-(«value(params.iyz)»)];
         [-(«value(params.ixz)»),«TAB»-(«value(params.iyz)»),«TAB»  «value(params.iz)»]]'''

    /**
     * Matlab code that evaluates to the 3x1 vector with the COM position, as
     * contained in the given inertia properties.
     */
    def public static com(InertiaParams params) '''
         [«value(params.com.x)»; «value(params.com.y)»; «value(params.com.z)»]'''

    def scriptContent(Robot robot) {
        if(iit::dsl::generator::common::Parameters::massPropertiesAreParametric(robot)) {
            return scriptContent_withParams(robot)
        } else {
            return scriptContent_onlyConstants(robot)
        }
    }


    def private scriptContent_withParams(Robot robot) '''
        function out = «functionName»(«inertiaParamsStruct»)

        % Inertia properties as written in the .kindsl model file. These
        %  values are assumed to be expressed in the default frame of the
        %  corresponding link.
        % 'lf' stands for 'link frame'
        «FOR l : Common::getInstance().abstractLinks(robot)»
            «val in = l.inertiaParams»
            «val struct = "out." + fieldName(l)»
            «struct».mass = «value(in.mass)»;
            «struct».tensor = ...
                «tensor(in)»;
            «struct».com = «com(in)»;
            com = «struct».com;
            block = [  0,    -com(3),  com(2);
                     com(3),  0,      -com(1);
                    -com(2),  com(1),  0 ] * «struct».mass;
            «struct».«fieldName_spatialInertia» = [«struct».tensor, block; block', «struct».mass*eye(3)];


        «ENDFOR»
    '''

    /**
     * Generates Matlab code that creates several structs with the inertia parameters
     * of the links of the given robot.
     * Each struct has three members, the inertia tensor, the COM location (3d vector) and the mass.
     * This code contains the values corresponding to the inertia parameters as contained
     * in the robot model, as well as parameters expressed in the default frame of each link
     * and in a frame centered in the COM, aligned with the link-frame. Therefore, three
     * structures are created for each rigid body of the given robot.
     */
    def private scriptContent_onlyConstants(Robot robot) {
        val List<InertiaParams> userFrame = new ArrayList<InertiaParams>()
        val List<InertiaParams> linkFrame = new ArrayList<InertiaParams>()
        val List<InertiaParams> comFrame  = new ArrayList<InertiaParams>()
        allInertiaParams(robot, userFrame, linkFrame, comFrame)
        val userFrameIt = userFrame.iterator()
        val linkFrameIt = linkFrame.iterator()
        val comFrameIt  = comFrame.iterator()
        val allLinks = robot.abstractLinks
        val outVar = "out"

        return'''
        function «outVar» = «functionName»()

        % Inertia parameters as written in the .kindsl model file
        «FOR l : allLinks»
            «val tmp = userFrameIt.next»
            «val struct = outVar + "." + l.name»
            «struct».mass = «value(tmp.mass)»;
            «struct».tensor = ...
                «tensor(tmp)»;
            «struct».com = «com(tmp)»;

        «ENDFOR»

        % Now the same inertia parameters expressed in the link frame (may be equal or not to
        %  the previous ones, depending on the model file)
        «FOR l : allLinks»
            «val tmp = linkFrameIt.next»
            «val struct = outVar + "." + fieldName(l)»
            «struct».mass = «value(tmp.mass)»;
            «struct».tensor = ...
                «tensor(tmp)»;
            «struct».com = «com(tmp)»;
            com = «struct».com;
            block = [  0,    -com(3),  com(2);
                     com(3),  0,      -com(1);
                    -com(2),  com(1),  0 ] * «struct».mass;
            «struct».«fieldName_spatialInertia» = [«struct».tensor, block; block', «struct».mass*eye(3)];

        «ENDFOR»

        % Same inertial properties expressed in a frame with origin in the COM of the link
        %  oriented as the default link-frame (the COM coordinates in such a frame should
        %  always be [0,0,0] ).
        «FOR l : allLinks»
            «val tmp = comFrameIt.next»
            «val struct = outVar + ".com_" + l.name»
            «struct».mass = «value(tmp.mass)»;
            «struct».tensor = ...
                «tensor(tmp)»;
            «struct».com = «com(tmp)»;

        «ENDFOR»
    '''}

    /*
     * Fills three lists with the inertia parameters of all the links, expressed in
     * different frames.
     * First list: parameters as written in the robot model
     * Second list: parameters expressed in the default link-frame
     * Third list: parameters expressed in a frame with origin in the COM, aligned
     *             as the default link-frame
     * All the lists are ordered as robot.abstractLinks
     */
    def private allInertiaParams(Robot rob,
        List<InertiaParams> userFrame,
        List<InertiaParams> linkFrame,
        List<InertiaParams> comFrame)
    {
        // Start with the robot base
        var InertiaParams inLinkFrame = null
        var InertiaParams inComFrame  = null
        for(l : rob.abstractLinks) {
            inLinkFrame = l.linkFrameInertiaParams
            inComFrame  = Utilities::rototranslate(inLinkFrame, inLinkFrame.com.x.asFloat,
                inLinkFrame.com.y.asFloat, inLinkFrame.com.z.asFloat, 0,0,0, false)

            userFrame.add(l.inertiaParams)
            linkFrame.add(inLinkFrame)
            comFrame.add(inComFrame)
        }
    }


    private static String TAB = "\t"
    private extension Common common = Common::getInstance()
}