package iit.dsl.generator.sl.robot.math

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.kinDsl.PILiteral
import iit.dsl.kinDsl.FloatLiteral
import iit.dsl.kinDsl.PlainExpr
import iit.dsl.kinDsl.MultExpr
import iit.dsl.kinDsl.DivExpr
import iit.dsl.kinDsl.ParameterLiteral
import iit.dsl.kinDsl.Joint

class Mathematica {

    def public notebook(Robot robot, String slRootPath) '''
        «val robotName = robot.name»
        (* Beginning of Notebook Content *)
        Notebook[{
        Cell[BoxData[
         RowBox[{
          RowBox[{
          "SetDirectory", "[", "\"\<«slRootPath»/RigidBodyDynamics/\>\"", "]"}],
          ";"}]], "Input",
         CellChangeTimes->{{3.4691655408842077`*^9, 3.4691655651588182`*^9}, {
          3.469166315104549*^9, 3.4691663196428537`*^9}, {3.499409252158552*^9,
          3.499409256220811*^9}, {3.525146342503011*^9, 3.52514635857464*^9}, {
          3.525152703171839*^9, 3.52515271342125*^9}}],

        Cell[BoxData[
         RowBox[{"<<", "RigidBodyDynamics.m"}]], "Input",
         CellChangeTimes->{{3.499409280874213*^9, 3.499409284632689*^9},
           3.4994102006706963`*^9, {3.4994103390985413`*^9, 3.499410339567994*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{
          RowBox[{"SetDirectory", "[", "\"\<«slRootPath»/«robot.robotFolderName»/math\>\"", "]"}],
          ";"}]], "Input",
         CellChangeTimes->{
          3.469165616187872*^9, 3.487616125166923*^9, 3.499410202079053*^9, {
           3.52514637103411*^9, 3.525146374304959*^9}}],

        Cell["", "Text",
         CellChangeTimes->{{3.487621177793989*^9, 3.487621180100277*^9}}],

        Cell[BoxData[
         RowBox[{"OpenGLKinematics", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582726736*^9, 3.4691656162179947`*^9, {3.525146378593598*^9,
           3.525146383686726*^9}}],

        Cell[BoxData[
         RowBox[{"InvDynNE", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.4691655827608423`*^9, 3.4691656162535877`*^9, {3.499409294531172*^9,
           3.4994092946474867`*^9}, {3.525146389247444*^9, 3.525146392188701*^9},
           3.525151813183488*^9, {3.525152860858181*^9, 3.525152863572134*^9}, {
           3.525155668635*^9, 3.525155669011446*^9}, {3.525155860832596*^9,
           3.525155863184689*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{"InvDynArt", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.4691655827923203`*^9, 3.469165616284869*^9, {3.525146395907315*^9,
           3.525146401733008*^9}}],

        Cell[BoxData[
         RowBox[{"LinkEndpointKinematics", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582826668*^9, 3.469165616318798*^9, {3.525146404960537*^9,
           3.525146410197776*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{"GeometricJacobian", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",",
           RowBox[{"{", "104", "}"}], ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582859626*^9, 3.469165616352508*^9, {3.487618257581111*^9,
           3.48761825911502*^9}, {3.499409314363538*^9, 3.499409332230188*^9}, {
           3.499410238750249*^9, 3.499410249789199*^9}, 3.499410607628263*^9, {
           3.525146413537066*^9, 3.525146416492277*^9}}],

        Cell[BoxData[
         RowBox[{"ForDynArt", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{{3.525146419655585*^9, 3.52514642229861*^9}}],

        Cell[BoxData[
         RowBox[{"ForDynComp", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582894937*^9, 3.4691656164047832`*^9, 3.499410251421309*^9,
           3.499410355748085*^9, {3.52514642559848*^9, 3.525146428415546*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{"LinkInformation", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582926524*^9, 3.469165616435227*^9, {3.525146431186875*^9,
           3.525146434575663*^9}}],

        Cell["\<\
        Note that the list below is {1,2,3,4,5,6,7,8,9,10,11,12}, but each number \
        replaced by the successor link\
        \>", "Text"],

        Cell[BoxData[""], "Input",
         CellChangeTimes->{3.46916558296457*^9, 3.469165616469927*^9,
          3.487620831126145*^9}],

        Cell[BoxData[
         RowBox[{"ParmEst", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.4691655829948797`*^9, 3.469165616154977*^9, {3.525146437669165*^9,
           3.525146439684971*^9}}],

        Cell[BoxData["\t"], "Input",
         CellChangeTimes->{3.499409450658988*^9}],

        Cell[CellGroupData[{

        Cell[BoxData["\[AliasDelimiter]"], "Input",
         CellChangeTimes->{3.525146451826103*^9}],

        Cell[BoxData["\[AliasDelimiter]"], "Output",
         CellChangeTimes->{3.525151787140445*^9, 3.525151820416446*^9,
          3.525151900535952*^9, 3.525151967371533*^9, 3.525152058497048*^9,
          3.525152134681759*^9, 3.525152240817702*^9, 3.525152720084728*^9,
          3.525152880902732*^9, 3.525152995312067*^9, 3.525155871299821*^9,
          3.525157395534119*^9}]
        }, Open  ]]
        },
        WindowToolbars->{},
        CellGrouping->Automatic,
        WindowSize->{671, 773},
        WindowMargins->{{63, Automatic}, {22, Automatic}},
        PrivateNotebookOptions->{"ColorPalette"->{RGBColor, 128}},
        ShowSelection->True,
        ShowCellLabel->True,
        ShowCellTags->False,
        RenderingOptions->{"ObjectDithering"->True,
        "RasterDithering"->False},
        CharacterEncoding->"MacintoshAutomaticEncoding",
        FrontEndVersion->"7.0 for Linux x86 (32-bit) (February 25, 2009)",
        StyleDefinitions->"Default.nb"
        ]
        (* End of Notebook Content *)

        (* Internal cache information *)
        (*CellTagsOutline
        CellTagsIndex->{}
        *)
        (*CellTagsIndex
        CellTagsIndex->{}
        *)
        (*NotebookFileOutline
        Notebook[{
        Cell[545, 20, 395, 8, 32, "Input"],
        Cell[943, 30, 230, 4, 32, "Input"],
        Cell[1176, 36, 259, 6, 32, "Input"],
        Cell[1438, 44, 82, 1, 31, "Text"],
        Cell[1523, 47, 239, 5, 32, "Input"],
        Cell[1765, 54, 571, 12, 32, "Input"],
        Cell[2339, 68, 328, 8, 32, "Input"],
        Cell[2670, 78, 268, 6, 32, "Input"],
        Cell[2941, 86, 441, 8, 32, "Input"],
        Cell[3385, 96, 274, 6, 32, "Input"],
        Cell[3662, 104, 397, 9, 32, "Input"],
        Cell[4062, 115, 236, 5, 32, "Input"],
        Cell[4301, 122, 130, 3, 51, "Text"],
        Cell[4434, 127, 114, 2, 32, "Input"],
        Cell[4551, 131, 326, 8, 32, "Input"],
        Cell[4880, 141, 70, 1, 32, "Input"],
        Cell[CellGroupData[{
        Cell[4975, 146, 85, 1, 32, "Input"],
        Cell[5063, 149, 340, 5, 31, "Output"]
        }, Open  ]]
        }
        ]
        *)

        (* End of internal cache information *)
    '''


    def public dynModel(Robot robot) '''
        { (* Base Coordinate System *)
        {jointID,{ID=0}},
        {floatingBase,{«IF robot.base.floating»1«ELSE»0«ENDIF»}},
        {notUsed,{}},
        {notUsed,{}},
        {successors,{1}},
        {inertia,GenInertiaMatrixS["links",ID,1]},
        {massCenterMass,GenMCMVectorS["links",ID,1]},
        {mass,GenMassS["links",ID]},
        {baseVariables,GenBaseVariablesS["basec","baseo",ID]},
        {extForce,GenExtForceS["uex",ID]}
        }
        «FOR Joint j : robot.joints»
            { (* Joint «j.name» *)
            {jointID,{ID=«j.ID»}},
            «j.jointAxis»,
            {translation,{«jointTranslParams(j)»}},
            {rotationMatrix,{«jointRotParams(j)»}},
            {successors,{«j.successors»}},
            {inertia,GenInertiaMatrixS["links",ID,1]},
            {massCenterMass,GenMCMVectorS["links",ID,1]},
            {mass,GenMassS["links",ID]},
            {jointVariables,GenVariablesS["state",ID]},
            {extForce,GenExtForceS["uex",ID]}
            }
        «ENDFOR»
        { (* dummy to draw endeffector *)
        {jointID,{ID=104}},
        {jointAxis,{0,0,0}},
        {translation,{eff$1$$x[[1]],eff$1$$x[[2]],eff$1$$x[[3]]}},
        {rotationMatrix,{eff$1$$a[[1]],eff$1$$a[[2]],eff$1$$a[[3]]}},
        {successors,{}},
        {inertia,{{0,0,0},{0,0,0},{0,0,0}}},
        {massCenterMass,{eff$1$$mcm[[1]],eff$1$$mcm[[2]],eff$1$$mcm[[3]]}},
        {mass,{eff$1$$m}},
        {jointVariables,{0,0,0,0,0}},
        {extForce,{0,0,0,0,0,0}}
        }
        '''

    def private dispatch jointAxis(PrismaticJoint j)'''
        {jointAxis,{0,0,0,0,0,1}}'''
    def private dispatch jointAxis(RevoluteJoint j)'''
        {jointAxis,{0,0,1}}'''
    def private getSuccessors(Joint j)'''
        «val children =  j.successorLink.childrenList.children»
        «IF children.empty»
            104
        «ELSE»
            «FOR ChildSpec child : children SEPARATOR ','»
                «child.joint.ID»
            «ENDFOR»
        «ENDIF»'''

    def private jointTranslParams(Joint j)
    '''«val tr=j.refFrame.translation»«string(tr.x)», «string(tr.y)», «string(tr.z)»'''
    def private jointRotParams(Joint j)
    '''«val rot=j.refFrame.rotation»«string(rot.x)», «string(rot.y)», «string(rot.z)»'''

    def private dispatch CharSequence string(FloatLiteral id)'''«id.value.str»'''
    def private dispatch CharSequence string(PlainExpr expr) '''«string(expr.identifier)»'''
    def private dispatch CharSequence string(MultExpr expr)  '''«expr.mult.str» «string(expr.identifier)»'''
    def private dispatch CharSequence string(DivExpr expr)   '''«string(expr.identifier)»/«expr.div»'''
    def private dispatch CharSequence string(ParameterLiteral id)  '''«id.str»'''
    def private dispatch CharSequence string(PILiteral pi)   '''«IF pi.minus»-«ENDIF»Pi'''


    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    private extension iit.dsl.generator.sl.Common slCommon = new iit.dsl.generator.sl.Common()
}