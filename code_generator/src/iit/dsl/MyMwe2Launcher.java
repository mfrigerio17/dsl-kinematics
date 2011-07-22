package iit.dsl;

import org.eclipse.emf.mwe2.launch.runtime.Mwe2Launcher;

/*
 * To deploy the parser and code generator as a stand-alone jar package use
 * "Export as.. executable jar" after right clicking on the project, in the
 * project explorer.
 * First you have to create a run configuration, where you choose this class
 * as the main class.
 * When you export the jar you will have to select such run configuration.
 *
 * Execute the parser/generator from the command line with
 * 		java -jar <path to exported jar> <workflow module name> [-p <property=value>]
 * e.g.
 * 		java -jar mygenerator.jar workflow.KinDslGenerator -p targetDir=generated
 */

/*
 * Simple wrapper class to make the "export as executable jar" happy, since
 * otherwise it complains that it does not find a class with main()
 */
public class MyMwe2Launcher {
	public static void main(String[] args) {
		Mwe2Launcher.main(args);
	}

}
