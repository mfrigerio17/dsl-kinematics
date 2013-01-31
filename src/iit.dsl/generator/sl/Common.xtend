package iit.dsl.generator.sl

import iit.dsl.kinDsl.Robot

class Common {

def String robotFolderName(Robot robot) {
    robot.name.toLowerCase()
}
def String robotName(Robot robot) {
    robot.name.toLowerCase()
}

def String robotUserFolderName(Robot robot) {
    robotFolderName(robot) + "User"
}

def String benchmarkIDFileName(Robot robot) {
    "main_benchmarkID"
}

def String main_inertiaM_filename(Robot robot) {
    "main_inertiaM"
}

}