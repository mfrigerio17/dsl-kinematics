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

def String userConfigFolder() {
    return "config"
}

def String benchmarkFileName(Robot robot) {
    "mymain"
}

}