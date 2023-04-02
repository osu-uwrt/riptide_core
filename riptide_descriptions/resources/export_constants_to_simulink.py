#Generates Constants from riptide_descriptions (tempest.yaml) for simulink

import os
import yaml

class GenerateConstants:

    # Path variables to important files
    PATH_TO_SIMULINK = os.path.join(os.path.dirname(__file__), "..", "..", "..", "riptide_simulink")
    PATH_TO_TEMPEST = os.path.join(os.path.dirname(__file__), "..", "config", "talos.yaml")
    PATH_TO_WRITE = os.path.join(PATH_TO_SIMULINK, "Models", "Simple_3_Model", "Constants", "RobotConstants.m")

    constants_needed = ["height", "volume", "com", "cob", "inertia", "thrusters", "coefficients"]
    thrusters_constants_needed = [
        "VUSPos", "VUSDir", 
        "VUPPos", "VUPDir", 
        "HUSPos", "HUSDir", 
        "HUPPos", "HUPDir", 
        "HLSPos", "HLSDir", 
        "HLPPos", "HLPDir", 
        "VLSPos", "VLSDir", 
        "VLPPos", "VLPDir", 
    ]
    coefficient_constants_needed = ["roll", "pitch", "yaw", "x", "y", "z"]

    # Contains all constant data after loading (except world)
    DATA = 0

    # checks if riptide_simulink exists
    # NEEDS riptide_simulink to generate constants
    def checkSimulink(self) -> bool:

        if os.path.exists(self.PATH_TO_SIMULINK):
            return True
        else:
            print ("riptide_simulink does not exist")
            return False

    def loadConstants(self):

        with open(self.PATH_TO_TEMPEST, "r") as data:
            self.DATA = yaml.safe_load(data)
    
    def readConstant(self, constant):

        if "." in constant:
            constant = constant.split(".")
            if constant[0] == "thrusters": 
                return self.readThrusterConstant(constant[1])
            elif constant[0] == "coefficients":
                return self.readCoefficientConstant(constant[1])
        else:
            if constant == "height":
                return 0.32
            else:
                return self.DATA[constant]

    def readThrusterConstant(self, constant):
        thruster_to_index = {"VUS": 0, "VUP" : 1, "HUS":2, "HUP":3, "HLS": 4, "HLP": 5, "VLS": 6, "VLP": 7}
        index = thruster_to_index[constant[:3]]

        poseDir = self.DATA["thrusters"][index]["pose"]

        if constant[3:] == "Pos":
            return poseDir[:3]
        else:
            return poseDir[3:]

    def readCoefficientConstant(self, constant):
        result = [0, 0, 0]

        if constant in ["roll", "pitch", "yaw"]:
            coef_type = "angular"
            coef_index = ["roll", "pitch", "yaw"].index(constant)
        elif constant in ["x", "y", "z"]:
            coef_type = "linear"
            coef_index = ["x", "y", "z"].index(constant)

        result[0] = self.DATA["controller"][coef_type]["damping"]["quadratic"][coef_index]
        result[1] = self.DATA["controller"][coef_type]["damping"]["linear"][coef_index]

        return result

    def writeConstants(self):
        #TODO: addd thruster number comments to matlab file
        if self.checkSimulink():

            self.loadConstants()
            
            with open(self.PATH_TO_WRITE, "w") as importedData:

                for constant in self.constants_needed:
                    if constant == 'thrusters':
                        for thruster in self.thrusters_constants_needed:
                            result = self.readConstant("thrusters." + thruster)
                            importedData.write(f"importedData.robot.thrusters.{thruster} = {result};\n")

                    elif constant == 'coefficients':
                        for coeff in self.coefficient_constants_needed:
                            result = self.readConstant("coefficients." + coeff)
                            importedData.write(f"importedData.robot.coefficients.{coeff} = {result};\n")

                    else:
                        result = self.readConstant(constant)
                        importedData.write(f"importedData.robot.{constant} = {result};\n")


if __name__ == "__main__":
    run = GenerateConstants()
    run.writeConstants()