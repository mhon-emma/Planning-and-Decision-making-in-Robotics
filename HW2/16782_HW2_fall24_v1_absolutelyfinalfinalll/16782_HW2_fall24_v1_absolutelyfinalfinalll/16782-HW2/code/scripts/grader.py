import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    problems = [["map1.txt", "1.575521,0.788558,1.573756,0.755398,1.570796",
                                # "0.392699,2.356194,3.141592,2.8274328,4.712388"],
                                "0.392099,2.356754,3.131222,2.8274325,4.712358"],
            ["map2.txt", "0.392309,2.356534,3.155532",
                                "1.571156,0.755288,1.570596"]]
    scores = []
    for aPlanner in [0, 1, 2, 3]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "../output/grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./../build/verifier {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python visualizer.py ../output/grader_out/tmp.txt --gifFilepath=../output/grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./../build/planner", "../output/grader_out/grader_results.csv")