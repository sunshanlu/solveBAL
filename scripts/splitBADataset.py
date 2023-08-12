filePath = r"../res/BALProblem.txt"  # BAL Problem dataset path
edgeFile = r"../res/EdgeFile.txt"  # edge rule file path
camVertexFile = r"../res/CamVertexFile.txt"  # initial camera params file path
pointVertexFile = r"../res/PointVertexFile.txt"  # initial world point params file path

edgeNum = 83718
camParamNum = 16 * 9
pointParamNum = 22106 * 3

if __name__ == '__main__':
    # read BALProblem file
    with open(filePath, "r") as f:
        dataLines = f.readlines()
        edgesData = dataLines[1: edgeNum + 1]
        camParamsData = dataLines[edgeNum + 1: edgeNum + 1 + camParamNum]
        pointsData = dataLines[edgeNum + 1 + camParamNum: edgeNum + 1 + camParamNum + pointParamNum]

        # write the edge information to the EdgeFile.txt
        with open(edgeFile, "w", encoding="utf8") as edgeF:
            for i in range(edgeNum):
                edgeF.write(edgesData[i])

        # write the camera pose information to the CamVertexFile.txt
        with open(camVertexFile, "w", encoding="utf8") as camVertexF:
            idx = 0
            for i in range(16):
                writeStr = ''
                for j in range(9):
                    writeStr += camParamsData[idx].strip() + "\t"
                    idx += 1
                writeStr += "\n"
                camVertexF.write(writeStr)

        # write the point in the world coordinate system to the PointVertexFile.txt
        with open(pointVertexFile, "w", encoding="utf8") as pointVertexF:
            idx = 0
            for i in range(22106):
                writeStr = ""
                for j in range(3):
                    writeStr += pointsData[idx].strip() + "\t"
                    idx += 1
                writeStr += "\n"
                pointVertexF.write(writeStr)
