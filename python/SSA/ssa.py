from itertools import permutations
import numpy as np

if __name__ == '__main__':

    data = {"A": {"B": 3, "C": 9, "D": 15, "E": 20},
            "B": {"A": 3, "C": 5, "D": 11, "E": 15},
            "C": {"A": 9, "B": 5, "D": 5, "E": 9},
            "D": {"A": 15, "B": 11, "C": 5, "E": 3},
            "E": {"A": 20, "B": 15, "C": 9, "D": 3}}


    def findLowestPointPath(pointDistances, startPoint, endPoint):

        startPointDistance = pointDistances.get(startPoint).copy()
        startPointDistance.pop(endPoint)

        sortedDict = {}
        distanceDict = {}

        # 针对起始点对应的dict,遍历除目的地点之外的所有点
        for point in startPointDistance:

            processedPoint = [startPoint, point, endPoint]

            sortedPoint = [startPoint, point]

            distanceTotal = startPointDistance.get(point)

            node = point

            while node is not None:
                pointDistance = pointDistances.get(node)
                node = findLowestDistancePoint(pointDistance, processedPoint)

                if node is None:
                    distance = pointDistance.get(endPoint)
                    sortedPoint.append(endPoint)
                else:
                    distance = pointDistance.get(node)
                    sortedPoint.append(node)

                distanceTotal += distance
                processedPoint.append(node)

            sortedDict[point] = sortedPoint
            distanceDict[point] = distanceTotal

        lowestDistancePoint = sorted(distanceDict.items(), key=lambda d: d[1], reverse=False)[0][0]

        return sortedDict.get(lowestDistancePoint)


    def findLowestDistancePoint(poinDistance, processed):
        """
            从字典中获取最短距离节点
        """
        lowestDistance = float("inf")

        lowestDistanceNode = None

        for node in poinDistance:
            distance = poinDistance[node]

            if distance < lowestDistance and node not in processed:
                lowestDistance = distance
                lowestDistanceNode = node

        return lowestDistanceNode


    def findPointPathByPermutations(pointDistances, pointList):
        permutationsList = pointList[1: -1]

        lowestDistance = float("inf")
        lowestDistancePathPoint = None

        for path in permutations(permutationsList, len(permutationsList)):

            startPoint = pointList[0]

            endPoint = pointList[-1]

            distanceTotal = 0

            for index in range(len(path)):
                distanceTotal += pointDistances.get(startPoint).get(path[index])
                startPoint = path[index]

            distanceTotal += pointDistances.get(path[-1]).get(endPoint)

            if distanceTotal < lowestDistance:
                lowestDistance = distanceTotal
                lowestDistancePathPoint = path

        return pointList[0] + "-" + "-".join(lowestDistancePathPoint) + "-" + pointList[-1]


    # print(findPointPathByPermutations(data, ["A", "B", "C", "D", "E"]))

    # print(list(permutations(["a", "b", "c", "d"], 2)))

    def initDistanceArray(pointDistanceDicts, pointNum):
        """ 传入每个点之间对应的距离字典以及需要计算的点的个数,返回点与点对应的距离矩阵 """

        pointArray = np.zeros((pointNum, pointNum))

        for i in range(pointNum):
            pointDistanceDict = pointDistanceDicts.get(i)

            for j in range(pointNum):

                if i == j:
                    continue

                if pointArray[i][j] != 0:
                    continue

                pointArray[i][j] = pointArray[j][i] = pointDistanceDict.get(j)

        return pointArray


    def computePointPathLength(pointArray, pointPath):
        """ 传入距离矩阵以及点的轨迹列表,计算路线距离 """

        length = 0

        for i in range(len(pointPath) - 1):
            length += pointArray[pointPath[i]][pointPath[i + 1]]

        return length


    def disruptionOrderPath(pointPath):

        pointNum = len(pointPath)

        changeHead = np.random.randint(1, pointNum - 1)
        changeTail = np.random.randint(1, pointNum - 1)

        if changeHead > changeTail:
            changeHead, changeTail = changeTail, changeHead

        changePath = pointPath[changeHead: changeTail + 1]
        changePath.reverse()

        newPath = pointPath[:changeHead] + changePath + pointPath[changeTail + 1:]

        return changeHead, changeTail, newPath


    def diffNewAndOldPathDistance(pointArray, newPath, oldPath, head, tail):

        newLength = pointArray[newPath[head - 1]][newPath[head]] + pointArray[newPath[tail]][newPath[tail + 1]]
        oldLength = pointArray[oldPath[head - 1]][oldPath[head]] + pointArray[oldPath[tail]][oldPath[tail + 1]]

        deltaDistance = newLength - oldLength

        return deltaDistance


    def findLowestPointPathBySA(pointDistanceDicts, pointPath, T_start=2000, T_end=1e-20, T_speed=0.95, Lk=50):
        """
        :param pointDistanceDicts: 点与点之间距离字典,样例数据如下
                 data = {0: {1: 3, 2: 9, 3: 15, 4: 20},
                        1: {0: 3, 2: 5, 3: 11, 4: 15},
                        2: {0: 9, 1: 5, 3: 5, 4: 9},
                        3: {0: 15, 1: 11, 2: 5, 4: 3},
                        4: {0: 20, 1: 15, 2: 9, 3: 3}}
        :param pointPath: 初始点顺序列表,样例数据如下[0,1,2,3,4]
        :param T_start: 起始温度,默认为2000
        :param T_end: 结束温度,默认为1e-20
        :param T_speed: 降温速率,默认为0.995
        :param LK: 内循环次数,马尔科夫链长,默认为50
        :return: 退火后的最小运行距离
        """
        pointNum = len(pointPath)

        pointArray = initDistanceArray(pointDistanceDicts, pointNum)

        bestLength = computePointPathLength(pointArray, pointPath)

        bestPointPath = pointPath
        initPointPath = pointPath

        while T_start > T_end:
            for i in range(Lk):
                head, tail, newPointPath = disruptionOrderPath(initPointPath)
                deltaDistance = diffNewAndOldPathDistance(pointArray, newPointPath, initPointPath, head, tail)

                """ 比较新旧路径长度之差,如果差值小于0,将新路径赋值给初始化路径 """
                if deltaDistance < 0:

                    initPointPath = newPointPath

                    newLength = computePointPathLength(pointArray, newPointPath)

                    if newLength < bestLength:
                        bestLength = newLength
                        bestPointPath = newPointPath

                elif np.random.random() < np.exp(-deltaDistance / T_start):
                    """ 以概率的方式接受状态 """
                    initPointPath = newPointPath

            initPointPath = bestPointPath

            T_start *= T_speed

        return bestPointPath


    data = {0: {1: 3, 2: 9, 3: 15, 4: 20, 5: 30, 6: 35},
            1: {0: 3, 2: 5, 3: 11, 4: 15, 5: 25, 6: 30},
            2: {0: 9, 1: 5, 3: 5, 4: 9, 5: 20, 6: 25},
            3: {0: 15, 1: 11, 2: 5, 4: 3, 5: 15, 6: 20},
            4: {0: 20, 1: 15, 2: 9, 3: 3, 5: 10, 6: 15},
            5: {0: 30, 1: 25, 2: 20, 3: 15, 4: 10, 6: 5},
            6: {0: 35, 1: 30, 2: 25, 3: 20, 4: 15, 5: 5}}

    import time
    start = time.time()

    print(findLowestPointPathBySA(data, [0, 2, 3, 4, 1, 5, 6]))

    end = time.time()

    print(end - start)

    #
    # print(findLowestPointPathBySA(data, [0, 2, 3, 1, 4]))
    #
    # pointArray = initDistanceArray(data, 5)
    #
    # print(computePointPathLength(pointArray, [0,1,2,3,4]))
    # print(computePointPathLength(pointArray, [0,1,3,2,4]))
    #
    # print(diffNewAndOldPathDistance(pointArray, [0,1,2,3,4],[0,1,3,2,4],2,3))

