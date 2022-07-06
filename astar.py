import math
from functools import reduce
import typing
import cProfile
import heapq


diagnostic_use_astar = True
agents_block = False

class ATile:
    def __init__(self, passable, cost, game_state):
        self.cost = cost  # cost to enter
        self.game_state = game_state
        self.in_unvisited_set = False
        self.visited = False  # open 1, closed 0
        self._passable = passable
        self.agent_present = False
        self.gCost, self.hCost = 0, 0  # g: dist from start, h: dist from end
        self.xIndex, self.yIndex = 0, 0  # inverted y?
        self.parent = None

    # used when comparing ATile objects on priority queue (min heap implementation)
    def __gt__(self, cost):
        return self.fCost > cost

    def __lt__(self, cost):
        return self.fCost < cost

    def __eq__(self, cost):
        return self.fCost == cost

    @property
    def fCost(self):  # don't need to set
        return self.gCost + self.hCost

    @property
    def passable(self):
        if agents_block == False:
            return self._passable
        else:
            return self._passable and (self.game_state.get_agent_at(self.xIndex, self.yIndex) == None)


def HeuristicPath(xStart, yStart, xEnd, yEnd):
    x_dist = abs(xEnd - xStart)
    y_dist = abs(yEnd - yStart)
    if x_dist > y_dist:
        return x_dist
    else:
        return y_dist

'''not thread safe, 2 diff threads should not use'''
class AStar:
    # gets passed 2d array of tiles, and a function of how to convert them
    # to _Agrid
    def __init__(self, inputGrid, inputTileToAstarTile, game_state):
        # gives a 2d grid of ATile's and unpassable squares or tiles
        self.inputGrid = inputGrid
        self.game_state = game_state
        #self.transferTilesToAStarTiles = inputTileToAstarTile
        #self.inputTileToAstarTile_excludeAgents = inputTileToAstarTile_excludeAgents
        #self._updateGrid(self.transferTilesToAStarTiles)  # initializes [][] ?
        self.count_times_called = 0
        self.result = []
        self.neighbors = [None, None, None, None, None, None, None, None]  # cached!!!! reuse this object
        self.tiles_to_reinitialize = []
        self.saved_paths = {}
        self.grid = \
            list(map(lambda x: list(map(lambda j: ATile(inputTileToAstarTile(j), j.cost, game_state), x)), self.inputGrid))

        for j, row in enumerate(self.grid):
            for i, tile in enumerate(row):  # don't need to use enumerate
                tile.xIndex = i  # self.grid[j][i].xIndex = i
                tile.yIndex = j  # self.grid[j][i].yIndex = j
                tile.parent = None
                tile.gCost = math.inf
                tile.visited = False
                tile.in_unvisited_set = False

        self.save_all_paths()


    def _updateGrid(self, find_attack_range):
        global agents_block
        agents_block = not find_attack_range

        for tile in self.tiles_to_reinitialize:
            tile.parent = None
            tile.gCost = math.inf
            tile.hCost = 0
            tile.visited = False
            tile.in_unvisited_set = False

        self.tiles_to_reinitialize = []

        # for j, row in enumerate(self.grid):
        #     for i, tile in enumerate(row):  # don't need to use enumerate
        #         tile.parent = None
        #         tile.gCost = math.inf
        #         tile.visited = False
        #         tile.in_unvisited_set = False

    def save_all_paths(self):
        from time import time
        import sys
        start_time = time()
        for j, row in enumerate(self.grid[1:len(self.grid)-1]):
            for i, tile in enumerate(row[1:len(row)-1]):  # don't need to use enumerate
                self.dj_fill(i+1, j+1)

        elapsed_time = time() - start_time
        print(f'save_all_paths() finished! elapsed time: {elapsed_time}')
        print(f'memory usage of dict: {sys.getsizeof(self.saved_paths)}')

    def dj_fill(self, xStart, yStart):
        self.count_times_called += 1
        # priority queue implemented via a min-heap
        unvisited_set = []  # , visited_set = set([]), set([])

        self._updateGrid(True)  #ignore agents

        current = self.grid[yStart][xStart]
        current.gCost = 0
        current.in_unvisited_set = True
        self.tiles_to_reinitialize.append(current)
        heapq.heappush(unvisited_set, current)
        while True:
            current = heapq.heappop(unvisited_set)
            self._getNeighbors(current)
            for neighbor in self.neighbors:
                if current.gCost + neighbor.cost <= neighbor.gCost:
                    if neighbor.visited == True or neighbor.passable == False:
                        continue
                    neighbor.gCost = current.gCost + neighbor.cost
                    neighbor.parent = current
                if neighbor.in_unvisited_set == False:
                    self.tiles_to_reinitialize.append(neighbor)
                    neighbor.in_unvisited_set = True
                    heapq.heappush(unvisited_set, neighbor)
            current.visited = True
            x_end, y_end = current.xIndex, current.yIndex
            # found a shortest path, generate result and update dictionary
            # result = []
            # while True:
            #     result.append((current.xIndex, current.yIndex))
            #     if current.parent == None:
            #         break
            #     else:
            #         current = current.parent
            # result.reverse()

            #self.saved_paths.update({(xStart, yStart, x_end, y_end): result})
            self.saved_paths.update({(xStart, yStart, x_end, y_end): current.gCost})

            # if self.grid[yEnd][xEnd].visited == True:
            #     # found shortest path
            #     self.result.clear()
            #     # print(f'parent for [12,1],[13,1],[14,1] {self.grid[1][12].parent, self.grid[1][13].parent, self.grid[1][14].parent}')
            #     while True:
            #         self.result.append([current.xIndex, current.yIndex])
            #
            #         if current.parent == None:
            #             break
            #         else:
            #             current = current.parent
            #
            #     self.result.reverse()
            #     return self.result  # path has been found

            if len(unvisited_set) <= 0:
                # done with fill
                return

    ''' to be used to find area that could be moved in one action'''
    def paths_with_max_cost(self, xStart, yStart, max_cost):
        result = []
        # self.count_times_called += 1
        # priority queue implemented via a min-heap
        unvisited_set = []  # , visited_set = set([]), set([])

        self._updateGrid(False)  # agents are considered blocking

        current = self.grid[yStart][xStart]
        current.gCost = 0
        current.in_unvisited_set = True
        self.tiles_to_reinitialize.append(current)
        heapq.heappush(unvisited_set, current)
        while True:
            current = heapq.heappop(unvisited_set)
            # check if highest priority element on the queue (heap implementation)
            # has a fCost/gCost higher than the max cost, if so exit and return results
            if current.gCost > max_cost:
                return result
            # consider only neighbors that are unvisited
            self._getNeighbors(current)
            for neighbor in self.neighbors:
                if neighbor.visited == True or neighbor.passable == False:
                    continue  # filter out tiles
                if current.gCost + neighbor.cost <= neighbor.gCost:
                    neighbor.gCost = current.gCost + neighbor.cost
                    neighbor.parent = current
                if neighbor.in_unvisited_set == False:
                    self.tiles_to_reinitialize.append(neighbor)
                    neighbor.in_unvisited_set = True
                    heapq.heappush(unvisited_set, neighbor)
            current.visited = True
            result.append((current.xIndex, current.yIndex))
            #self.saved_paths.update({(xStart, yStart, x_end, y_end): current.gCost})

            # unvisited_set.remove(current)

            # if self.grid[yEnd][xEnd].visited == True:
            #     # found shortest path
            #     self.result.clear()
            #     # print(f'parent for [12,1],[13,1],[14,1] {self.grid[1][12].parent, self.grid[1][13].parent, self.grid[1][14].parent}')
            #     while True:
            #         self.result.append((current.xIndex, current.yIndex))
            #
            #         if current.parent == None:
            #             break
            #         else:
            #             current = current.parent
            #
            #     self.result.reverse()
            #     return self.result  # path has been found

            if len(unvisited_set) <= 0:
                # no more unvisited nodes, end cannot be reached
                self.result = self.result  # []
                return self.result

            # unvisited set has 2500 items
            # current = reduce(lambda i, j: i if i.gCost <= j.gCost else j, unvisited_set)

    # tile.gCost is the tentative cost, shortest path so far to initial node
    # dj
    def path_dj(self, xStart, yStart, xEnd, yEnd, find_attack_range=False):
        # self.count_times_called += 1
        # priority queue implemented via a min-heap
        unvisited_set = []  # , visited_set = set([]), set([])

        self._updateGrid(find_attack_range)

        current = self.grid[yStart][xStart]
        current.gCost = 0
        current.in_unvisited_set = True
        self.tiles_to_reinitialize.append(current)
        heapq.heappush(unvisited_set, current)
        while True:
            current = heapq.heappop(unvisited_set)
            # consider only neighbors that are unvisited
            # neighbors = list(filter(lambda x: x.visited == False and x.passable == True,
            #                         self._getNeighbors(current)))
            self._getNeighbors(current)
            # unvisited_set.union(neighbors)
            # unvisited_set = set().union(unvisited_set, neighbors)
            for neighbor in self.neighbors:
                if current.gCost + neighbor.cost <= neighbor.gCost:
                    if neighbor.visited == True or neighbor.passable == False:
                        continue
                    neighbor.gCost = current.gCost + neighbor.cost
                    neighbor.parent = current
                if neighbor.in_unvisited_set == False:
                    self.tiles_to_reinitialize.append(neighbor)
                    neighbor.in_unvisited_set = True
                    heapq.heappush(unvisited_set, neighbor)
            current.visited = True
            # unvisited_set.remove(current)

            if self.grid[yEnd][xEnd].visited == True:
                # found shortest path
                self.result.clear()
                # print(f'parent for [12,1],[13,1],[14,1] {self.grid[1][12].parent, self.grid[1][13].parent, self.grid[1][14].parent}')
                while True:
                    self.result.append((current.xIndex, current.yIndex))

                    if current.parent == None:
                        break
                    else:
                        current = current.parent

                self.result.reverse()
                return self.result  # path has been found

            if len(unvisited_set) <= 0:
                # no more unvisited nodes, end cannot be reached
                self.result = []
                return []

            # unvisited set has 2500 items
            # current = reduce(lambda i, j: i if i.gCost <= j.gCost else j, unvisited_set)

    def path_astar(self, xStart, yStart, xEnd, yEnd, find_attack_range=False):
        # self.count_times_called += 1
        # priority queue implemented via a min-heap
        unvisited_set = []  # , visited_set = set([]), set([])

        # if find_attack_range == False:  # agents will be considered blocking
        #     self._updateGrid(self.transferTilesToAStarTiles)
        # else:  # agents will not be considered blocking
        #     self._updateGrid(self.inputTileToAstarTile_excludeAgents)
        #
        # # initialize graph
        # for j, row in enumerate(self.grid):
        #     for i, column in enumerate(row):
        #         self.grid[j][i].parent = None
        #         # self.grid[j][i].closed = -1  # invalid?
        #         self.grid[j][i].gCost = math.inf
        #         self.grid[j][i].visited = False

        self._updateGrid(find_attack_range)

        current = self.grid[yStart][xStart]
        current.gCost = 0
        current.hCost = HeuristicPath(xStart, yStart, xEnd, yEnd)
        current.in_unvisited_set = True
        self.tiles_to_reinitialize.append(current)
        heapq.heappush(unvisited_set, current)

        while True:
            current = heapq.heappop(unvisited_set)
            # consider only neighbors that are unvisited
            # neighbors = list(filter(lambda x: x.visited == False and x.passable == True,
            #                         self._getNeighbors(current)))
            self._getNeighbors(current)
            #  newlist = [x if x != "banana" else "orange" for x in fruits]
            #self.neighbors = [x if x.visited == False and x.passable == True else None for x in self.neighbors]
            for neighbor in self.neighbors:
                if neighbor.visited == True or neighbor.passable == False:
                    continue
                # neighbor.fCost > current.fCost + dist
                if current.gCost + neighbor.cost <= neighbor.gCost:
                    neighbor.gCost = current.gCost + neighbor.cost
                    neighbor.hCost = HeuristicPath(neighbor.xIndex, neighbor.yIndex, xEnd, yEnd)
                    neighbor.parent = current

                if neighbor.in_unvisited_set == False:
                    self.tiles_to_reinitialize.append(neighbor)
                    neighbor.in_unvisited_set = True
                    heapq.heappush(unvisited_set, neighbor)
            current.visited = True
            # unvisited_set.remove(current)

            if self.grid[yEnd][xEnd].visited == True:
                # found shortest path
                #result = []  # initialize
                self.result.clear()

                while True:
                    self.result.append((current.xIndex, current.yIndex))

                    if current.parent == None:
                        break
                    else:
                        current = current.parent

                self.result.reverse()
                # print(result)
                #self.result = result
                return self.result  # path has been found

            if len(unvisited_set) <= 0:
                # no more unvisited nodes, end cannot be reached
                self.result = []
                return []

            # unvisited set has 2500 items
            # current = reduce(lambda i, j: i if i.gCost <= j.gCost else j, unvisited_set)

    def path(self, xStart, yStart, xEnd, yEnd, find_attack_range=False):
        # cProfile.runctx('self.path_astar(xStart, yStart, xEnd, yEnd, find_attack_range)', {},
        #                 {"self": self, "xStart": xStart, "yStart": yStart, "xEnd": xEnd, "yEnd": yEnd,
        #                  "find_attack_range": find_attack_range}, filename="profile/path.dat")
        #self.path_dj(xStart, yStart, xEnd, yEnd, find_attack_range)

        # look up path, check for conflicts, if conflict, rely on a* algorithm for new path
        self.count_times_called += 1
        path = []
        path.append((xEnd, yEnd))
        current_tile = self.grid[yEnd][xEnd]
        while (current_tile is not self.grid[yStart][xStart]):
            # try:
            current_g_cost = self.saved_paths.get((xStart, yStart, current_tile.xIndex, current_tile.yIndex), None)
            if current_g_cost == None:
                path = []
                break
            self._getNeighbors(current_tile)
            best_g_cost = current_g_cost
            best_tile = current_tile
            for neighbor in self.neighbors:
                neighbor_g_cost = self.saved_paths.get((xStart, yStart, neighbor.xIndex, neighbor.yIndex), math.inf)
                if best_g_cost > neighbor_g_cost:
                    best_g_cost = neighbor_g_cost
                    best_tile = neighbor
            current_tile = best_tile
            path.append((current_tile.xIndex, current_tile.yIndex))
            # except Exception as e:
            #     print(f'path error: {e}')

        path.reverse()
        found_conflict = False
        for xy in path[1:]:
            if self.game_state.get_agent_at(xy[0],xy[1]) != None:
                found_conflict = True
                break

        if found_conflict == True:
            path = self.path_astar(xStart, yStart, xEnd, yEnd, find_attack_range)
            #print('Pathing conflict found')
        else:
            #print('No pathing conflict')
            pass

        self.result = path
        return self.result

    def _getNeighbors(self, current):
        i, j = current.xIndex, current.yIndex
        #result = []
        #try:

        self.neighbors[0] = self.grid[j][i + 1]  # theta = 0pi
        self.neighbors[1] = self.grid[j + 1][i + 1]
        self.neighbors[2] = self.grid[j + 1][i]
        self.neighbors[3] = self.grid[j + 1][i - 1]
        self.neighbors[4] = self.grid[j][i - 1]
        self.neighbors[5] = self.grid[j - 1][i - 1]
        self.neighbors[6] = self.grid[j - 1][i]
        self.neighbors[7] = self.grid[j - 1][i + 1]  # theta just under 2pi
        # except:
        #     pass
        #return result

    # def _getNeighbors(self, current):
    #     i, j = current.xIndex, current.yIndex
    #     result = []
    #     try:
    #         result.append(self.grid[j][i + 1])  # theta = 0pi
    #         result.append(self.grid[j + 1][i + 1])
    #         result.append(self.grid[j + 1][i])
    #         result.append(self.grid[j + 1][i - 1])
    #         result.append(self.grid[j][i - 1])
    #         result.append(self.grid[j - 1][i - 1])
    #         result.append(self.grid[j - 1][i])
    #         result.append(self.grid[j - 1][i + 1])  # theta just under 2pi
    #     except:
    #         pass
    #     return result

    # get rid of!?
    def getNeighbors(self, xIndex, yIndex) -> list[tuple[int, int]]:
        result = []

        try:
            result.append((xIndex + 1, yIndex))  # theta = 0pi
            result.append((xIndex + 1, yIndex + 1))
            result.append((xIndex, yIndex + 1))
            result.append((xIndex - 1, yIndex + 1))
            result.append((xIndex - 1, yIndex))
            result.append((xIndex - 1, yIndex - 1))
            result.append((xIndex, yIndex - 1))
            result.append((xIndex + 1, yIndex - 1))  # theta just under 2pi
            result = list(filter(lambda x: self.grid[x[1]][x[0]].passable == True, result))
        except:
            pass
        return result

    ## 4th paramater (find_attack_range) should be removed (or be always set to True)
    def get_coords_in_range(self, x_start, y_start, max_distance, find_attack_range=False) \
            -> list[tuple[int, int]]:
        outer_set, inner_set, new_set = set(), set(), set()

        self._updateGrid(find_attack_range)

        cur_dist = 0
        outer_set.add((x_start, y_start))
        while cur_dist < max_distance:
            inner_set.update(outer_set)
            for xy in outer_set:
                temp4debug = self.getNeighbors(xy[0], xy[1])
                asdf = set(temp4debug)
                new_set.update(asdf)

            # discard inner_set from new_set, not interested in exploring inner_set
            new_set.discard(inner_set)
            cur_dist += 1
            outer_set = new_set
            new_set = set()

        inner_set.update(outer_set)
        return list(inner_set)