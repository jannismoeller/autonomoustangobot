#ifndef AUTONOMOUSTANGOBOT_ASTAR_H
#define AUTONOMOUSTANGOBOT_ASTAR_H

#include <octomap/OcTreeKey.h>
#include <octomap/OcTree.h>
#include "NavigationNode.h"
#include "NavigationOcTree.h"
#include <unordered_map>
#include "AStarDataStructures.h"

using namespace std;
using namespace octomap;

namespace octomapcustomization {
    /**
     * class that provides static methods for pathfinding on an NavigationOcTree
     */
    class AStar {
    private:
        static constexpr float MOVE_STRAIGHT            = 1.0f;                     // sqrt(1)
        static constexpr float MOVE_DIAGONAL            = 1.4142135623730950488f;   // sqrt(2)
        static constexpr float MOVE_DIAGONAL_VERTICAL   = 1.7320508075688772935f;   // sqrt(3)
        static const float NEIGHBOR_COST[];

        static const int KEY_NEIGHBORS_0[];
        static const int KEY_NEIGHBORS_1[];

        /**
         * calculates an estimation of the shortest path between a and b
         */
        static inline float calcHeuristicCost(const OcTreeKey a, const OcTreeKey b) {
            // calculate the length of each side of the cuboid that both keys span
            int side0 = abs((int)a[0] - (int)b[0]);
            int side1 = abs((int)a[1] - (int)b[1]);
            int side2 = abs((int)a[2] - (int)b[2]);

            // sort the sides in ascending order by a simple unrolled bubble sort
            if(side0 > side1)
                swap(side0, side1);
            if(side1 > side2)
                swap(side1, side2);
            if(side0 > side1)
                swap(side0, side1);

            // in the shortest theoretically possible path with movement allowed
            // in STRAIGHT, DIAGONAL and DIAGONAL_VERTICAL direction
            // the shortest side will be the amount of cells traveled DIAGONAL_VERTICAL,
            // the difference between the shortest and the medium side will be travelled DIAGONAL
            // and the rest will be travelled STRAIGHT
            return side0 * MOVE_DIAGONAL_VERTICAL
                   + (side1 - side0) * MOVE_DIAGONAL
                   + (side2 - side1) * MOVE_STRAIGHT;
        }

        /**
         * @return key that equals the by x, y and z translated input key
         */
        static inline OcTreeKey translateKey(const OcTreeKey key,
                                             int xTranslation,
                                             int yTranslation,
                                             int zTranslation) {
            OcTreeKey newKey(key);
            newKey[0] += xTranslation;
            newKey[1] += yTranslation;
            newKey[2] += zTranslation;
            return newKey;
        }

        /**
         * after a path was found this methods steps backwards through all the path-nodes 
         * and extracts them to a list. Steps in the same direction get simplified 
         * to one single large step
         */
        static void extractPath(NavigationOcTree *const ocTree, AStarNode *const targetNode,
                                AStarPath **path) {
            vector<point3d> *wayPoints = new vector<point3d>;
            AStarNode *node = targetNode;
            double pathLength = 0.0;

            point3d lastCoord = ocTree->keyToCoord(node->key);
            lastCoord.z() = node->navigationNode->getNavigationInformation()->zCoord + (float)ocTree->getResolution();
            wayPoints->push_back(lastCoord);
            if(node->cameFromNode != NULL) {
                int lastKeyDeltaX = (int) node->cameFromNode->key[0] - (int) node->key[0];
                int lastKeyDeltaY = (int) node->cameFromNode->key[1] - (int) node->key[1];

                OcTreeKey lastKey = node->key;
                node = node->cameFromNode;

                while (node != NULL && node->cameFromNode != NULL) {
                    assert(node->key != lastKey);
                    if (lastKeyDeltaX != (int) node->cameFromNode->key[0] - (int) node->key[0] ||
                        lastKeyDeltaY != (int) node->cameFromNode->key[1] - (int) node->key[1] ||
                        node->cameFromNode->cameFromNode == NULL) {
                        point3d coord = ocTree->keyToCoord(node->key);
                        coord.z() = node->navigationNode->getNavigationInformation()->zCoord + (float)ocTree->getResolution();
                        wayPoints->push_back(coord);

                        pathLength += lastCoord.distance(coord);
                        lastCoord = coord;

                        lastKeyDeltaX = (int) node->cameFromNode->key[0] - (int) node->key[0];
                        lastKeyDeltaY = (int) node->cameFromNode->key[1] - (int) node->key[1];
                    }
                    assert(node != node->cameFromNode);
                    lastKey = node->key;
                    node = node->cameFromNode;
                }
            }
            
            // startnode != targetNode, 
            // distance between startnode and the second node needs to be added
            if(node != NULL){
                point3d coord = ocTree->keyToCoord(node->key);
                coord.z() = node->navigationNode->getNavigationInformation()->zCoord + (float)ocTree->getResolution();

                pathLength += lastCoord.distance(coord);
            }

            std::reverse(wayPoints->begin(),wayPoints->end());

            *path = new AStarPath(wayPoints, pathLength);
        }

    public:
        /**
         * Finds the shortest path between two keys
         * @return true if there is a path between the keys, false if not
         */
        static bool findShortestPath(NavigationOcTree *const ocTree,
                                     NavigationNode *const startNode,
                                     const OcTreeKey startKey, const OcTreeKey targetKey,
                                     AStarPath **path) {
            int dx = abs((int)startKey[0] - (int)targetKey[0]);
            int dy = abs((int)startKey[1] - (int)targetKey[1]);

            unsigned long openSetInitialHeapSize = (unsigned long)((dx + dy) * 3);
            
            AStarOpenSet openSet(openSetInitialHeapSize);
            AStarClosedSet closedSet;

            assert(ocTree->isNodeOccupied(startNode));
            assert(startNode->getNavigationInformation());
            
            openSet.add(new AStarNode(NULL, startKey, startNode, 0,
                                  calcHeuristicCost(startKey, targetKey)));

            bool pathExists = false;
            float res = (float)ocTree->getResolution();

            AStarNode *current;
            AStarNode *neighbor;
            while (!openSet.isEmpty()) {
                current = openSet.pop();
                closedSet.add(current);

                if(current->key == targetKey) {
                    // found a path

                    pathExists = true;
                    extractPath(ocTree, current, path);
                    __android_log_print(ANDROID_LOG_DEBUG , "OCTREE", "found a Path! targetNode Gcost is: %f Path contains %lu nodes and is of length: %f" , 
                                        current->getGCost(), 
                                        (*path)->wayPointList->size(), 
                                        (*path)->length);
                    break;
                }

                for (uint8_t i = 0; i < 8; ++i) {
                    if(current->navigationNode->getNeighbor(i) != NULL)
                    {
                        NavigationNode * neighborNode = current->navigationNode->getNeighbor(i);
                        OcTreeKey neighbourKey = translateKey(current->key,
                                                              KEY_NEIGHBORS_0[i],
                                                              KEY_NEIGHBORS_1[i],
                                                              0);
                        neighbourKey[2] = ocTree->coordToKey(neighborNode->getNavigationInformation()->zCoord);

                        assert(current->key != neighbourKey);
                        
                        if(closedSet.contains(neighbourKey)) {
                            continue;
                        }

                        float gCostNeighbor = current->getGCost() + NEIGHBOR_COST[i] * res;
                        if (!(neighbor = openSet.get(neighbourKey))) {
                            openSet.add(new AStarNode(current,
                                                  neighbourKey, neighborNode,
                                                  gCostNeighbor, calcHeuristicCost(neighbourKey, targetKey)));
                        }
                        else if (gCostNeighbor < neighbor->getGCost()) {
                            neighbor->setGCost(gCostNeighbor);
                            neighbor->cameFromNode = current;
                            openSet.updateNode(neighbor);
                        }
                    }
                }
            }

            point3d startCoords = ocTree->keyToCoord(startKey);
            point3d tragetCoords = ocTree->keyToCoord(targetKey);
            __android_log_print(ANDROID_LOG_DEBUG , "OCTREE", "Search of a Path successful: %d after visiting %lu nodes"
                                        "\n startCoords were: (%f ,%f ,%f)"
                                        "\n tragetCoords were: (%f ,%f ,%f)" , 
                                pathExists, closedSet.size(),
                                startCoords.x(), startCoords.y(), startCoords.z(),
                                tragetCoords.x(), tragetCoords.y(), tragetCoords.z());

            return pathExists;
        }
    }; // end class AStar

    const float AStar::NEIGHBOR_COST[8]{
            MOVE_STRAIGHT,
            MOVE_DIAGONAL,
            MOVE_STRAIGHT,
            MOVE_DIAGONAL,
            MOVE_STRAIGHT,
            MOVE_DIAGONAL,
            MOVE_STRAIGHT,
            MOVE_DIAGONAL
    };
    const int AStar::KEY_NEIGHBORS_0[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
    const int AStar::KEY_NEIGHBORS_1[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
    
} // end namespace octomapcustomization

#endif //AUTONOMOUSTANGOBOT_ASTAR_H
