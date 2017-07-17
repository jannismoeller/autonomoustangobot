#include <android/log.h>
#include <octomap/OcTree.h>
#include "NavigationOcTree.h"
#include "AStar.h"
#include "../octomapjni/GeometryHelper.h"

using namespace std;

namespace octomapcustomization {

    /// The following arrays are constant lookup-arrays 
    /// to make the tree-traversal easier and more efficient
    
    const point3d NavigationOcTree::DIRECTIONS[8] = {
            point3d(-1.0f, -1.0f, -1.0f),
            point3d(1.0f, -1.0f, -1.0f),
            point3d(-1.0f, 1.0f, -1.0f),
            point3d(1.0f, 1.0f, -1.0f),
            point3d(-1.0f, -1.0f, 1.0f),
            point3d(1.0f, -1.0f, 1.0f),
            point3d(-1.0f, 1.0f, 1.0f),
            point3d(1.0f, 1.0f, 1.0f)
    };

    const uint16_t NavigationOcTree::KEYS_0[8] = {0, 1, 0, 1, 0, 1, 0, 1};
    const uint16_t NavigationOcTree::KEYS_1[8] = {0, 0, 1, 1, 0, 0, 1, 1};
    const uint16_t NavigationOcTree::KEYS_2[8] = {0, 0, 0, 0, 1, 1, 1, 1};

    const int NavigationOcTree::KEY_NEIGHBORS_0[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
    const int NavigationOcTree::KEY_NEIGHBORS_1[8] = {0, -1, -1, -1, 0, 1, 1, 1};

    /**
     * Constructor, calls the base classes constructor
     */
    NavigationOcTree::NavigationOcTree(double resolution)
            : OccupancyOcTreeBase<NavigationNode>(resolution) {
        ocTreeMemberInit.ensureLinking();
    }

    /**
     * Destructor, calls the clear() method
     */
    NavigationOcTree::~NavigationOcTree() {
        clear();
    }

    /**
     * Method that deletes all nodes of the tree and deallcoates their memory
     */
    void NavigationOcTree::clear() {
        if (this->root) {
            deleteNavigationNodeRecurs(root);
            this->tree_size = 0;
            this->root = NULL;
            // max extent of tree changed:
            this->size_changed = true;
        }
    }

    /**
     * Similar to the base classes deleteNodeRecurs method. 
     * Makes sure the additional memory allocated for the navigation_information 
     * is deallocated correctly to prevent memory leaks
     */
    void NavigationOcTree::deleteNavigationNodeRecurs(NavigationNode *node) {
        assert(node);
        // TODO: maintain tree size?

        if (nodeHasChildren(node)) {
            for (unsigned int i = 0; i < 8; i++) {
                if (nodeChildExists(node, i)) {
                    this->deleteNavigationNodeRecurs(getNodeChild(node, i));
                }
            }
            node->deallocChildren();
        } // else: node has no children

        if (node->navInfo != NULL) {
            if (node->navInfo->neighbors != NULL) {
                node->disconnectNeighbors();
                node->deallocNeighbors();
            }
            node->deallocNavigationInformation();
        }

        delete node;
    }

    // needed for the AbstractOcTree factory
    NavigationOcTree::StaticMemberInitializer NavigationOcTree::ocTreeMemberInit;

    /**
     * Virtually overridden method to prevent occupied nodes from getting pruned
     */
    bool NavigationOcTree::isNodeCollapsible(const NavigationNode *node) const {
        // Debug assertion
        if (!isNodeOccupied(node))
            assert(node->navInfo == NULL);
        
        return !isNodeOccupied(node) && OcTreeBaseImpl::isNodeCollapsible(node);
    }
    
    /**
     * @return the OcTreeKey constructed from the minimum of each component of both parameter OcTreeKeys 
     */
    OcTreeKey NavigationOcTree::minKey(OcTreeKey a, OcTreeKey b) {
        return OcTreeKey(a[0] < b[0] ? a[0] : b[0],
                         a[1] < b[1] ? a[1] : b[1],
                         a[2] < b[2] ? a[2] : b[2]);
    }

    /**
     * @return the OcTreeKey constructed from the minimum of each component of both parameter OcTreeKeys 
     */
    OcTreeKey NavigationOcTree::maxKey(OcTreeKey a, OcTreeKey b) {
        return OcTreeKey(a[0] > b[0] ? a[0] : b[0],
                         a[1] > b[1] ? a[1] : b[1],
                         a[2] > b[2] ? a[2] : b[2]);
    }

    /**
     * Virtually overwritten method of the base class to insert a poincloud into the tree
     * changes to the original are the tracking of the changeBoundingBox
     */
    void NavigationOcTree::insertPointCloud(const Pointcloud &scan,
                                            const octomap::point3d &sensor_origin, double maxrange,
                                            bool lazy_eval, bool discretize) {
        KeySet free_cells, occupied_cells;
        if (discretize)
            computeDiscreteUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);
        else
            computeUpdate(scan, sensor_origin, free_cells, occupied_cells, maxrange);

        // insert data into tree  -----------------------
        for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
            OccupancyOcTreeBase::updateNode(*it, false, lazy_eval);
            changeBBXMin = minKey(*it, changeBBXMin);
            changeBBXMax = maxKey(*it, changeBBXMax);
        }

        double squaredMaxRange = maxrange * maxrange;
        OcTreeKey key;
        if (maxrange > 0) {
            for (int i = 0; i < (int) scan.size(); ++i) {
                if ((scan[i] - sensor_origin).norm_sq() <= squaredMaxRange
                    && this->coordToKeyChecked(scan[i], key)) {
                    updateNodeTrackChangeBBX(key, true, lazy_eval, scan[i].z());
                }
            }
        } else {
            for (int i = 0; i < (int) scan.size(); ++i) {
                if (this->coordToKeyChecked(scan[i], key))
                    updateNodeTrackChangeBBX(key, true, lazy_eval, scan[i].z());
            }
        }
    }

    /**
     * Insert the PointCloudData received from a Google Tango Callback into the OcTree
     */
    void NavigationOcTree::insertTangoPointCloud(jfloat *pFB, int numPoints, float m4x4[],
                                                 point3d sensorOrigin, double maxrange,
                                                 double minrange) {
        Pointcloud pointCloud;
        transFormAndFilterPoints(pFB, numPoints, m4x4, pointCloud, minrange);

        this->insertPointCloud(pointCloud, sensorOrigin, maxrange, false, true);

        hasChangedSinceLastNavigationUpdate = true;
    }

    /**
     * Transforms the tango pointcloud points in the floatbuffer that are in local space 
     * into global space by multiplying them with the transformation matrix.
     * Points that are too close to the camera get filtered out because they are most likely 
     * messurement artifacts
     */
    void NavigationOcTree::transFormAndFilterPoints(const jfloat *pFB, int numPoints,
                                                    const float *m4x4,
                                                    Pointcloud &pointCloud,
                                                    const double filterMinDistance) const {
        float p[3];

        for (int i = 0; i < numPoints * 4; i += 4) {
            // Sensor coordinate system is (y^,x->,z/^)
            if (pFB[i + 2] >= filterMinDistance) {

                // m4x4 x m4 Matrix multiplication to transform the point from device to global space
                // simplified to 3 components because the 4th is not needed in this case
                p[0] = m4x4[0] * pFB[i + 0] + m4x4[4] * pFB[i + 1] + m4x4[8] * pFB[i + 2] +
                       m4x4[12];
                p[1] = m4x4[1] * pFB[i + 0] + m4x4[5] * pFB[i + 1] + m4x4[9] * pFB[i + 2] +
                       m4x4[13];
                p[2] = m4x4[2] * pFB[i + 0] + m4x4[6] * pFB[i + 1] + m4x4[10] * pFB[i + 2] +
                       m4x4[14];

                pointCloud.push_back(p[0], p[1], p[2]);
            }
        }
    }

    /**
     * Checks if the bots circular ground plane is overlapping the cells circular shape
     */
    bool NavigationOcTree::isInRange(point3d botPosition, float botRadius, point3d cellCoordinate,
                                     float cellSize) {
        // Comparison is only needed for the XY-Projection of the points
        // The real distance is not needed, comparing the squared distances will yield the same result
        float xDist = botPosition.x() - cellCoordinate.x();
        float yDist = botPosition.y() - cellCoordinate.y();
        float maxDistance = botRadius + cellSize * 0.5f;
        return (xDist * xDist + yDist * yDist) <= (maxDistance * maxDistance);
    }

    /**
     * Tests if there is a collision between the OcTree and the defined cylinder
     */
    bool NavigationOcTree::testCollisionWithCylinder(point3d pos,
                                                     float radius, float height,
                                                     unsigned char maxDepth) {
        // Z / 3. Coordinate is pointing UP
        NavigationOcTree::leaf_bbx_iterator it = begin_leafs_bbx(
                point3d(pos.x() - radius, pos.y() - radius, pos.z() - 0),
                point3d(pos.x() + radius, pos.y() + radius, pos.z() + height),
                maxDepth);
        NavigationOcTree::leaf_bbx_iterator itEnd = end_leafs_bbx();

        for (; it != itEnd; it++) {
            if (isNodeOccupied((NavigationNode *) it.operator->()) &&
                isInRange(pos, radius, it.getCoordinate(), (float) it.getSize())) {
                return true;
            }
        }

        return false;
    }

    /**
     * @return the point3d constructed from the minimum of each component of both parameter points 
     */
    point3d NavigationOcTree::minCoordinatesPoint(point3d p1, point3d p2) {
        return point3d((p1.x() < p2.x()) ? p1.x() : p2.x(),
                       (p1.y() < p2.y()) ? p1.y() : p2.y(),
                       (p1.z() < p2.z()) ? p1.z() : p2.z());
    }

    /**
     * @return the point3d constructed from the maximum of each component of both parameter points 
     */
    point3d NavigationOcTree::maxCoordinatesPoint(point3d p1, point3d p2) {
        return point3d((p1.x() > p2.x()) ? p1.x() : p2.x(),
                       (p1.y() > p2.y()) ? p1.y() : p2.y(),
                       (p1.z() > p2.z()) ? p1.z() : p2.z());
    }

    /**
     * Method to fill a box by setting the values directly
     */
    void NavigationOcTree::fillBox(point3d from, point3d to, bool occupied) {
        point3d boxMin = minCoordinatesPoint(from, to);
        point3d boxMax = maxCoordinatesPoint(from, to);

        OcTreeKey boxMinKey, boxMaxKey;
        assert(coordToKeyChecked(boxMin, boxMinKey));
        assert(coordToKeyChecked(boxMax, boxMaxKey));

        OcTreeKey key;
        for (key[0] = boxMinKey[0]; key[0] <= boxMaxKey[0]; key[0]++) {
            for (key[1] = boxMinKey[1]; key[1] <= boxMaxKey[1]; key[1]++) {
                for (key[2] = boxMinKey[2]; key[2] <= boxMaxKey[2]; key[2]++) {
                    float newValue = occupied ?
                                     this->getClampingThresMaxLog() :
                                     this->getClampingThresMinLog();
                    setNodeValue(key, newValue, false, (float) keyToCoord(key[2]));
                }
            }
        }

        // Track changes
        changeBBXMax = maxKey(changeBBXMax, boxMaxKey);
        changeBBXMin = minKey(changeBBXMin, boxMinKey);

        hasChangedSinceLastNavigationUpdate = true;
    }

    /**
     * Method to fill a box by updating the values as if it was a measurement
     */
    void NavigationOcTree::fillBoxAsMeasurements(point3d from, point3d to, bool occupied) {
        point3d boxMin = minCoordinatesPoint(from, to);
        point3d boxMax = maxCoordinatesPoint(from, to);

        OcTreeKey boxMinKey, boxMaxKey;
        assert(coordToKeyChecked(boxMin, boxMinKey));
        assert(coordToKeyChecked(boxMax, boxMaxKey));

        OcTreeKey key;
        for (key[0] = boxMinKey[0]; key[0] <= boxMaxKey[0]; key[0]++) {
            for (key[1] = boxMinKey[1]; key[1] <= boxMaxKey[1]; key[1]++) {
                for (key[2] = boxMinKey[2]; key[2] <= boxMaxKey[2]; key[2]++) {
                    if (occupied)
                        updateNode(key, occupied, false, (float) keyToCoord(key[2]));
                    else
                        OccupancyOcTreeBase::updateNode(key, occupied);
                }
            }
        }

        // Track changes
        changeBBXMax = maxKey(changeBBXMax, boxMaxKey);
        changeBBXMin = minKey(changeBBXMin, boxMinKey);

        hasChangedSinceLastNavigationUpdate = true;
    }

    /**
     * Fill the buffer with the 3D-Positions of the occupied nodes by 
     * iterating the tree in a recursive manner
     */
    void
    NavigationOcTree::addNodeToBufferRecursive(jfloat **pFB, int *vertexCount, int bufferMaxVerts,
                                               NavigationNode *node, double nodeSize,
                                               point3d nodePos) {
        if (!this->isNodeOccupied(node)) {
            return;
        } else if (this->nodeHasChildren(node)) {
            double halfNodeSize = nodeSize / 2.0;
            double quarterNodeSize = halfNodeSize / 2.0;
            //add children by recursion
            for (unsigned int i = 0; i < 8 && *vertexCount < bufferMaxVerts; ++i) {
                if (this->nodeChildExists(node, i)) {
                    addNodeToBufferRecursive(pFB, vertexCount, bufferMaxVerts,
                                             this->getNodeChild(node, i),
                                             halfNodeSize,
                                             nodePos + (DIRECTIONS[i] * quarterNodeSize));
                }
            }
        } else {
            *((*pFB)++) = nodePos.x();
            *((*pFB)++) = nodePos.y();
            assert(node->navInfo);
            *((*pFB)++) = node->navInfo->zCoord;
            // for visualization of navigation information the 4th float of the vert (in XYZW Format)
            // is set to 1 if the node is navigable by the bot
            if (node->navInfo->neighborCount > 0) {
                if (node->navInfo->isFrontierNode)
                    *((*pFB)++) = 1.0f;
                else
                    *((*pFB)++) = 0.5f;
            } else {
                if (node->navInfo->neighbors != NULL) {
                    *((*pFB)++) = 3.0f;
                } else {
                    if (node->navInfo->isFrontierNode)
                        *((*pFB)++) = 2.0f;
                    else
                        *((*pFB)++) = 0.0f;
                }
            }

            (*vertexCount)++;
        }
    }

    /**
     * Fill the buffer with the 3D-Positions of the occupied nodes by 
     * iterating the tree manually for better performance
     */
    int NavigationOcTree::fillBufferWithNodePoints(jfloat *pFB, int bufferMaxVertsCount) {
        jfloat **ppFB = &pFB;

        NavigationNode *root = this->getRoot();

        if (!root) {
            return 0;
        }
        double rootNodeSize = this->getNodeSize(0);

        point3d rootPos(0, 0, 0);

        int vertCount = 0;

        // Iterate nodes recursively
        addNodeToBufferRecursive(ppFB, &vertCount, bufferMaxVertsCount,
                                 root, rootNodeSize, rootPos);

        return vertCount;
    }

    /**
     * Fill the buffer with the 3D-Positions of the occupied nodes by 
     * iterating the tree via the leaf-iterator that OctoMap provides
     */
    int NavigationOcTree::fillBufferWithNodePointsViaLeafIterator(jfloat *pFB,
                                                                  int bufferMaxVertsCount) {
        leaf_iterator it = this->begin_leafs();
        leaf_iterator itEnd = this->end_leafs();

        point3d nodePos;

        int vertCount = 0;
        int totalNodeCount = 0;
        for (; it != itEnd && vertCount < bufferMaxVertsCount; it++) {
            NavigationNode *node = (NavigationNode *) it.operator->();
            if (this->isNodeOccupied(node)) {
                nodePos = it.getCoordinate();
                pFB[vertCount * 4] = nodePos.x();
                pFB[vertCount * 4 + 1] = nodePos.y();
                pFB[vertCount * 4 + 2] = nodePos.z();
                // for visualization of navigation information the 4th float of the vert (in XYZW Format) 
                // is set to 1 if the node is navigable by the bot
                assert(node->navInfo);
                pFB[vertCount * 4 + 3] = (node->navInfo->neighborCount != 0) ? 1.0f : 0.0f;

                vertCount++;
            }
            totalNodeCount++;
        }

        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "TotalNodeCount: %d, iterator at end: %d",
                            totalNodeCount, (it == itEnd));
        return vertCount;
    }

    /**
     * Effitient seerch for the node at given searchedNodeKey
     * utilizing the key and the stack leading to the current node
     * The lower the common parent of the node and the targetNode are 
     * the more efficient is this method
     */
    void NavigationOcTree::searchFromCurrent(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                             OcTreeKey *currentNodeKey, int *currentDepth,
                                             OcTreeKey searchedNodeKey) {

        unsigned int depthDifferenceX = (*currentNodeKey)[0] ^searchedNodeKey[0];
        unsigned int depthDifferenceY = (*currentNodeKey)[1] ^searchedNodeKey[1];
        unsigned int depthDifferenceZ = (*currentNodeKey)[2] ^searchedNodeKey[2];

        unsigned int depthDifference = depthDifferenceX | depthDifferenceY;
        depthDifference |= depthDifferenceZ;

        int unusedKeyBitsInt32 = 32 - this->getTreeDepth();

        // use the gcc built in instruction to count leading zeros
        int parentDepth = __builtin_clz(depthDifference) - unusedKeyBitsInt32;

        // if the currentKey and the searchKey both lay somewhere in the current node they can have 
        // a common parent at a depth where no node exists. To prevent trying to access that node we set:
        if (parentDepth > *currentDepth)
            parentDepth = *currentDepth;

        assert(parentDepth >= 0);
        assert(parentDepth < 17);
        // If parentDepth == 16 the searchedKey is the same as the currentKey
        assert(parentDepth != 16);
        assert(parentDepth <= *currentDepth);
        assert((*stackToCurrentNode)[parentDepth] != NULL);

        while (this->nodeHasChildren((*stackToCurrentNode)[parentDepth])) {
            uint8_t childIndex = computeChildIdx(searchedNodeKey,
                                                 this->getTreeDepth() - 1 - parentDepth);

            if (!this->nodeChildExists((*stackToCurrentNode)[parentDepth], childIndex)) {
                //Handle unknown Space
                (*stackToCurrentNode)[++parentDepth] = NULL;
                break;
            }

            (*stackToCurrentNode)[parentDepth + 1] = this->getNodeChild(
                    (*stackToCurrentNode)[parentDepth], childIndex);
            parentDepth++;
        }
        *currentDepth = parentDepth;
        *currentNodeKey = searchedNodeKey;
    }

    /**
     * Effitient seerch for the direkt upper neighbor of a node 
     * utilizing the stack leading to the current node
     * WARNING: Boundries of OcTree are unchecked
     */
    void NavigationOcTree::getUpperNeighbor(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                            OcTreeKey *currentNodeKey,
                                            int *currentDepth) {
        // y-Component of the new key 
        // shifting to skip "returning" the same node again when the node is pruned
        uint16_t newYComponent =
                (*currentNodeKey)[2] + ((uint16_t) 1 << (this->getTreeDepth() - *currentDepth));

        // Bitwise XOR to extract the digits that changed
        unsigned int depthDifference = newYComponent ^(*currentNodeKey)[2];
        // using the gcc built in method count leading zeros
        // calculate the depth of the lowest common parent of the node and its neighbor
        // because it is performed on 32 bit we have to subtract 16
        int parentDepth = __builtin_clz(depthDifference) - this->getTreeDepth();
        assert(parentDepth <= *currentDepth);
        // Next line not necessary because we added 1 at currentDepth to the key,
        // so the lowest common parent can only be lower than the currentDepth
//        parentDepth = (parentDepth < *currentDepth) ? parentDepth : *currentDepth;
        (*currentNodeKey)[2] = newYComponent;

        while (this->nodeHasChildren((*stackToCurrentNode)[parentDepth])) {
            uint8_t childIndex = computeChildIdx((*currentNodeKey),
                                                 this->getTreeDepth() - 1 - parentDepth);

            if (!this->nodeChildExists((*stackToCurrentNode)[parentDepth], childIndex)) {
                //Handle unknown Space
                ++parentDepth;
                (*stackToCurrentNode)[parentDepth] = NULL;
                break;
            }

            (*stackToCurrentNode)[parentDepth + 1] = this->getNodeChild(
                    (*stackToCurrentNode)[parentDepth], childIndex);
            parentDepth++;
        }
        *currentDepth = parentDepth;
    }

    /**
     * The upper neighbors of a node are visited in an iterative manner
     * and checked for occupancy/unknown space
     * This version of the overloaded Mehtod already takes an existing nodestack 
     * and thus is more effitient because it doesnt need to build up the stack itself first
     * WARNING: Boundries of OcTree are unchecked
     * @return 0 for all cells free, 1 for >=1 cell occupied, 2 for >=1 cell unknown space
     */
    int NavigationOcTree::raySteppingUp(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                        OcTreeKey *currentNodeKey, int *currentDepth,
                                        double maxZCoord) {
        if (this->isNodeOccupied((*stackToCurrentNode)[*currentDepth])) {
            //Handle occupied Space: stop if start node is occupied
            return 1;
        }

        // Cut off the last few bits of depth that where not reached 
        // to set the key to the lower border of the current cell
        (*currentNodeKey)[2] &= (0xFFFF << (this->getTreeDepth() - *currentDepth));

        // this needs to be subtracted to get the real lower border of the nodes space
        double smallestSizeHalf = this->getResolution() / 2;

        while ((this->keyToCoord((*currentNodeKey)[2])
                + this->getNodeSize((unsigned int) (*currentDepth))
                - smallestSizeHalf)
               <= maxZCoord) {
            getUpperNeighbor(stackToCurrentNode, currentNodeKey, currentDepth);
            if ((*stackToCurrentNode)[*currentDepth] == NULL)
                return 2;
            else if (this->isNodeOccupied((*stackToCurrentNode)[*currentDepth]))
                return 1;
        }

        return 0;
    }

    /**
     * The upper neighbors of a node are visited in an iterative manner
     * and checked for occupancy/unknown space
     * WARNING: Boundries of OcTree are unchecked
     * @return 0 for all cells free, 1 for >=1 cell occupied, 2 for >=1 cell unknown space
     */
    int NavigationOcTree::raySteppingUp(OcTreeKey currentNodeKey, double maxZCoord) {

        array<NavigationNode *, stackSize> stackToCurrentNode;
        // Reserving space for root node and all 16 levels under it
        int currentDepth = 0;

        // built stack to the start node:
        stackToCurrentNode[0] = this->getRoot();

        while (this->nodeHasChildren(stackToCurrentNode[currentDepth])) {
            uint8_t childIndex = computeChildIdx(currentNodeKey,
                                                 this->getTreeDepth() - 1 - currentDepth);

            if (!this->nodeChildExists(stackToCurrentNode[currentDepth], childIndex)) {
                //Handle unknown Space: stop if start node is in unknown space
                return 2;
            }

            stackToCurrentNode[currentDepth + 1] = this->getNodeChild(
                    stackToCurrentNode[currentDepth], childIndex);
            currentDepth++;
        }

        if (this->isNodeOccupied(stackToCurrentNode[currentDepth])) {
            //Handle occupied Space: stop if start node is occupied
            return 1;
        }

        // Cut off the last few bits of depth that where not reached 
        // to set the key to the lower border of the current cell
        currentNodeKey[2] &= (0xFFFF << (this->getTreeDepth() - currentDepth));

        // this needs to be subtracted to get the real lower border of the nodes space
        double smallestSizeHalf = this->getResolution() / 2;

        // while the next theoretical neighbor position still is under the maxZCoord
        // step further up
        while ((this->keyToCoord(currentNodeKey[2])
                + this->getNodeSize((unsigned int) currentDepth)
                - smallestSizeHalf)
               <= maxZCoord) {
            getUpperNeighbor(&stackToCurrentNode, &currentNodeKey, &currentDepth);
            if (stackToCurrentNode[currentDepth] == NULL)
                return 2;
            else if (this->isNodeOccupied(stackToCurrentNode[currentDepth]))
                return 1;
        }

        return 0;
    }

    /**
     * Navigation-Update Step 1:
     * Determine the occupied nodes of the tree that have enoguh free space above them 
     * to fit in the bot hight-wise
     */
    void NavigationOcTree::handleNodeFindNodesWithFreeSpaceAbove(
            array<NavigationNode *, stackSize> *stackToCurrentNode,
            OcTreeKey currentNodeKey, int currentDepth, float,
            float botHeight) {
        debug_total_leafnode_count++;

        // early out for free-space nodes
        if (!isNodeOccupied((*stackToCurrentNode)[currentDepth]))
            return;

        // all occupied leaf-nodes should have the depth of 16 
        // because they are not allowed to be pruned
        assert(currentDepth == 16);

        array<NavigationNode *, stackSize> nodeStackCopy = *stackToCurrentNode;


        NavigationNode *node = nodeStackCopy[currentDepth];

        getUpperNeighbor(&nodeStackCopy, &currentNodeKey, &currentDepth);
        NavigationNode *nodeNeighbor = nodeStackCopy[currentDepth];

        debug_total_occupied_leafnode_count++;
        assert(node->navInfo);
        double maxZCoord = node->navInfo->zCoord + botHeight;
        if (nodeNeighbor == NULL) {
            // set neighbor-pointers and pointers of neighbors to this node to null
            if (node->navInfo->neighborCount != 0) {
                node->disconnectNeighbors();
                node->deallocNeighbors();
            }

            node->navInfo->hasFreeSpaceAbove = false;
            debug_neighbor_NULL_count++;
            node->navInfo->isFrontierNode = false;
        } else {
            int rayResult = raySteppingUp(&nodeStackCopy, &currentNodeKey, &currentDepth,
                                          maxZCoord);
            if (rayResult == 0) {
                node->navInfo->hasFreeSpaceAbove = true;
                debug_navigatable_node_count++;
                node->navInfo->isFrontierNode = false;
            } else {
                // set neighbor-pointers and pointers of neighbors to this node to null
                if (node->navInfo->neighborCount != 0) {
                    node->disconnectNeighbors();
                    node->deallocNeighbors();
                }
                debug_neighbor_occupied_count++;
                node->navInfo->hasFreeSpaceAbove = false;
                // if the ray detected a unknown space cell after the first few free ones
                // the node lays on (under) the frontier of unknown space
                node->navInfo->isFrontierNode = (rayResult == 2);
            }
        }
    }

    /**
     * Navigation-Update Step 2:
     * Updating the neighbor relations between nodes with hasFreeSpaceAbove 
     * by efficiently checking the possible neighbor-positions
     */
    void NavigationOcTree::handleNodeFindNeighbors(
            array<NavigationNode *, stackSize> *stackToCurrentNode,
            OcTreeKey currentNodeKey, int currentDepth,
            float stepHeight, float) {
        NavigationNode *node = (*stackToCurrentNode)[currentDepth];
        // early out for free-space nodes and nodes that are not navigable
        if (!isNodeOccupied(node))
            return;
        assert(node->navInfo);
        if (!node->navInfo->hasFreeSpaceAbove) {
            assert(node->navInfo->neighborCount == 0);
            return;
        }

        array<NavigationNode *, stackSize> nodeStackCopy = *stackToCurrentNode;


        // Allocate neighbors array if necessary
        if (node->navInfo->neighbors == NULL || node->navInfo->neighborCount == 0) {
            assert(node->navInfo->neighborCount == 0);
            assert(node->navInfo->neighbors == NULL);
            assert(node->navInfo->hasFreeSpaceAbove);
            assert(!node->navInfo->isFrontierNode);
            node->allocNeighbors();
        }

        int neighborCurrentDepth = currentDepth;

        OcTreeKey oldNeighborKey = currentNodeKey;
        OcTreeKey newNeighborKey;

        NavigationNode *neighbor;

        // check every possible neighbor
        for (uint8_t i = 0; i < 8; ++i) {
            bool debug_connected_neighbar_newly = false;
            // Only search for neighbor if the neighbor didn't search and find this node first
            if (node->navInfo->neighbors[i] == NULL) {
                debug_connected_neighbar_newly = true;
                newNeighborKey[0] = (uint16_t) (currentNodeKey[0] + KEY_NEIGHBORS_0[i]);
                newNeighborKey[1] = (uint16_t) (currentNodeKey[1] + KEY_NEIGHBORS_1[i]);
                int neighborUnknownSpaceCount = 0;
                // start with the theoretical neighbor position that is 1 cell higher than this node
                // then go down
                for (int a = 1; a >= -1; --a) {

                    newNeighborKey[2] = (uint16_t) (currentNodeKey[2] + a);

                    searchFromCurrent(&nodeStackCopy, &oldNeighborKey, &neighborCurrentDepth,
                                      newNeighborKey);
                    debug_searchFromCurrent_count++;

                    neighbor = nodeStackCopy[neighborCurrentDepth];

                    if (neighbor != NULL) {
                        if (isNodeOccupied(neighbor)) {
                            if (neighbor->navInfo->hasFreeSpaceAbove &&
                                abs(node->navInfo->zCoord - neighbor->navInfo->zCoord) <=
                                stepHeight) {
                                // Neighbor is navigable 
                                // and also reachable taking the stepHeight into account
                                neighborUnknownSpaceCount = 0;

                                // Add neighbor to nodes neighbor-array
                                node->setNeighbor(i, neighbor);

                                // Allocate memory
                                if (neighbor->navInfo->neighbors == NULL)
                                    neighbor->allocNeighbors();

                                // Add node to neighbors neighbor-array
                                neighbor->setNeighbor(NavigationNode::getOppositeNeighborIdx(i),
                                                      node);
                            } else {
                                // the neighbor doesn't have enough free space above him, 
                                // but if he is on the frontier, this node should be too
                                if (neighbor->navInfo->isFrontierNode)
                                    neighborUnknownSpaceCount = 3;
                            }
                            // Neighbor was occupied, whether or not he had enough free space above him
                            // there won't be any other visitable ones underneath him
                            break;
                        }
                    } else { // handle neighbor in unknown space
                        neighborUnknownSpaceCount++;
                        // to move the pointer to a non null "stack-top" again we have to decrement the currentDepth
                        neighborCurrentDepth--;
                    }
                }
                // if at least a certain number neighbor positions were free and no neighbor with free space above found, this node is on the frontier
                if (neighborUnknownSpaceCount >= 2)
                    node->navInfo->isFrontierNode = true;
            }

            // Debug assertions to check the integrity of the neighborhood
            if (node->navInfo->neighbors[i] != NULL) {
                assert(node->navInfo->neighbors[i]->navInfo);
                assert(node->navInfo->neighbors[i]->navInfo->neighborCount != 0);
                assert(node->navInfo->neighbors[i]->navInfo->neighbors);
                assert(node == node->navInfo->neighbors[i]->navInfo->neighbors[
                                       NavigationNode::getOppositeNeighborIdx(i)]);
            }
        }

        if (node->navInfo->neighborCount == 0) {
            node->deallocNeighbors();
        }
    }

    /**
     * Navigation-Update Step 3:
     * Reduction step of nodes that are at the borders of the neighbor-net
     */
    void NavigationOcTree::handleNodeReduceNeighbors(
            array<NavigationNode *, stackSize> *stackToCurrentNode,
            OcTreeKey, int currentDepth, float, float) {

        assert(stackToCurrentNode != NULL);
        NavigationNode *node = (*stackToCurrentNode)[currentDepth];
        assert(node != NULL);
        if (isNodeOccupied(node))
            assert(node->navInfo);
        else
            assert(node->navInfo == NULL);
        // early out for free-space nodes and non navigable nodes, as these don't have neighbors
        if (!isNodeOccupied(node) ||
            !node->navInfo->hasFreeSpaceAbove ||
            node->navInfo->neighborCount == 0) {
            if (node->navInfo && !node->navInfo->hasFreeSpaceAbove) {
                assert(!node->navInfo->isFrontierNode);
            }
            if (node->navInfo && node->navInfo->neighborCount == 0)
                assert(!node->navInfo->isFrontierNode);
            if (node->navInfo)
                node->navInfo->isFrontierNode = false;
            return;
        }

        assert(node->navInfo->neighbors);
        assert(node->navInfo->hasFreeSpaceAbove);

        int nullNeighborCount = 8 - node->navInfo->neighborCount;

        // only reduce this node if it has more "null-neighbors" 
        // than null neighbors from the current reduction step
        assert(node->navInfo->removedNeighborCount <= nullNeighborCount);
        // never set this value < 1
        int nullNeighborsNecessaryForReduction = 2;
        if (nullNeighborCount >=
            node->navInfo->removedNeighborCount + nullNeighborsNecessaryForReduction) {

            NavigationNode *neighbor;
            for (uint8_t i = 0; i < 8; ++i) {
                neighbor = node->navInfo->neighbors[i];
                if (neighbor != NULL) {
                    assert(neighbor->navInfo);
                    assert(neighbor->navInfo->neighbors);
                    assert(neighbor->navInfo->neighbors[NavigationNode::getOppositeNeighborIdx(
                            i)] == node);
                    assert(neighbor->navInfo->neighborCount > 0);
                    int debug_neighbor_neighborcount = neighbor->navInfo->neighborCount;
                    int debug_neighbor_removedneighborcount = neighbor->navInfo->removedNeighborCount;
                    neighbor->removeNeighbor(NavigationNode::getOppositeNeighborIdx(i));
                    if (node->navInfo->wasFrontierNode)
                        neighbor->navInfo->isFrontierNode = true;
                    assert(neighbor->navInfo->neighborCount == debug_neighbor_neighborcount - 1);
                    assert(neighbor->navInfo->removedNeighborCount ==
                           debug_neighbor_removedneighborcount + 1);

                    // if this node was the neighbors last neighbor, 
                    // remove the neighbor array of the neighbor
                    if (neighbor->navInfo->neighborCount == 0) {
                        neighbor->deallocNeighbors();
                        neighbor->navInfo->isFrontierNode = false;
                    }

                    node->removeNeighbor(i);
                    // if this was the last neighbor of this node, remove the neighbor array 
                    // and stop the iteration
                    if (node->navInfo->neighborCount == 0) {
                        node->deallocNeighbors();
                        break;
                    }
                }
            }
            node->navInfo->isFrontierNode = false;
        }
    }

    /**
     * Reset the fields of all nodes that are necessary for the simulation of a cellular automate
     */
    void NavigationOcTree::handleNodeResetCellularFields(
            array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey,
            int currentDepth, float, float) {
        NavigationNode *node = (*stackToCurrentNode)[currentDepth];
        if (node->navInfo != NULL) {
            node->navInfo->removedNeighborCount = 0;
            node->navInfo->wasFrontierNode = node->navInfo->isFrontierNode;
        }
    }

    /** 
     * removes the frontier property for all nodes that have 8 or 0 neighbors 
     * or dont have enough FreeSpaceAbove
     */
    void NavigationOcTree::handleNodeResetFrontier(
            array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey,
            int currentDepth, float, float) {
        NavigationNode *node = (*stackToCurrentNode)[currentDepth];
        if (node->navInfo != NULL)
            assert(!(node->navInfo->neighborCount == 8 && node->navInfo->isFrontierNode));

        if (node->navInfo != NULL &&
            (node->navInfo->neighborCount == 8 || !node->navInfo->hasFreeSpaceAbove)) {
            node->navInfo->isFrontierNode = false;
        }
        if (node->navInfo != NULL && node->navInfo->neighborCount == 0)
            node->navInfo->isFrontierNode = false;

        // only for debugging purposes
        if (node->navInfo) {
            if (!node->navInfo->hasFreeSpaceAbove) {
                assert(node->navInfo->neighborCount <= 0);
                assert(!node->navInfo->isFrontierNode);
            }
            if (node->navInfo->neighborCount > 0) {
                assert(node->navInfo->hasFreeSpaceAbove);
            }
            if (node->navInfo->isFrontierNode) {
                assert(node->navInfo->hasFreeSpaceAbove);
            }
        }
    }

    /** 
     * @return true if key1 only has components that are smaller than their counterpart of key2
     */
    inline
    bool NavigationOcTree::keyAllComponentsSmallerOrEqual(OcTreeKey key1, OcTreeKey key2) {
        return key1.k[0] <= key2.k[0] &&
               key1.k[1] <= key2.k[1] &&
               key1.k[2] <= key2.k[2];
    }

    /** 
     * @return true if key1 only has components that are bigger or equal than their counterpart of key2
     */
    inline
    bool NavigationOcTree::keyAllComponentsBiggerOrEqual(OcTreeKey key1, OcTreeKey key2) {
        return key1.k[0] >= key2.k[0] &&
               key1.k[1] >= key2.k[1] &&
               key1.k[2] >= key2.k[2];
    }

    /**
     * Recursive Part of the bounding box iteration withOUT further checking 
     * of the bounding box boreders (more efficient)
     */
    void
    NavigationOcTree::iterateLeafsRecursive(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                            OcTreeKey currentNodeKey, int currentDepth,
                                            int currentShiftsFromBottom,
                                            void (NavigationOcTree::*handleNode)(
                                                    array<NavigationNode *, stackSize> *stackToCurrentNode,
                                                    OcTreeKey,
                                                    int currentDepth, float stepHeight,
                                                    float botHeight),
                                            float stepHeight, float botHeight) {
        NavigationNode *node = (*stackToCurrentNode)[currentDepth];
        if (this->nodeHasChildren(node)) {

            currentDepth++;
            currentShiftsFromBottom--;

            for (unsigned int i = 0; i < 8; ++i) {
                if (this->nodeChildExists(node, i)) {
                    (*stackToCurrentNode)[currentDepth] = getNodeChild(node, i);
                    OcTreeKey childNodeKey(currentNodeKey);

                    uint16_t keyLocal0 = KEYS_0[i] << currentShiftsFromBottom;
                    uint16_t keyLocal1 = KEYS_1[i] << currentShiftsFromBottom;
                    uint16_t keyLocal2 = KEYS_2[i] << currentShiftsFromBottom;

                    childNodeKey[0] |= keyLocal0;
                    childNodeKey[1] |= keyLocal1;
                    childNodeKey[2] |= keyLocal2;

                    iterateLeafsRecursive(stackToCurrentNode, childNodeKey, currentDepth,
                                          currentShiftsFromBottom, handleNode, stepHeight,
                                          botHeight);
                }
            }
        } else {
            (this->*handleNode)(stackToCurrentNode, currentNodeKey, currentDepth, stepHeight,
                                botHeight);
        }
    }

    /**
     * Recursive Part of the bounding box iteration with further checking 
     * of the bounding box boreders
     */
    void NavigationOcTree::iterateBoundingBoxRecursive(
            array<NavigationNode *, stackSize> *stackToCurrentNode,
            int currentDepth, int currentShiftsFromBottom,
            OcTreeKey minKeyBBX, OcTreeKey maxKeyBBX,
            OcTreeKey minKeyNode, OcTreeKey maxKeyNode,
            void (NavigationOcTree::*handleNode)(
                    array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey,
                    int currentDepth, float stepHeight, float botHeight),
            float stepHeight, float botHeight) {
        NavigationNode *node = (*stackToCurrentNode)[currentDepth];
        if (this->nodeHasChildren(node)) {

            if (keyAllComponentsBiggerOrEqual(maxKeyNode, minKeyBBX) &&
                keyAllComponentsSmallerOrEqual(minKeyNode, maxKeyBBX)) { // Node is overlapping

                // Testing if further simplification is applicable
                if (keyAllComponentsBiggerOrEqual(minKeyNode, minKeyBBX) &&
                    keyAllComponentsSmallerOrEqual(maxKeyNode,
                                                   maxKeyBBX)) {// Node completely in BBX

                    currentDepth++;
                    currentShiftsFromBottom--;

                    // Iterate children without further checking
                    for (unsigned int i = 0; i < 8; ++i) {
                        if (this->nodeChildExists(node, i)) {
                            (*stackToCurrentNode)[currentDepth] = getNodeChild(node, i);

                            OcTreeKey childKey(minKeyNode);

                            uint16_t keyLocal0 = KEYS_0[i] << currentShiftsFromBottom;
                            uint16_t keyLocal1 = KEYS_1[i] << currentShiftsFromBottom;
                            uint16_t keyLocal2 = KEYS_2[i] << currentShiftsFromBottom;

                            // because the bit we want to set is always 0 on the minKey, 
                            // bitwise OR will set it to our value
                            childKey[0] |= keyLocal0;
                            childKey[1] |= keyLocal1;
                            childKey[2] |= keyLocal2;

                            iterateLeafsRecursive(stackToCurrentNode, childKey, currentDepth,
                                                  currentShiftsFromBottom, handleNode,
                                                  stepHeight, botHeight);
                        }
                    }
                } else { // Node partially overlapping the BBX
                    currentDepth++;
                    currentShiftsFromBottom--;

                    // Iterate children with checking
                    for (unsigned int i = 0; i < 8; ++i) {
                        if (this->nodeChildExists(node, i)) {
                            (*stackToCurrentNode)[currentDepth] = getNodeChild(node, i);

                            OcTreeKey childMinKey(minKeyNode), childMaxKey(maxKeyNode);

                            uint16_t keyLocal0 = KEYS_0[i] << currentShiftsFromBottom;
                            uint16_t keyLocal1 = KEYS_1[i] << currentShiftsFromBottom;
                            uint16_t keyLocal2 = KEYS_2[i] << currentShiftsFromBottom;

                            uint16_t bitmask = ~((uint16_t) 1 << currentShiftsFromBottom);

                            // because the bit we want to set is always 0 on the minKey, 
                            // bitwise OR will set it to our value
                            childMinKey[0] |= keyLocal0;
                            childMinKey[1] |= keyLocal1;
                            childMinKey[2] |= keyLocal2;
                            // on the maxKey the bit is always 1,
                            // so bitwise AND and a bitmask will set it to 0,
                            // so that the following bitwise OR will set it to our value
                            childMaxKey[0] = (childMaxKey[0] & bitmask) | keyLocal0;
                            childMaxKey[1] = (childMaxKey[1] & bitmask) | keyLocal1;
                            childMaxKey[2] = (childMaxKey[2] & bitmask) | keyLocal2;

                            iterateBoundingBoxRecursive(stackToCurrentNode, currentDepth,
                                                        currentShiftsFromBottom,
                                                        minKeyBBX, maxKeyBBX, childMinKey,
                                                        childMaxKey, handleNode,
                                                        stepHeight, botHeight);
                        }
                    }
                }
            }
            // else Node completely outside of BBX

        } else {
            (this->*handleNode)(stackToCurrentNode, minKeyNode, currentDepth, stepHeight,
                                botHeight);
        }
    }

    /**
     * Function that allows the user to hand in a function pointer to a method 
     * that will be called for every leaf Node
     */
    void NavigationOcTree::iterateBoundingBox(OcTreeKey minKeyBBX, OcTreeKey maxKeyBBX,
                                              void (NavigationOcTree::*handleNode)(
                                                      array<NavigationNode *, stackSize> *stackToCurrentNode,
                                                      OcTreeKey,
                                                      int currentDepth, float stepHeight,
                                                      float botHeight),
                                              float stepHeight, float botHeight) {
        // Tree empty
        if (getRoot() == NULL)
            return;

        assert(minKeyBBX[0] <= maxKeyBBX[0]);
        assert(minKeyBBX[1] <= maxKeyBBX[1]);
        assert(minKeyBBX[2] <= maxKeyBBX[2]);

        // bitwise XOR component wise followed by bitwise OR to find the first depth:
        OcTreeKey xorKeys(minKeyBBX[0] ^ maxKeyBBX[0],
                          minKeyBBX[1] ^ maxKeyBBX[1],
                          minKeyBBX[2] ^ maxKeyBBX[2]);
        uint16_t orKeyComponets = xorKeys[0] | xorKeys[1] | xorKeys[2];

        int unusedKeyBitsInt32 = 32 - this->getTreeDepth();

        int leadingZeros = __builtin_clz(orKeyComponets) - unusedKeyBitsInt32;

        OcTreeKey minKeyNode, maxKeyNode;

        uint16_t bitmaskCommanPath = ((uint16_t) 0xFFFF << (this->getTreeDepth() - leadingZeros));

        minKeyNode[0] = maxKeyNode[0] = bitmaskCommanPath & minKeyBBX[0];
        minKeyNode[1] = maxKeyNode[1] = bitmaskCommanPath & minKeyBBX[1];
        minKeyNode[2] = maxKeyNode[2] = bitmaskCommanPath & minKeyBBX[2];

        uint16_t bitmaskTrailingOnes = (uint16_t) 0xFFFF >> leadingZeros;

        maxKeyNode[0] |= bitmaskTrailingOnes;
        maxKeyNode[1] |= bitmaskTrailingOnes;
        maxKeyNode[2] |= bitmaskTrailingOnes;


        array<NavigationNode *, stackSize> stackToParent;
        // Reserving space for root node and all 16 levels under it
        int currentDepth = 0;
        stackToParent[currentDepth] = this->getRoot();

        // Get Common Parent node, as far down the tree as possible
        for (; currentDepth < leadingZeros &&
               this->nodeHasChildren(stackToParent[currentDepth]); ++currentDepth) {
            uint8_t childIndex = computeChildIdx(minKeyNode,
                                                 this->getTreeDepth() - 1 - currentDepth);
            if (this->nodeChildExists(stackToParent[currentDepth], childIndex))
                stackToParent[currentDepth + 1] = this->getNodeChild(stackToParent[currentDepth],
                                                                     childIndex);
            else {
                // Handle early out for empty BBX
            }
        }

        iterateBoundingBoxRecursive(&stackToParent, currentDepth,
                                    this->getTreeDepth() - currentDepth,
                                    minKeyBBX, maxKeyBBX, minKeyNode, maxKeyNode, handleNode,
                                    stepHeight, botHeight);
    }

    /**
     * virtually overwritten method to convert the node from occupied to free or the other way around
     */
    void
    NavigationOcTree::updateNodeLogOdds(NavigationNode *occupancyNode, const float &update) const {
        bool isNodeOccupiedBeforeUpdate = occupancyNode->navInfo != NULL;
        OccupancyOcTreeBase::updateNodeLogOdds(occupancyNode, update);

        if (isNodeOccupiedBeforeUpdate && !isNodeOccupied(occupancyNode)) {
            occupancyNode->convertToFreeNode();
        } else if (!isNodeOccupiedBeforeUpdate && isNodeOccupied(occupancyNode))
            occupancyNode->convertToOccupiedNode();
    }

    /**
     * Exact copy of the base classes method, necessary because the authors of Octomap 
     * didn't make "setNodeValueRecurs" a virtual method
     */
    NavigationNode *
    NavigationOcTree::setNodeValue(const OcTreeKey &key, float log_odds_value, bool lazy_eval,
                                   float zCoord) {
        // clamp log odds within range:
        log_odds_value = std::min(std::max(log_odds_value, this->clamping_thres_min),
                                  this->clamping_thres_max);

        bool createdRoot = false;
        if (this->root == NULL) {
            this->root = new NavigationNode();
            this->tree_size++;
            createdRoot = true;
        }

        return setNodeValueRecurs(this->root, createdRoot, key, 0, log_odds_value, lazy_eval,
                                  zCoord);
    }

    /**
     * Mostly a copy of the base classes method, edits for keeping the integrity 
     * of the NavigationNodes are marked by comments
     */
    NavigationNode *
    NavigationOcTree::setNodeValueRecurs(NavigationNode *node, bool node_just_created,
                                         const OcTreeKey &key,
                                         unsigned int depth, const float &log_odds_value,
                                         bool lazy_eval,
                                         float zCoord) {
        bool created_node = false;

        assert(node);

        // follow down to last level
        if (depth < this->tree_depth) {
            unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
            if (!this->nodeChildExists(node, pos)) {
                // child does not exist, but maybe it's a pruned node?
                if (!this->nodeHasChildren(node) && !node_just_created) {
                    // current node does not have children AND it is not a new node
                    // -> expand pruned node
                    this->expandNode(node);
                } else {
                    // not a pruned node, instantiate requested child
                    this->createNodeChild(node, pos);
                    created_node = true;
                }
            }

            if (lazy_eval)
                return setNodeValueRecurs(this->getNodeChild(node, pos), created_node, key,
                                          depth + 1, log_odds_value, lazy_eval, zCoord);
            else {
                NavigationNode *retval = setNodeValueRecurs(this->getNodeChild(node, pos),
                                                            created_node, key, depth + 1,
                                                            log_odds_value, lazy_eval, zCoord);
                // prune node if possible, otherwise set own probability
                // note: combining both did not lead to a speedup!
                if (this->pruneNode(node)) {
                    // return pointer to current parent (pruned), the just updated node no longer exists
                    retval = node;
                } else {
                    node->updateOccupancyChildren();
                }

                return retval;
            }
        }

            // at last level, update node, end of recursion
        else {
            if (use_change_detection) {
                bool occBefore = this->isNodeOccupied(node);
                node->setLogOdds(log_odds_value);
// change from original code:
                if (isNodeOccupied(node)) {
                    if (node->navInfo == NULL)
                        node->allocNavigationInformation();
                    if (node->navInfo->zCoord == 0)
                        node->navInfo->zCoord = zCoord;
                    else
                        updateZCoord(node, zCoord);
                } else if (node->navInfo != NULL) {
                    node->convertToFreeNode();
                }
// end change

                if (node_just_created) {  // new node
                    changed_keys.insert(std::pair<OcTreeKey, bool>(key, true));
                } else if (occBefore !=
                           this->isNodeOccupied(node)) {  // occupancy changed, track it
                    KeyBoolMap::iterator it = changed_keys.find(key);
                    if (it == changed_keys.end())
                        changed_keys.insert(std::pair<OcTreeKey, bool>(key, false));
                    else if (it->second == false)
                        changed_keys.erase(it);
                }
            } else {
                node->setLogOdds(log_odds_value);
// change from original code:
                if (isNodeOccupied(node)) {
                    if (node->navInfo == NULL)
                        node->allocNavigationInformation();
                    if (node->navInfo->zCoord == 0)
                        node->navInfo->zCoord = zCoord;
                    else
                        updateZCoord(node, zCoord);
                } else if (node->navInfo != NULL) {
                    node->convertToFreeNode();
                }
// end change
            }
            return node;
        }
    }

    /**
     * Small wrapper of the updateNode method that keeps track of the change Bounding Box
     */
    NavigationNode *NavigationOcTree::updateNodeTrackChangeBBX(const OcTreeKey &key, bool occupied,
                                                               bool lazy_eval, float zCoord) {

        changeBBXMin = minKey(key, changeBBXMin);
        changeBBXMax = maxKey(key, changeBBXMax);
        return updateNode(key, occupied, lazy_eval, zCoord);
    }

    /**
     * Exact copy of the base classes method, necessary because the authors of Octomap 
     * didn't make "updateNodeRecurs" a virtual method
     */
    NavigationNode *
    NavigationOcTree::updateNode(const OcTreeKey &key, bool occupied, bool lazy_eval,
                                 float zCoord) {
        float logOdds = this->prob_miss_log;
        if (occupied)
            logOdds = this->prob_hit_log;

        return updateNode(key, logOdds, lazy_eval, zCoord);
    }

    /**
     * Exact copy of the base classes method, necessary because the authors of Octomap 
     * didn't make "updateNodeRecurs" a virtual method
     */
    NavigationNode *
    NavigationOcTree::updateNode(const OcTreeKey &key, float log_odds_update, bool lazy_eval,
                                 float zCoord) {
        // early abort (no change will happen).
        // may cause an overhead in some configuration, but more often helps
        NavigationNode *leaf = this->search(key);
        // no change: node already at threshold
        if (leaf
            && ((log_odds_update >= 0 && leaf->getLogOdds() >= this->clamping_thres_max)
                || (log_odds_update <= 0 && leaf->getLogOdds() <= this->clamping_thres_min))) {
            return leaf;
        }

        bool createdRoot = false;
        if (this->root == NULL) {
            this->root = new NavigationNode();
            this->tree_size++;
            createdRoot = true;
        }

        return updateNodeRecurs(this->root, createdRoot, key, 0, log_odds_update, lazy_eval,
                                zCoord);
    }

    /**
     * Mostly a copy of the base classes method, edits for keeping the integrity 
     * of the NavigationNodes are marked by comments
     */
    NavigationNode *NavigationOcTree::updateNodeRecurs(NavigationNode *node, bool node_just_created,
                                                       const OcTreeKey &key,
                                                       unsigned int depth,
                                                       const float &log_odds_update, bool lazy_eval,
                                                       float zCoord) {
        bool created_node = false;

        assert(node);

        // follow down to last level
        if (depth < this->tree_depth) {
            unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
            if (!this->nodeChildExists(node, pos)) {
                // child does not exist, but maybe it's a pruned node?
                if (!this->nodeHasChildren(node) && !node_just_created) {
                    // current node does not have children AND it is not a new node 
                    // -> expand pruned node
                    this->expandNode(node);
                } else {
                    // not a pruned node, instantiate requested child
                    this->createNodeChild(node, pos);
                    created_node = true;
                }
            }

            if (lazy_eval)
                return updateNodeRecurs(this->getNodeChild(node, pos), created_node, key, depth + 1,
                                        log_odds_update, lazy_eval, zCoord);
            else {
                NavigationNode *retval = updateNodeRecurs(this->getNodeChild(node, pos),
                                                          created_node, key, depth + 1,
                                                          log_odds_update, lazy_eval, zCoord);
                // prune node if possible, otherwise set own probability
                // note: combining both did not lead to a speedup!
                if (this->pruneNode(node)) {
                    // return pointer to current parent (pruned), the just updated node no longer exists
                    retval = node;
                } else {
                    node->updateOccupancyChildren();
                }

                return retval;
            }
        }

            // at last level, update node, end of recursion
        else {
            if (use_change_detection) {
                bool occBefore = this->isNodeOccupied(node);
                updateNodeLogOdds(node, log_odds_update);
// change from original code:
                if (isNodeOccupied(node)) {
                    assert(node->navInfo);
                    if (node->navInfo->zCoord == 0)
                        node->navInfo->zCoord = zCoord;
                    else
                        updateZCoord(node, zCoord);
                }
// end change
                if (node_just_created) {  // new node
                    changed_keys.insert(std::pair<OcTreeKey, bool>(key, true));
                } else if (occBefore !=
                           this->isNodeOccupied(node)) {  // occupancy changed, track it
                    KeyBoolMap::iterator it = changed_keys.find(key);
                    if (it == changed_keys.end())
                        changed_keys.insert(std::pair<OcTreeKey, bool>(key, false));
                    else if (!it->second)
                        changed_keys.erase(it);
                }
            } else {
                updateNodeLogOdds(node, log_odds_update);
// change from original code:
                if (isNodeOccupied(node)) {
                    assert(node->navInfo);
                    if (node->navInfo->zCoord == 0)
                        node->navInfo->zCoord = zCoord;
                    else
                        updateZCoord(node, zCoord);
                }
// end change
            }
            return node;
        }
    }

    /**
     * update the Z-Coordinate of the node needed for the stepHeight
     * the higher the probability of the node beeing occupied the lower is the impact of the update
     */
    void NavigationOcTree::updateZCoord(NavigationNode *node, const float &update) const {
        assert(node);
        assert(node->navInfo);
        float nodeProb = (float) node->getOccupancy();
        node->navInfo->zCoord = node->navInfo->zCoord * nodeProb + update * (1.0f - nodeProb);
    }

    void NavigationOcTree::setChangedBBXToEntireTree() {
        changeBBXMin[0] = 0;
        changeBBXMin[1] = 0;
        changeBBXMin[2] = 0;
        changeBBXMax[0] = 0xFFFF;
        changeBBXMax[1] = 0xFFFF;
        changeBBXMax[2] = 0xFFFF;

        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "setChangedBBXToEntireTree!");

        hasChangedSinceLastNavigationUpdate = true;
    }

    void NavigationOcTree::setChangedBBXToEmpty() {
        changeBBXMin[0] = 0xFFFF;
        changeBBXMin[1] = 0xFFFF;
        changeBBXMin[2] = 0xFFFF;
        changeBBXMax[0] = 0;
        changeBBXMax[1] = 0;
        changeBBXMax[2] = 0;

        hasChangedSinceLastNavigationUpdate = false;
    }

    /**
     * Iterating die Boundingbox to find every Node that has enough FreeSpaceAbove 
     * for the bots collisionbox to fit hightwise
     * 
     * A Functionpointer to "handleNodeFindNodesWithFreeSpaceAbove" is being passed 
     * into the iterateBoundingBox Method
     */
    void NavigationOcTree::updateNodesWithFreeSpaceAbove(float botHeight, OcTreeKey changeBBXMin,
                                                         OcTreeKey changeBBXMax) {

        debug_navigatable_node_count = 0;
        debug_total_occupied_leafnode_count = 0;
        debug_total_leafnode_count = 0;
        debug_neighbor_NULL_count = 0;
        debug_neighbor_occupied_count = 0;

        this->iterateBoundingBox(changeBBXMin, changeBBXMax,
                                 &NavigationOcTree::handleNodeFindNodesWithFreeSpaceAbove, 0.0f,
                                 botHeight);

//        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "navigatable_node_count: %d, "
//                                    "debug_total_occupied_leafnode_count: %d"
//                                    "\ndebug_neighbor_NULL_count: %d "
//                                    "debug_neighbor_occupied_count: %d"
//                                    "\ndebug_total_leafnode_count: %d",
//                            debug_navigatable_node_count, debug_total_occupied_leafnode_count,
//                            debug_neighbor_NULL_count, debug_neighbor_occupied_count,
//                            debug_total_leafnode_count);
    }

    /**
     * Iterating die Boundingbox to find possible neighbors for every cell 
     * with enough FreeSpaceAbove and connecting them if successfull
     * 
     * A Functionpointer to "handleNodeFindNeighbors" is being passed 
     * into the iterateBoundingBox Method
     */
    void NavigationOcTree::updateAdjacencyNet(float botMaxStepHeight, OcTreeKey changeBBXMin,
                                              OcTreeKey changeBBXMax) {
        __android_log_print(ANDROID_LOG_INFO, "OCTREE",
                            "Going into iterateBoundingBox with min: %04x, %04x, %04x max: %04x, %04x, %04x",
                            changeBBXMin[0], changeBBXMin[1], changeBBXMin[2],
                            changeBBXMax[0], changeBBXMax[1], changeBBXMax[2]);
        this->iterateBoundingBox(changeBBXMin, changeBBXMax,
                                 &NavigationOcTree::handleNodeFindNeighbors, botMaxStepHeight,
                                 0.0f);
    }

    /**
     * Iterating die Boundingbox reductionIterations times while cellularly reducing 
     * the neighbor graph
     * 
     * Functionpointers to "handleNodeResetCellularFields" and to "handleNodeReduceNeighbors" are 
     * being passed into the iterateBoundingBox Method
     */
    void NavigationOcTree::reduceAdjacencyNet(int reductionIterations, OcTreeKey changeBBXMin,
                                              OcTreeKey changeBBXMax) {
        for (int i = 0; i < reductionIterations; ++i) {
            __android_log_print(ANDROID_LOG_DEBUG, "OCTREE", "reduceAdjacencyNet: iteration: %d",
                                i);
            this->iterateBoundingBox(changeBBXMin, changeBBXMax,
                                     &NavigationOcTree::handleNodeResetCellularFields, 0.0f, 0.0f);
            this->iterateBoundingBox(changeBBXMin, changeBBXMax,
                                     &NavigationOcTree::handleNodeReduceNeighbors, 0.0f, 0.0f);
        }
    }

    /**
     * small implementation of a clamp method
     */
    bool clamp(int &value, int min, int max) {
        assert(min < max);
        if (value < min) {
            value = min;
            return true;
        } else if (value > max) {
            value = max;
            return true;
        } else
            return false;
    }

    /**
     * Translates a key by the given steps in the coresponding directions
     */
    bool translateKey(OcTreeKey &key, int xTranslation, int yTranslation, int zTranslation) {
        bool hitBoundaries = false;
        int newKeyX = (int) key[0] + xTranslation;
        int newKeyY = (int) key[1] + yTranslation;
        int newKeyZ = (int) key[2] + zTranslation;

        hitBoundaries = clamp(newKeyX, 0, 0xFFFF) || hitBoundaries;
        hitBoundaries = clamp(newKeyY, 0, 0xFFFF) || hitBoundaries;
        hitBoundaries = clamp(newKeyZ, 0, 0xFFFF) || hitBoundaries;

        key[0] = (uint16_t) newKeyX;
        key[1] = (uint16_t) newKeyY;
        key[2] = (uint16_t) newKeyZ;

        return hitBoundaries;
    }

    /**
     * Method that implements the Navigability-Updating function with its 3 steps
     */
    bool NavigationOcTree::updateNavigabilityAndAdjacencyNet(float botRadius, float botHeight,
                                                             float maxStepHeight) {
        if (!hasChangedSinceLastNavigationUpdate)
            return false;

        // check if min is smaller than max on all axis
        if (changeBBXMin[0] > changeBBXMax[0] ||
            changeBBXMin[1] > changeBBXMax[1] ||
            changeBBXMin[2] > changeBBXMax[2])
            return false;

        OcTreeKey bbxMinNavigability = changeBBXMin;
        OcTreeKey bbxMaxNavigability = changeBBXMax;

        setChangedBBXToEmpty();

        int botHeightInCells = (int) ceil(botHeight / (float) getResolution());
        int reductionIterations = (int) ceil(botRadius / (float) getResolution());

        translateKey(bbxMinNavigability, 0, 0, -botHeightInCells);
        OcTreeKey bbxMinAdjacencyNet = bbxMinNavigability;
        OcTreeKey bbxMaxAdjacencyNet = bbxMaxNavigability;
        translateKey(bbxMinAdjacencyNet, -reductionIterations, -reductionIterations,
                     -reductionIterations - botHeightInCells);
        translateKey(bbxMaxAdjacencyNet, reductionIterations,
                     reductionIterations, reductionIterations);

        // Step 1: 
        __android_log_print(ANDROID_LOG_INFO, "OCTREE",
                            "updateNodesWithFreeSpaceAbove begin with height: %f", botHeight);
        updateNodesWithFreeSpaceAbove(botHeight, bbxMinAdjacencyNet, bbxMaxAdjacencyNet);
        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "updateNodesWithFreeSpaceAbove end");

        // Step 2:
        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "updateAdjacencyNet begin");
        updateAdjacencyNet(maxStepHeight, bbxMinAdjacencyNet, bbxMaxAdjacencyNet);
        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "updateAdjacencyNet end");

        // reset frontier for nodes that had the free-space above them
        this->iterateBoundingBox(bbxMinAdjacencyNet, bbxMaxAdjacencyNet,
                                 &NavigationOcTree::handleNodeResetFrontier, 0.0f, 0.0f);

        // Step 3:
        __android_log_print(ANDROID_LOG_INFO, "OCTREE",
                            "reduceAdjacencyNet begin of %d reductionIterations",
                            reductionIterations);
        reduceAdjacencyNet(reductionIterations, bbxMinAdjacencyNet, bbxMaxAdjacencyNet);
        __android_log_print(ANDROID_LOG_INFO, "OCTREE", "reduceAdjacencyNet end");

        return true;
    }

    /**
     * Iterates the OcTree recursively while combining the frontier Nodes 
     * that are spacially close to each other to clusters
     */
    void NavigationOcTree::clusterNodesRecursive(
            vector<NavigationOcTree::NodeCluster *> *clusterList, double clusterMaxRange,
            NavigationNode *node, double nodeSize, point3d nodePos) {

        if (this->nodeHasChildren(node)) {
            double halfNodeSize = nodeSize / 2.0;
            double quarterNodeSize = halfNodeSize / 2.0;
            //add children by recursion
            for (unsigned int i = 0; i < 8; ++i) {
                if (this->nodeChildExists(node, i)) {
                    clusterNodesRecursive(clusterList, clusterMaxRange, this->getNodeChild(node, i),
                                          halfNodeSize,
                                          nodePos + (DIRECTIONS[i] * quarterNodeSize));
                }
            }
        } else if (!this->isNodeOccupied(node)) {
            return;
        } else if ((*node).navInfo->isFrontierNode && node->navInfo->neighborCount > 0) {
            if (clusterList->empty())
                clusterList->push_back(new NodeCluster(nodePos, clusterMaxRange));
            else {
                vector<NodeCluster *>::iterator iterator = (*clusterList).begin();
                double leastSqDst = (*iterator)->squareDst(nodePos);
                NodeCluster *closestCluster = *iterator;
                double clusterSqDst;
                for (iterator++; iterator < (*clusterList).end(); ++iterator) {
                    if ((clusterSqDst = (*iterator)->squareDst(nodePos)) < leastSqDst) {
                        leastSqDst = clusterSqDst;
                        closestCluster = *iterator;
                    }
                }
                if (closestCluster->isInRange(nodePos))
                    closestCluster->addNode(nodePos);
                else
                    clusterList->push_back(new NodeCluster(nodePos, clusterMaxRange));
            }
        }
    }

    /**
     * Iterates the OcTree and combines the frontier Nodes that are spacially close to each other 
     * to clusters
     */
    vector<NavigationOcTree::NodeCluster *> *NavigationOcTree::clusterNavigableFrontierNodes(
            float maxRange) {
        NavigationNode *root = this->getRoot();
        if (!root)
            return NULL;

        double rootNodeSize = this->getNodeSize(0);
        point3d rootPos(0, 0, 0);

        vector<NodeCluster *> *clusterList = new vector<NodeCluster *>;
        clusterList->reserve(20);

        clusterNodesRecursive(clusterList, maxRange, root, rootNodeSize, rootPos);

        return clusterList;
    }

    /**
     * Takes a list of clusters and tries to find the shortest path to each of them.
     * It then combines the two in a PathToCluster Object and gives back a list of these
     */
    vector<NavigationOcTree::PathToCluster *> *NavigationOcTree::calculatePathsToClusters(
            const vector<NodeCluster *> *const clusterList,
            const point3d start, const float botNodeSearchRange) {
        OcTreeKey startKey;
        NavigationNode *startNode = NULL;
        NavigationNode *otherNode = NULL;

        if (!findNextNavigableNode(start, botNodeSearchRange, startKey, &startNode)) {
            __android_log_print(ANDROID_LOG_INFO, "OCTREE",
                                "Didnt find navigatable Node in Range %f", botNodeSearchRange);
            return NULL;
        } else
            __android_log_print(ANDROID_LOG_INFO, "OCTREE",
                                "Found navigatable Node in Range %f", botNodeSearchRange);

        vector<PathToCluster *> *paths = new vector<PathToCluster *>;
        AStarPath *actualPathPointer = NULL;
        AStarPath **path = &actualPathPointer;
        for (NodeCluster *cluster : *clusterList) {
            assert(cluster != NULL);
            OcTreeKey targetKey;
            if (findNextNavigableNode(cluster->getCenter(), (float) (cluster->getMaxRange()),
                                      targetKey, &otherNode)) {
                bool foundPath = AStar::findShortestPath(this, startNode, startKey,
                                                         targetKey, path);
                if (!foundPath)
                    *path = NULL;
            } else {
                *path = NULL;
            }
            paths->push_back(new PathToCluster(cluster, targetKey, *path));
        }

        return paths;
    }

    /**
     * Searches in the range around pos for a navigable Node and returns the closest one
     */
    bool NavigationOcTree::findNextNavigableNode(const point3d pos, const float range,
                                                 OcTreeKey &key, NavigationNode **node) const {
        auto it = begin_leafs_bbx(pos - point3d(range, range, range),
                                  pos + point3d(range, range, range));
        auto itEnd = end_leafs_bbx();

        NavigationNode *tempNode = NULL;
        OcTreeKey closestNodeKey;
        NavigationNode *closestNode = NULL;
        double closestNodeSqDst = numeric_limits<double>::infinity();
        for (; it != itEnd; it++) {
            tempNode = (NavigationNode *) it.operator->();
            if (isNodeOccupied(tempNode) &&
                tempNode->getNavigationInformation()->neighborCount > 0) {
                double sqDst = (pos - it.getCoordinate()).norm_sq();
                if (sqDst < closestNodeSqDst) {
                    closestNodeSqDst = sqDst;
                    closestNodeKey = it.getIndexKey();
                    closestNode = tempNode;
                }
            }
        }
        if (closestNodeSqDst != numeric_limits<double>::infinity()) {
            key = closestNodeKey;
            *node = closestNode;
            return true;
        } else {
            return false;
        }
    }

    AStarPath *NavigationOcTree::findNextPath(const point3d botPosition, const point3d botDirection,
                                              const float botNodeSearchRange,
                                              const float clusterMaxRange) {
        vector<NavigationOcTree::NodeCluster *> *clusterList = clusterNavigableFrontierNodes(
                clusterMaxRange);
        assert(clusterList != NULL);
        if (clusterList->empty()) {
            __android_log_print(ANDROID_LOG_INFO, "OCTREE", "Didnt find any nodes to cluster");
            return NULL;
        }

        vector<PathToCluster *> *pathList = calculatePathsToClusters(clusterList, botPosition,
                                                                     botNodeSearchRange);
        if (pathList == NULL) { // botPosition wasn't in range of any navigable node
            return NULL;
        }
        assert(!pathList->empty());

        // find longest path and largest cluster for normalization
        double longestPath = 0;
        long largestCluster = 0;
        for (PathToCluster *path : *pathList) {
            if (path->path != NULL) {
                longestPath = max(path->path->length, longestPath);
            }
            largestCluster = max(path->cluster->getNodeCount(), largestCluster);
        }

        // give the paths a rating based on path-length, direction, and node count of the cluster
        for (PathToCluster *path : *pathList) {
            if (path->path == NULL) {
                path->rating = numeric_limits<double>::min();
            } else {
                point3d clusterNearestNavigable = keyToCoord(path->nearestNavigableNodeKey);
                assert(clusterNearestNavigable.x() == (*path->path->wayPointList).back().x()
                       && clusterNearestNavigable.y() == (*path->path->wayPointList).back().y());
                Vector3 botToCluster = (*path->path->wayPointList)[0] - botPosition;

                double angle = GeometryHelper::angleDifference(botDirection, botToCluster);
                path->angleNormalized = angle / M_PI;
                path->clusterNodeCountNormalized =
                        (double) path->cluster->getNodeCount() / largestCluster;
                path->pathLengthNormalized = path->path->length / longestPath;

                path->rating = FACTOR_ANGLE * (1 - path->angleNormalized)
                               + FACTOR_CLUSTERSIZE * path->clusterNodeCountNormalized
                               + FACTOR_PATHLENGTH * (1 - path->pathLengthNormalized);
            }
        }

        // find the best rated path:
        PathToCluster *bestPath = (*pathList)[0];
        for (PathToCluster *path : *pathList) {
            if (bestPath->path == NULL) {
                bestPath = path;
                continue;
            }
            assert(path != NULL);
            if (path->rating > bestPath->rating) {
                bestPath = path;
            }
        }
        if (bestPath->path == NULL) {
            __android_log_print(ANDROID_LOG_DEBUG, "OCTREE", "seems like %lu paths were NULL",
                                pathList->size());
        } else {
            __android_log_print(ANDROID_LOG_DEBUG, "OCTREE",
                                "seems like bestPath != NULL, length: %f number of waypoints: %lu\n"
                                        "angleNormalized: %f, clusterNodeCountNormalized: %f, pathLengthNormalized: %f",
                                bestPath->path->length, bestPath->path->wayPointList->size(),
                                bestPath->angleNormalized, bestPath->clusterNodeCountNormalized,
                                bestPath->pathLengthNormalized);
        }

        return bestPath->path;
    }

} // end namespace