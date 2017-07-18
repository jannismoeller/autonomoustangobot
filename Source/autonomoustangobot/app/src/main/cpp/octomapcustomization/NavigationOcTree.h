//
// Created by Jannis on 23.06.2017.
//

#ifndef AUTONOMOUSTANGOBOT_NAVIGATIONOCTREE_H
#define AUTONOMOUSTANGOBOT_NAVIGATIONOCTREE_H

#include <string>
#include <octomap/OccupancyOcTreeBase.h>
#include <jni.h>
#include "NavigationNode.h"
#include "AStarDataStructures.h"

using namespace octomap;
using namespace std;

namespace octomapcustomization{
    class NavigationOcTree : public OccupancyOcTreeBase <NavigationNode>{
    
    public:
        /**
         * Class that represents a cluster of nodes
         */
        class NodeCluster{
        private:
            point3d center;
            long nodeCount = 1;
            const double maxRange;
            const double maxRangeSquared;

        public:
            const point3d getCenter() const { return center; };
            const long getNodeCount() const { return nodeCount; };
            const double getMaxRange() const { return maxRange; }

            NodeCluster(const point3d &center, const double maxRange) : center(center), maxRange(maxRange), maxRangeSquared(maxRange * maxRange) {};
            
            inline
            double squareDst(point3d nodePos){
                point3d difference = nodePos - center;
                return difference.norm_sq();
            };
            
            bool isInRange(point3d nodePos){
                return squareDst(nodePos) <= maxRangeSquared;
            };
            
            void addNode(point3d nodePos){
                nodeCount++;
                center = center * ((double) (nodeCount-1) / (double) nodeCount) 
                         + nodePos * (1.0f / (double) nodeCount);
            }
        };

        /**
         * Class that associates a cluster of nodes with a path to it
         */
        class PathToCluster{
        public:
            NodeCluster*const cluster;
            OcTreeKey const nearestNavigableNodeKey;
            AStarPath*const path;
            double rating = 0;
            double angleNormalized = 1;
            double clusterNodeCountNormalized = 1;
            double pathLengthNormalized = 1;
            
            PathToCluster(NodeCluster*const cluster, 
                          OcTreeKey const nearestNavigableNodeKey, 
                          AStarPath*const path) : 
                    cluster(cluster),
                    nearestNavigableNodeKey(nearestNavigableNodeKey),
                    path(path) {};
        };
        
        /// Default constructor, sets resolution of leafs
        NavigationOcTree(double resolution);

        /**
         * Reads an NavigationOctree from a binary file 
         * @param _filename
         */
        NavigationOcTree(string _filename);

        virtual ~NavigationOcTree();

        /// virtual constructor: creates a new object of same type
        NavigationOcTree* create() const {return new NavigationOcTree(resolution); }

        // For compatibility with OctoVis the Type name must be "OcTree"
        // note that you can't load a file that was saved as the other after changing this
        string getTreeType() const {return "NavigationOcTree";}
//        string getTreeType() const {return "OcTree";}

        virtual void insertPointCloud(const Pointcloud& scan, const octomap::point3d& sensor_origin,
                                      double maxrange=-1., bool lazy_eval = false, bool discretize = false);
        void insertTangoPointCloud(jfloat *pointsFloatBuffer, int numPoints, float transformationMatrix[], point3d sensorOrigin, double maxrange, double minrange);

        bool testCollisionWithCylinder(point3d pos, float radius, float height, unsigned char maxDepth);
        void fillBox(point3d from, point3d to, bool occupied);
        void fillBoxAsMeasurements(point3d from, point3d to, bool occupied);
        int fillBufferWithNodePoints(jfloat *pFB, int bufferMaxVertsCount);
        int fillBufferWithNodePointsViaLeafIterator(jfloat *pFB, int bufferMaxVertsCount);
        int raySteppingUp(OcTreeKey currentNodeKey, double maxZCoord);
        bool updateNavigabilityAndAdjacencyNet(float botRadius, float botHeight,
                                                 float maxStepHeight);
        void setChangedBBXToEntireTree();
        void setChangedBBXToEmpty();
        AStarPath *findNextPath(const point3d botPosition,const point3d botDirection, 
                                const float botNodeSearchRange, const float clusterMaxRange);

    protected:
        /**
         * Static member object which ensures that this OcTree's prototype
         * ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot
         * files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer{
        public:
            StaticMemberInitializer() {
                NavigationOcTree* tree = new NavigationOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this octree.
             */
            void ensureLinking() {};
        };

        /// to ensure static initialization (only once)
        static StaticMemberInitializer ocTreeMemberInit;

    private:
        static const point3d DIRECTIONS[];
        static const uint16_t KEYS_0[];
        static const uint16_t KEYS_1[];
        static const uint16_t KEYS_2[];

        static const int KEY_NEIGHBORS_0[];
        static const int KEY_NEIGHBORS_1[];

        static const int treeDepth = 16;
        static const int stackSize = treeDepth + 1;

        static constexpr double FACTOR_ANGLE = 2;
        static constexpr double FACTOR_CLUSTERSIZE = 1;
        static constexpr double FACTOR_PATHLENGTH = 2;

        int debug_navigatable_node_count = 0;
        int debug_total_occupied_leafnode_count = 0;
        int debug_total_leafnode_count = 0;
        int debug_neighbor_NULL_count = 0;
        int debug_neighbor_occupied_count = 0;

        int debug_searchFromCurrent_count = 0;

        OcTreeKey changeBBXMin, changeBBXMax;
        bool hasChangedSinceLastNavigationUpdate = false;

        void clear();

        void transFormAndFilterPoints(const jfloat *pFB, int numPoints, const float *m4x4,
                                      Pointcloud &pointCloud,
                                      const double filterMinDistance) const;

        bool isInRange(point3d botPosition, float botRadius, point3d cellCoordinate, float cellSize);

        inline point3d minCoordinatesPoint(point3d p1, point3d p2);

        inline point3d maxCoordinatesPoint(point3d p1, point3d p2);

        void addNodeToBufferRecursive(jfloat **pFB, int *vertexCount, int bufferMaxVerts,
                                      NavigationNode *node, double nodeSize, point3d nodePos);

        void searchFromCurrent(array<NavigationNode *, stackSize> *stackToCurrentNode,
                               OcTreeKey *currentNodeKey,
                               int *currentDepth, OcTreeKey searchedNodeKey);

        void getUpperNeighbor(array<NavigationNode *, stackSize> *stackToCurrentNode,
                              OcTreeKey *currentNodeKey,
                              int *currentDepth);
        int raySteppingUp(array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey *currentNodeKey,
                          int *currentDepth, double maxZCoord);

        void handleNodeFindNodesWithFreeSpaceAbove(
                array<NavigationNode *, stackSize> *stackToCurrentNode,
                OcTreeKey currentNodeKey, int currentDepth, float,
                float botHeight);

        void handleNodeFindNeighbors(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                             OcTreeKey currentNodeKey, int currentDepth,
                                             float stepHeight, float);

        void handleNodeReduceNeighbors(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                               OcTreeKey currentNodeKey, int currentDepth, float,
                                               float);

        void handleNodeResetCellularFields(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                           OcTreeKey, int currentDepth, float, float);
        
        void handleNodeResetFrontier(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                                 OcTreeKey, int currentDepth, float, float);

        inline bool keyAllComponentsSmallerOrEqual(OcTreeKey key1, OcTreeKey key2);
        inline bool keyAllComponentsBiggerOrEqual(OcTreeKey key1, OcTreeKey key2);

        void iterateLeafsRecursive(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                   OcTreeKey currentNodeKey, int currentDepth,
                                   int currentShiftsFromBottom,
                                   void (NavigationOcTree::*handleNode)(array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey,
                                                      int currentDepth, float stepHeight, float botHeight),
                                   float stepHeight, float botHeight);

        void iterateBoundingBoxRecursive(array<NavigationNode *, stackSize> *stackToCurrentNode,
                                         int currentDepth, int currentShiftsFromBottom,
                                         OcTreeKey minKeyBBX, OcTreeKey maxKeyBBX,
                                         OcTreeKey minKeyNode, OcTreeKey maxKeyNode,
                                         void (NavigationOcTree::*handleNode)(array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey,
                                                            int currentDepth, float stepHeight, float botHeight),
                                         float stepHeight, float botHeight);

        void iterateBoundingBox(OcTreeKey minKeyBBX, OcTreeKey maxKeyBBX,
                                void (NavigationOcTree::*handleNode)(array<NavigationNode *, stackSize> *stackToCurrentNode, OcTreeKey,
                                                   int currentDepth, float stepHeight, float botHeight),
                                float stepHeight, float botHeight);

        void updateNodesWithFreeSpaceAbove(float botHeight, OcTreeKey changeBBXMin,
                                           OcTreeKey changeBBXMax);
        void updateAdjacencyNet(float botMaxStepHeight, OcTreeKey changeBBXMin, OcTreeKey changeBBXMax);
        void reduceAdjacencyNet(int reductionIterations, OcTreeKey changeBBXMin,
                                        OcTreeKey changeBBXMax);

        void deleteNavigationNodeRecurs(NavigationNode *node);

        NavigationNode *setNodeValue(const OcTreeKey &key, float log_odds_value, bool lazy_eval, float zCoord);
        NavigationNode *setNodeValueRecurs(NavigationNode *node, bool node_just_created, const OcTreeKey &key,
                       unsigned int depth, const float &log_odds_value, bool lazy_eval, float zCoord);

        NavigationNode *updateNodeTrackChangeBBX(const OcTreeKey &key, bool occupied, bool lazy_eval, float zCoord);
        NavigationNode *updateNode(const OcTreeKey &key, bool occupied, bool lazy_eval, float zCoord);
        NavigationNode *updateNode(const OcTreeKey &key, float log_odds_update, bool lazy_eval, float zCoord);
        NavigationNode *updateNodeRecurs(NavigationNode *node, bool node_just_created, const OcTreeKey &key,
                                     unsigned int depth, const float &log_odds_update, bool lazy_eval, float zCoord);

        virtual bool isNodeCollapsible(const NavigationNode* node) const;
        virtual void updateNodeLogOdds(NavigationNode* occupancyNode, const float& update) const;
        void updateZCoord(NavigationNode* node, const float& update) const;

        OcTreeKey minKey(OcTreeKey a, OcTreeKey b);
        OcTreeKey maxKey(OcTreeKey a, OcTreeKey b);

        void clusterNodesRecursive(vector<NodeCluster *> *clusterList, double clusterMaxRange,
                               NavigationNode *node, double nodeSize, point3d nodePos);

        bool findNextNavigableNode(const point3d pos, const float, OcTreeKey &key,
                                   NavigationNode **node) const;
        
        vector<NodeCluster*> *clusterNavigableFrontierNodes(float maxRange);
        vector<PathToCluster *> *calculatePathsToClusters(
                const vector<NodeCluster *> *const clusterList,
                const point3d start, const float botNodeSearchRange);
    };
} // end namespace


#endif //AUTONOMOUSTANGOBOT_NAVIGATIONOCTREE_H
