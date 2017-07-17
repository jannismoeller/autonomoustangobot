#ifndef AUTONOMOUSTANGOBOT_NAVIGATIONNODE_H
#define AUTONOMOUSTANGOBOT_NAVIGATIONNODE_H

#include <octomap/OcTreeNode.h>

using namespace octomap;

namespace octomapcustomization {

    class NavigationNode;
    
    struct navigation_information {
        // precise averaged out zCoord of the node
        float zCoord = 0;
        bool hasFreeSpaceAbove = false;
        // node lays on the border to unknown space
        bool isFrontierNode = false;
        // Pointer to array with horizontal neighbors, might be NULL
        NavigationNode ** neighbors = NULL;
        int8_t neighborCount = 0;
        // The following two are just used in the reduction steps 
        // to simulate a cellular automate
        int8_t removedNeighborCount = 0;
        bool wasFrontierNode = false;
    };
    
    class NavigationNode : public OcTreeNode {
    public:
        friend class NavigationOcTree;

        virtual ~NavigationNode();

        static uint8_t getOppositeNeighborIdx(uint8_t i);
        
        void setNeighbor(int i, NavigationNode *node);
        void removeNeighbor(int i);
        NavigationNode *getNeighbor(int i) const;
        inline const navigation_information *getNavigationInformation() const { return navInfo; }
        // file I/O
        std::istream& readData(std::istream &s);
        std::ostream& writeData(std::ostream &s) const;

    private:
        // Is only allocated and set when the node is occupied
        navigation_information *navInfo = NULL;
        void deallocChildren();
        void allocNavigationInformation();
        void deallocNavigationInformation();
        void allocNeighbors();
        void deallocNeighbors();
        void convertToFreeNode();
        void convertToOccupiedNode();

        void disconnectNeighbors();
    };
}

#endif //AUTONOMOUSTANGOBOT_NAVIGATIONNODE_H
