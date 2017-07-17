#include "NavigationNode.h"


namespace octomapcustomization{

    void NavigationNode::deallocChildren() {
        delete[] children;
        children = NULL;
    }
    
    void NavigationNode::allocNavigationInformation() {
        assert(navInfo == NULL);
        navInfo = new navigation_information;
    }
    
    void NavigationNode::deallocNavigationInformation() {
        assert(navInfo != NULL);
        if (navInfo->neighbors != NULL) {
            deallocNeighbors();
        }
        delete navInfo;
        navInfo = NULL;
    }

    void NavigationNode::allocNeighbors() {
        assert(navInfo);
        assert(navInfo->neighbors == NULL);
        navInfo->neighbors = new NavigationNode *[8];
        for (unsigned int i = 0; i < 8; i++) {
            navInfo->neighbors[i] = NULL;
        }
        assert(navInfo->neighborCount == 0);
    }
        
    void NavigationNode::deallocNeighbors() {
        assert(navInfo);
        assert(navInfo->neighbors);
        for (int i = 0; i < 8; ++i) {
            assert(navInfo->neighbors[i] == NULL);
        }
        delete[] navInfo->neighbors;
        navInfo->neighbors = NULL;
        assert(navInfo->neighborCount == 0);
    }

    uint8_t NavigationNode::getOppositeNeighborIdx(uint8_t i){
        return (uint8_t)((i + 4) % 8);
    }

    void NavigationNode::convertToFreeNode() {
        assert(navInfo != NULL);
        if(navInfo->neighbors != NULL){
            // detach all neighbors
            for (unsigned int i=0; i<8; i++) {
                if(navInfo->neighbors[i] != NULL) {
                    assert(navInfo->neighbors[i]->navInfo != NULL);
                    assert(navInfo->neighbors[i]->navInfo->neighbors != NULL);
                    navInfo->neighbors[i]->removeNeighbor(getOppositeNeighborIdx(i));
                    if(navInfo->neighbors[i]->navInfo->neighborCount <= 0)
                        navInfo->neighbors[i]->deallocNeighbors();
                    removeNeighbor(i);
                }
            }
        }
        assert(navInfo->neighborCount == 0);
        deallocNavigationInformation();
    }

    void NavigationNode::convertToOccupiedNode() {
        allocNavigationInformation();
    }

    void NavigationNode::setNeighbor(int i, NavigationNode *node) {
        assert(navInfo);
        assert(navInfo->neighbors);
        assert(navInfo->neighbors[i] != node);
        assert(navInfo->neighbors[i] == NULL);
        if(navInfo->neighbors[i] == NULL)
            navInfo->neighborCount++;
        navInfo->neighbors[i] = node;
    }
    
    void NavigationNode::removeNeighbor(int i) {
        assert(navInfo);
        assert(navInfo->neighbors);
        assert(navInfo->neighbors[i] != NULL);
        assert(navInfo->neighborCount > 0);
        navInfo->neighbors[i] = NULL;
        navInfo->removedNeighborCount++;
        navInfo->neighborCount--;
    }

    NavigationNode::~NavigationNode() {
        assert(navInfo == NULL);
    }

    void NavigationNode::disconnectNeighbors() {
        assert(navInfo);
        assert(navInfo->neighbors);
        for (uint8_t i = 0; i < 8; ++i) {
            if(navInfo->neighbors[i] != NULL){
                assert(this == navInfo->neighbors[i]->navInfo->neighbors[getOppositeNeighborIdx(i)]);
                navInfo->neighbors[i]->removeNeighbor(getOppositeNeighborIdx(i));
                if(navInfo->neighbors[i]->navInfo->neighborCount <= 0)
                    navInfo->neighbors[i]->deallocNeighbors();
                removeNeighbor(i);
            }
        }
        assert(navInfo->neighborCount == 0);
    }

    NavigationNode *NavigationNode::getNeighbor(int i) const {
        assert(navInfo);
        assert(navInfo->neighbors);
        assert(i >= 0 && i < 8);
        return navInfo->neighbors[i];
    }
    
    std::ostream &NavigationNode::writeData(std::ostream &s) const {
        s.write((const char*) &value, sizeof(value)); // occupancy
        char hasNavigationInfo = navInfo != NULL;
        s.write((const char*) &hasNavigationInfo, sizeof(char));

        if(hasNavigationInfo){
            s.write((const char*) &navInfo->zCoord, sizeof(float));
        }
        
        return s;
    }

    std::istream &NavigationNode::readData(std::istream &s) {
        s.read((char*) &value, sizeof(value)); // occupancy
        char hasNavigationInfo;
        s.read((char*) &hasNavigationInfo, sizeof(char));

        if(hasNavigationInfo){
            allocNavigationInformation();
            s.read((char*) &navInfo->zCoord, sizeof(float));
        }
        
        return s;
    }
}