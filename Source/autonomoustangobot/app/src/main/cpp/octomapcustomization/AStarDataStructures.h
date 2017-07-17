//
// Created by Jannis on 01.07.2017.
//

#ifndef AUTONOMOUSTANGOBOT_PRIORITYQUEUE_H
#define AUTONOMOUSTANGOBOT_PRIORITYQUEUE_H

#include <android/log.h>
#include <unordered_map>
#include <vector>
#include <octomap/OcTreeKey.h>
#include "NavigationNode.h"

using namespace std;
using namespace octomap;

namespace octomapcustomization {
    class AStarNode {
    private:
        // cost to this node
        float gCost = 0;
        // heuristic cost from this node to target
        const float hCost = 0;
        // sum of gCost and hCost, gets cached for faster access
        float fCost = 0;

        // handle to position of the heap-index of this node in the hash-map
        unordered_map<OcTreeKey, unsigned long, OcTreeKey::KeyHash>::iterator unordered_mapHeapPositionsHandle;

    public:
        inline float getGCost() const { return gCost; }
        inline void setGCost(float val) {
            gCost = val;
            fCost = gCost + hCost;
        }

        inline float getHCost() const { return hCost; }

        inline float getFCost() const { return fCost; }

        AStarNode *cameFromNode;
        OcTreeKey const key;
        NavigationNode *const navigationNode;

        unsigned long getHeapPos() const { return unordered_mapHeapPositionsHandle.operator->()->second; }
        void setHeapPos(unsigned long pos){ unordered_mapHeapPositionsHandle.operator->()->second = pos; }

        unordered_map<OcTreeKey, unsigned long, OcTreeKey::KeyHash>::iterator getHeapPosHandle(){ return unordered_mapHeapPositionsHandle; }
        void setHeapPosHandle(unordered_map<OcTreeKey, unsigned long, OcTreeKey::KeyHash>::iterator handle){ unordered_mapHeapPositionsHandle = handle; }

        AStarNode(AStarNode *cameFromNode,
                  const OcTreeKey key, NavigationNode *const navigationNode,
                  const float gCost, const float hCost)
                : cameFromNode(cameFromNode),
                  key(key), navigationNode(navigationNode),
                  gCost(gCost), hCost(hCost)
        {
            fCost = gCost + hCost;
        }

        int compareTo(AStarNode*const node) const {
            assert(node != NULL);
            float compare = fCost - node->fCost;
            if(compare == 0.0f) {
                compare = hCost - node->hCost;
            }
            if(compare < 0.0f)
                return 1;
            else
                return -1;
        }
    };

    class AStarPath {
    public:
        const vector<point3d> *wayPointList;
        const double length;
        AStarPath(const vector<point3d> *wayPointList, const double length)
                : wayPointList(wayPointList), length(length) {}

        virtual ~AStarPath() {
            delete wayPointList;
        }
    };
    /**
     * PriorityQueue implementation using a binary heap
     * in combination with a unordered map for fast finding of elements
     */
    class AStarOpenSet {
    private:
        vector<AStarNode*> binHeapArray;
        unordered_map<OcTreeKey, unsigned long, OcTreeKey::KeyHash> heapPositions;

        void sortDown(AStarNode* node) {
            
            while (true) {
                assert(node->getHeapPos() <= binHeapArray.size());
                
                unsigned long childIndexLeft = node->getHeapPos() * 2 + 1;
                unsigned long childIndexRight = node->getHeapPos() * 2 + 2;
                unsigned long swapIndex = 0;

                
                if (childIndexLeft < binHeapArray.size()) {
                    swapIndex = childIndexLeft;
                    if (childIndexRight < binHeapArray.size()) {
                        if (binHeapArray[childIndexLeft]->compareTo(binHeapArray[childIndexRight]) < 0) {
                            swapIndex = childIndexRight; // highest Priority
                        }
                    }
                    if (node->compareTo(binHeapArray[swapIndex]) < 0) {
                        swap(node, binHeapArray[swapIndex]);
                    }
                    else {
                        return;
                    }
                }
                else {
                    return;
                }
            }
        }

        void sortUp(AStarNode* node) {
            
            unsigned long parentIndex;
            AStarNode* parentNode;
            
            while (node->getHeapPos() > 0) {
                parentIndex = (node->getHeapPos() - 1) / 2;
                parentNode = binHeapArray[parentIndex];
                
                if (node->compareTo(parentNode) > 0) {
                    swap(node, parentNode);
                }
                else {
                    break;
                }
            }
        }

        void swap(AStarNode* a, AStarNode* b) {
            binHeapArray[a->getHeapPos()] = b;
            binHeapArray[b->getHeapPos()] = a;
            unsigned long aHeapPos = a->getHeapPos();
            a->setHeapPos(b->getHeapPos());
            b->setHeapPos(aHeapPos);
        }

    public:
        AStarOpenSet(unsigned long initialHeapSize) {
            binHeapArray.reserve(initialHeapSize);
        }

        virtual ~AStarOpenSet() {
            for (int i = 0; i < binHeapArray.size(); ++i) {
                delete binHeapArray[i];
            }
        }

        void add(AStarNode*const node) {
            assert(node);

            auto res = heapPositions.insert({node->key, binHeapArray.size()});
            assert(res.second);
            node->setHeapPosHandle(res.first);
            
            node->setHeapPos(binHeapArray.size());
            binHeapArray.push_back(node);
            
            sortUp(node);

            for (unsigned long i = 0; i < binHeapArray.size(); ++i) {
                assert(binHeapArray[i] != NULL);
            }
        }

        AStarNode* pop() {
            assert(binHeapArray.size() > 0);

            AStarNode* firstNode = binHeapArray[0];
            heapPositions.erase(firstNode->getHeapPosHandle());
            binHeapArray[0] = binHeapArray[binHeapArray.size() - 1];
            binHeapArray[0]->setHeapPos(0);
            binHeapArray.pop_back();
            sortDown(binHeapArray[0]);
            return firstNode;
        }

        AStarNode* get(const OcTreeKey ocTreeKey) const {
            auto it = heapPositions.find(ocTreeKey);
            if(it == heapPositions.end())
                return NULL;
            else
                return binHeapArray[it.operator->()->second];
        }

        void updateNode(AStarNode*const node) {
            sortUp(node);
        }

        bool isEmpty() {
            return binHeapArray.empty();
        }
    };

    class AStarClosedSet {
    private:
        unordered_map<OcTreeKey, AStarNode *, OcTreeKey::KeyHash> map;
    public:
        virtual ~AStarClosedSet() {
            for (auto it = map.begin(); it != map.end(); ++it) {
                delete it.operator->()->second;
            }
        }

        void add(AStarNode *node) {
            auto res = map.insert({node->key, node});
            assert(res.second);
        }

        bool contains(OcTreeKey key){
            auto it = map.find(key);
            return it != map.end();
        }
        
        unsigned long size(){
            return map.size();
        }
    };
}

#endif //AUTONOMOUSTANGOBOT_PRIORITYQUEUE_H
