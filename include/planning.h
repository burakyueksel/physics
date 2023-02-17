#ifndef PLANNING_H_
#define PLANNING_H_

/**
 * @author  Burak Yueksel
 * @date    14 February 2023
 * @brief   Motion Planning libraries
 * @addtogroup PLANNING_H_
 **/

#include <stdio.h> // for print function
#include "vector.h"
#include "telemetry.h"

typedef struct Node {
    float x;
    float y;
    int index;
    struct Node* parent;
} Node;


/*
********************************************
** FUNCTION DECLERATIONS
********************************************
*/
float distNode(Node* n1, Node* n2);
Node* getNearestNode(Node** nodes, int numNodes, Node* q);
Node* generateRandomNode(float xMax, float yMax);
float** generateSingleObstacle(float x_m, float y_m, float radius_m);
float** generateRandomObstacles(int nrOfObstacles, float xMax, float yMax);
Node** RRT(float xStart, float yStart, float xGoal, float yGoal, float xMax, float yMax, float stepSize, int maxNodes, int numObstacles, float** obstacles);
void printRRTPath(Node** nodes, int numNodes);

#endif // PLANNING_H_