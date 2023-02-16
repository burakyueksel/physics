/**
 * @author  Burak Yueksel
 * @date    14 February 2023
 * @brief   Motion Planning Libraries
 * @addtogroup PLANNING
 **/

#include "planning.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

/** @brief Ccreate a new node
 */
Node* createNewNode(void)
{
    // create a node with the correct structure
    Node * node = (Node*) malloc(sizeof(Node));
    return node;
}

/** @brief Compute the Eucledian distance in 2D between two nodes
 */
float distNode(Node* n1, Node* n2)
{
    return sqrtf(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));
}
/** @brief Return the nearest node to the node q in 2D
 */
Node* getNearestNode(Node** nodes, int numNodes, Node* q)
{
    Node* nearest = NULL;
    float minDist = INFINITY;
    for (int i = 0; i < numNodes; i++) {
        float d = distNode(nodes[i], q);
        if (d < minDist) {
            nearest = nodes[i];
            minDist = d;
        }
    }
    return nearest;
}

/** @brief Compute a new node that steers to the nearest node in 2D
 */
void steer(Node* q, Node* nearest, float stepSize, Node* qNew)
{
    float theta = atan2(q->y - nearest->y, q->x - nearest->x);
    qNew->x = nearest->x + stepSize * cos(theta);
    qNew->y = nearest->y + stepSize * sin(theta);
    qNew->parent = nearest;
}

/** @brief Check if the next node is colliding wiht any obstacle in 2D.
 *  Returns 0 (hit obstacle) or 1 (obstacle free)
 */
int isObstacleFree(Node* q, Node* nearest, float stepSize, int numObstacles, float** obstacles)
{
    float dx = q->x - nearest->x;
    float dy = q->y - nearest->y;
    float dist = sqrtf(dx*dx + dy*dy);
    float numSteps = ceil(dist / stepSize);
    float x, y;
    for (int i = 1; i < numSteps; i++) {
        x = nearest->x + i * dx / numSteps;
        y = nearest->y + i * dy / numSteps;
        for (int j = 0; j < numObstacles; j++) {
            if (sqrtf(pow(x - obstacles[j][0], 2) + pow(y - obstacles[j][1], 2)) <= obstacles[j][2]) {
                return 0;
            }
        }
    }
    return 1;
}

/** @brief Generate a random node within the bounds of the 2D map
 */
Node* generateRandomNode(float xMax, float yMax)
{
    Node* q = createNewNode();
    q->x = ((float) rand() / RAND_MAX) * xMax;
    q->y = ((float) rand() / RAND_MAX) * yMax;
    q->parent = NULL;
    return q;
}

/** @brief RRT algorithm that runs in 2D
 */
Node** RRT(float xStart, float yStart, float xGoal, float yGoal, float xMax, float yMax, float stepSize, int maxNodes, int numObstacles, float** obstacles)
{
    // Create the start node.
    Node* startNode = createNewNode();
    startNode->x = xStart;
    startNode->y = yStart;
    startNode->parent = NULL;
    // Create the goal node
    Node* goalNode = createNewNode();
    goalNode->x = xGoal;
    goalNode->y = yGoal;
    goalNode->parent = NULL;
    
    // Allocate memory for nodes
    Node** nodes = (Node**) malloc(maxNodes * sizeof(Node*));
    // Start node is the first node
    nodes[0] = startNode;
    // Init the node number/ID
    int numNodes = 1;
    
    // Generate nodes until goal node is reached or max nodes is exceeded
    while (numNodes < maxNodes) {
        // Generate random point in space
        Node* q = createNewNode();
        q->x = (float)rand() / (float)RAND_MAX * xMax;
        q->y = (float)rand() / (float)RAND_MAX * yMax;
        q->parent = NULL;
        
        // Find nearest node
        Node* nearest = getNearestNode(nodes, numNodes, q);
        
        // Check if the nearest node is obstacle free
        // If it is, then steer to there
        if (isObstacleFree(q, nearest, stepSize, numObstacles, obstacles)) {
            // Add new node
            Node* newNode = createNewNode();
            // Steer to the nearest node, where the result is the new node
            steer(q, nearest, stepSize, newNode);
            // add the new node to the nodes
            nodes[numNodes] = newNode;
            // iterate the number of nodes
            numNodes++;
            // Check if goal node is reached (latest new node is very close to the goal node)
            if (distNode(newNode, goalNode) <= stepSize) {
                newNode->parent = goalNode;
                return nodes;
            }
        }
    }
    return NULL;
}

/** @brief Print function for the RRT path sarch result
 */
void printRRTResult(Node** nodes, float xGoal, float yGoal, int maxNodes, float stepSize)
{
    // Print path if one is found
    if (nodes != NULL) {
        Node* goalNode = (Node*) malloc(sizeof(Node));
        goalNode->x = xGoal;
        goalNode->y = yGoal;
        goalNode->parent = NULL;
        Node* nearestGoal = getNearestNode(nodes, maxNodes, goalNode);
        if (nearestGoal != NULL && distNode(nearestGoal, goalNode) <= stepSize) {
            printf("Path found:\n");
            //printPath(nearestGoal);
        } else {
            printf("No path found\n");
        }
    } else {
        printf("No path found\n");
    }
}