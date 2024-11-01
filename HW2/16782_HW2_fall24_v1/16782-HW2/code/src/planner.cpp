/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <unordered_map>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(
    double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

// fix can't find location?
bool configurationsEqual(double* config1, double* config2, int numofDOFs, double tolerance = 1e-3) 
{
    for (int i = 0; i < numofDOFs; ++i) {
        if (fabs(config1[i] - config2[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

struct Node 
{
    double* config; 
    Node* parent;   

    Node(double* config, int numofDOFs) 
	{
        this->config = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; ++i) 
		{
            this->config[i] = config[i];
        }
        this->parent = nullptr;
    }

    ~Node() 
    {
        delete[] config;
    }
};

double calculateDistance(double* config1, double* config2, int numofDOFs) 
{
    double distance = 0.0;
    for (int i = 0; i < numofDOFs; ++i) 
	{
        distance += (config1[i] - config2[i]) * (config1[i] - config2[i]);
    }
    return sqrt(distance);
}

void interpolate(double* from, double* to, double* result, double t, int numofDOFs) 
{
    for (int i = 0; i < numofDOFs; ++i) 
	{
        result[i] = from[i] + t * (to[i] - from[i]);
    }
}

Node* expandTree(Node* nearestNode, double* randomConfig, double stepSize, int numofDOFs) 
{
    double* newConfig = new double[numofDOFs];
    double distance = calculateDistance(nearestNode->config, randomConfig, numofDOFs);
    double t = (stepSize / distance);

    if (t > 1.0) t = 1.0;
    interpolate(nearestNode->config, randomConfig, newConfig, t, numofDOFs);

    Node* newNode = new Node(newConfig, numofDOFs);
    newNode->parent = nearestNode;
    delete[] newConfig; 
    return newNode;
}

Node* findNearestNode(const std::vector<Node*>& tree, double* randomConfig, int numofDOFs) 
{
    Node* nearestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (Node* node : tree) 
	{
        double dist = calculateDistance(node->config, randomConfig, numofDOFs);
        if (dist < minDistance) {
            minDistance = dist;
            nearestNode = node;
        }
    }
    return nearestNode;
}

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
// {
//     /* TODO: Replace with your implementation */
//     planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
// }
{

    int maxIterations = 50000;   
    double goalBias = 0.05;      
    double stepSize = PI / 10.0; 
    Node* sNode = new Node(armstart_anglesV_rad, numofDOFs);
    Node* gNode = new Node(armgoal_anglesV_rad, numofDOFs);

    std::srand(std::time(0));

    std::vector<Node*> tree;
    //T.init(qinit)
    Node* startNode = new Node(armstart_anglesV_rad, numofDOFs);
    tree.push_back(startNode);

    //for k=1 to K do
    for (int iter = 0; iter < maxIterations; ++iter) 
	{
        // Randomly sample
        //qrand <- RAND_CONF()
        double* randomConfig = new double[numofDOFs];
        if ((double)rand() / RAND_MAX < goalBias) 
		{
            // Sample the goal
            for (int i = 0; i < numofDOFs; ++i)
			{
                randomConfig[i] = armgoal_anglesV_rad[i];
                std::cout << "goal" << std::endl;
                
            }
        } else 
		{
            // Otherwise, randomly
            for (int i = 0; i < numofDOFs; ++i) 
            {
                randomConfig[i] = (double)rand() / RAND_MAX * 2 * PI;
                std::cout << "Random Search!" << std::endl;
            }
        }

        // qnear <- NEAREST_VERTEX(q, T)
        Node* nearestNode = findNearestNode(tree, randomConfig, numofDOFs);
        Node* newNode = expandTree(nearestNode, randomConfig, stepSize, numofDOFs);

        // if NEW_CONFIG(q,qnear,qnew) then
        if (IsValidArmConfiguration(newNode->config, numofDOFs, map, x_size, y_size)) 
		{
            tree.push_back(newNode);

            if (calculateDistance(newNode->config, armgoal_anglesV_rad, numofDOFs) < stepSize) 
			{
                std::cout << "Goal reached!" << std::endl;
                

                // Trace path
                std::vector<Node*> path;
                Node* current = newNode;
                while (current != nullptr) 
				{
                    path.push_back(current);
                    current = current->parent;
                    std::cout << "Trace Path" << std::endl;
                }

                // Ensure the start and goal configurations are explicitly included
                    if (!configurationsEqual(path.back()->config, armstart_anglesV_rad, numofDOFs)) 
					{
                        path.push_back(sNode); // Ensure the start configuration is included
                    }
                    if (!configurationsEqual(path.front()->config, armgoal_anglesV_rad, numofDOFs)) 
					{
                        path.insert(path.begin(), gNode); 
                    }
                
                // Store plan
                *planlength = path.size();
                *plan = (double**)malloc(*planlength * sizeof(double*));
                for (int i = *planlength - 1; i >= 0; --i) 
				{
                    (*plan)[*planlength - i - 1] = (double*)malloc(numofDOFs * sizeof(double));
                    for (int j = 0; j < numofDOFs; ++j) 
					{
                        (*plan)[*planlength - i - 1][j] = path[i]->config[j];
                        std::cout << "Store path" << std::endl;
                    }
                }

                // Free memory used by the tree
                for (Node* node : tree) 
				{
                    delete node;
                }
                delete[] randomConfig;
                return;
            }
        }
        delete[] randomConfig;
    }

    std::cout << "RRT failed to find a path within the given iterations." << std::endl;

    // Free memory used by the tree
    for (Node* node : tree) 
	{
        delete node;
    }
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//



static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
// {
//     /* TODO: Replace with your implementation */
//     planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
// }

{
    int maxIterations = 10000;   
    double stepSize = PI / 15.0; 

    std::srand(std::time(0));

    std::vector<Node*> startTree;
    std::vector<Node*> goalTree;
    Node* startNode = new Node(armstart_anglesV_rad, numofDOFs);
    Node* goalNode = new Node(armgoal_anglesV_rad, numofDOFs);
    startTree.push_back(startNode);
    goalTree.push_back(goalNode);

    for (int iter = 0; iter < maxIterations; ++iter) 
	{
        double* randomConfig = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; ++i) 
		{
            randomConfig[i] = (double)rand() / RAND_MAX * 2 * PI;
        }

        // Extend start nodes
        Node* nearestStartNode = findNearestNode(startTree, randomConfig, numofDOFs);
        Node* newStartNode = expandTree(nearestStartNode, randomConfig, stepSize, numofDOFs);

        if (IsValidArmConfiguration(newStartNode->config, numofDOFs, map, x_size, y_size)) 
		{
            startTree.push_back(newStartNode);

            // Connect start tree 
            Node* nearestGoalNode = findNearestNode(goalTree, newStartNode->config, numofDOFs);
            Node* newGoalNode = expandTree(nearestGoalNode, newStartNode->config, stepSize, numofDOFs);

            if (IsValidArmConfiguration(newGoalNode->config, numofDOFs, map, x_size, y_size)) 
			{
                goalTree.push_back(newGoalNode);

                // Check if the trees have connected
                if (calculateDistance(newGoalNode->config, newStartNode->config, numofDOFs) < stepSize) 
				{
                    std::cout << "Trees have connected!" << std::endl;

                    // Trace back the path from start to goal
                    std::vector<Node*> path;

                    // Trace the path from the start tree
                    Node* current = newStartNode;
                    while (current != nullptr) 
					{
                        path.push_back(current);
                        current = current->parent;
                    }

                    // Add the path from the goal tree (skipping the connecting node to avoid duplication)
                    current = newGoalNode->parent;
                    while (current != nullptr) 
					{
                        path.push_back(current);
                        current = current->parent;
                    }

                    // Ensure the start and goal configurations are explicitly included
                    if (!configurationsEqual(path.back()->config, armstart_anglesV_rad, numofDOFs)) 
					{
                        path.push_back(startNode); // Ensure the start configuration is included
                    }
                    if (!configurationsEqual(path.front()->config, armgoal_anglesV_rad, numofDOFs)) 
					{
                        path.insert(path.begin(), goalNode); 
                    }

                    // Store plan
                    *planlength = path.size();
                    *plan = (double**)malloc(*planlength * sizeof(double*));
                    for (int i = *planlength - 1; i >= 0; --i) 
					{
                        (*plan)[*planlength - i - 1] = (double*)malloc(numofDOFs * sizeof(double));
                        for (int j = 0; j < numofDOFs; ++j) 
						{
                            (*plan)[*planlength - i - 1][j] = path[i]->config[j];
                        }
                    }

                    // Free memory 
                    for (Node* node : startTree) delete node;
                    for (Node* node : goalTree) delete node;
                    delete[] randomConfig;
                    return;
                }
            }
        }
        delete[] randomConfig;
    }

    std::cout << "RRT-Connect failed to find a path within the given iterations." << std::endl;

    // Free memory used by the trees
    for (Node* node : startTree) delete node;
    for (Node* node : goalTree) delete node;
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//


// Node structure for the RRT* algorithm
struct StarNode 
{
    double* config;  // Configuration of the robot arm (joint angles)
    StarNode* parent;    // Parent node in the tree
    double pathCost;     // Cost from the start to this node

    StarNode(double* config, int numofDOFs) 
	{
        this->config = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; ++i) {
            this->config[i] = config[i];
        }
        this->parent = nullptr;
        this->pathCost = std::numeric_limits<double>::max(); // Initially set cost to a large value
    }

    ~StarNode() 
	{
        delete[] config;
    }
};

double calculateRRTStarDistance(double* config1, double* config2, int numofDOFs) 
{
    double distance = 0.0;
    for (int i = 0; i < numofDOFs; ++i) 
	{
        distance += (config1[i] - config2[i]) * (config1[i] - config2[i]);
    }
    return sqrt(distance);
}

// Helper function to interpolate between two configurations
void interpolateRRTStar(double* from, double* to, double* result, double t, int numofDOFs) 
{
    for (int i = 0; i < numofDOFs; ++i) {
        result[i] = from[i] + t * (to[i] - from[i]);
    }
}

// Expands the tree towards the random configuration or goal
StarNode* extendRRTStarTree(StarNode* closestNode, double* sampledConfig, double expansionStep, int numofDOFs) 
{
    double* newConfig = new double[numofDOFs];
    double distance = calculateRRTStarDistance(closestNode->config, sampledConfig, numofDOFs);
    double t = (expansionStep / distance);

    if (t > 1.0) t = 1.0; // Limit the interpolation
    interpolateRRTStar(closestNode->config, sampledConfig, newConfig, t, numofDOFs);

    StarNode* expandedNode = new StarNode(newConfig, numofDOFs);
    expandedNode->parent = closestNode;
    expandedNode->pathCost = closestNode->pathCost + calculateRRTStarDistance(closestNode->config, newConfig, numofDOFs);

    delete[] newConfig;  // Clean up temporary allocation
    return expandedNode;
}

// Find the nearest node in the tree to a given configuration
StarNode* findClosestRRTStarNode(const std::vector<StarNode*>& rrtStarTree, double* sampledConfig, int numofDOFs) 
{
    StarNode* closestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (StarNode* node : rrtStarTree) 
	{
        double dist = calculateRRTStarDistance(node->config, sampledConfig, numofDOFs);
        if (dist < minDistance) 
		{
            minDistance = dist;
            closestNode = node;
        }
    }
    return closestNode;
}

// Find all nodes in a given radius
std::vector<StarNode*> getRRTStarNeighborNodes(const std::vector<StarNode*>& rrtStarTree, StarNode* expandedNode, double rewiringRadius, int numofDOFs) 
{
    std::vector<StarNode*> neighboringNodes;
    for (StarNode* node : rrtStarTree) 
	{
        if (calculateRRTStarDistance(node->config, expandedNode->config, numofDOFs) < rewiringRadius) 
		{
            neighboringNodes.push_back(node);
        }
    }
    return neighboringNodes;
}

// Rewire the tree to optimize paths
void optimizeRRTStarTree(StarNode* expandedNode, const std::vector<StarNode*>& neighboringNodes, double expansionStep, double* map, int x_size, int y_size, int numofDOFs) {
    for (StarNode* neighbor : neighboringNodes) 
	{
        double newCost = expandedNode->pathCost + calculateRRTStarDistance(expandedNode->config, neighbor->config, numofDOFs);
        if (newCost < neighbor->pathCost && IsValidArmConfiguration(neighbor->config, numofDOFs, map, x_size, y_size)) 
		{
            neighbor->parent = expandedNode;
            neighbor->pathCost = newCost;
        }
    }
}

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
// {
//     /* TODO: Replace with your implementation */
//     planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
// }

{
    // Parameters
    int maxIterations = 100000;         // Maximum number of iterations
    double expansionStep = PI / 5.0;  // Step size for tree expansion
    double rewiringRadius = PI / 5.0;  // Search radius for rewiring nearby nodes

    // Random number generator setup
    std::srand(std::time(0));

    // Initialize the tree with the start node
    std::vector<StarNode*> rrtStarTree;
    StarNode* startNode = new StarNode(armstart_anglesV_rad, numofDOFs);
    startNode->pathCost = 0; // The start node has zero cost
    rrtStarTree.push_back(startNode);

    // RRT* main loop
    for (int iter = 0; iter < maxIterations; ++iter) 
	{
        // Randomly sample a configuration
        double* sampledConfig = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; ++i) 
		{
            sampledConfig[i] = (double)rand() / RAND_MAX * 2 * PI;
        }

        // Find the nearest node in the tree
        StarNode* closestNode = findClosestRRTStarNode(rrtStarTree, sampledConfig, numofDOFs);

        // Expand the tree towards the random sample
        StarNode* expandedNode = extendRRTStarTree(closestNode, sampledConfig, expansionStep, numofDOFs);

        // Check if the new configuration is valid
        if (IsValidArmConfiguration(expandedNode->config, numofDOFs, map, x_size, y_size)) 
		{
            rrtStarTree.push_back(expandedNode);

            // Rewire the tree within the search radius
            std::vector<StarNode*> neighboringNodes = getRRTStarNeighborNodes(rrtStarTree, expandedNode, rewiringRadius, numofDOFs);
            optimizeRRTStarTree(expandedNode, neighboringNodes, expansionStep, map, x_size, y_size, numofDOFs);

            // Check if the goal has been reached
            if (calculateRRTStarDistance(expandedNode->config, armgoal_anglesV_rad, numofDOFs) < expansionStep) 
			{
                std::cout << "Goal reached with RRT* Optimized!" << std::endl;

                // Trace back the path from goal to start
                std::vector<StarNode*> path;
                StarNode* current = expandedNode;
                while (current != nullptr) 
				{
                    path.push_back(current);
                    current = current->parent;
                }

                // Store the plan
                *planlength = path.size();
                *plan = (double**)malloc(*planlength * sizeof(double*));
                for (int i = *planlength - 1; i >= 0; --i) 
				{
                    (*plan)[*planlength - i - 1] = (double*)malloc(numofDOFs * sizeof(double));
                    for (int j = 0; j < numofDOFs; ++j) 
					{
                        (*plan)[*planlength - i - 1][j] = path[i]->config[j];
                    }
                }

                // Free memory used by the tree
                for (StarNode* node : rrtStarTree) delete node;
                delete[] sampledConfig;
                return;
            }
        }
        delete[] sampledConfig;
    }

    std::cout << "RRT* Optimized failed to find a path within the given iterations." << std::endl;

    // Free memory used by the tree
    for (StarNode* node : rrtStarTree) delete node;
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//


struct PRMNode 
{
    double* config;  
    std::vector<PRMNode*> neighbors; 

    PRMNode(double* config, int numofDOFs) 
	{
        this->config = new double[numofDOFs];
        for (int i = 0; i < numofDOFs; ++i) 
		{
            this->config[i] = config[i];
        }
    }

    ~PRMNode() 
	{
        delete[] config;
    }
};

std::pair<double, double> getEndEffectorPosition(double* config, int numofDOFs) 
{
    double x = 0.0;
    double y = 0.0;
    double cumulativeAngle = 0.0;

    // Iterate through each joint and link to compute the end-effector position
    for (int i = 0; i < numofDOFs; ++i) 
	{
        cumulativeAngle += config[i];  // Add the current joint angle to the cumulative angle
        x += LINKLENGTH_CELLS * cos(cumulativeAngle);  // x position
        y += LINKLENGTH_CELLS * sin(cumulativeAngle);  // y position
    }

    return std::make_pair(x, y);  // Return the (x, y) coordinates of the end-effector
}

double PRMCalculateDistance(double* config1, double* config2, int numofDOFs) 
{
    double distance = 0.0;
    for (int i = 0; i < numofDOFs; ++i) 
	{
        distance += (config1[i] - config2[i]) * (config1[i] - config2[i]);
    }
    return sqrt(distance);
}

// Compare configurations with a tolerance
bool PRMConfigurationsEqual(double* config1, double* config2, int numofDOFs, double tolerance = 1e-3) 
{
    for (int i = 0; i < numofDOFs; ++i) 
	{
        if (fabs(config1[i] - config2[i]) > tolerance) 
		{
            return false;
        }
    }
    return true;
}

// Sample random configurations
std::vector<PRMNode*> PRMSampleConfigurations(int numSamples, int numofDOFs, double* map, int x_size, int y_size) 
{
    std::vector<PRMNode*> nodes;

    for (int i = 0; i < numSamples; ++i) 
	{
        double* config = new double[numofDOFs];
        for (int j = 0; j < numofDOFs; ++j) 
		{
            config[j] = (double)rand() / RAND_MAX * 2 * PI;
        }

        // Collision-free?
        if (IsValidArmConfiguration(config, numofDOFs, map, x_size, y_size)) 
		{
            nodes.push_back(new PRMNode(config, numofDOFs));
        } else 
		{
            delete[] config;
        }
    }

    return nodes;
}

void PRMConnectNodes(std::vector<PRMNode*>& nodes, double radius, int numofDOFs, double* map, int x_size, int y_size) 
{
    for (PRMNode* node1 : nodes) 
	{
        for (PRMNode* node2 : nodes) 
		{
            if (node1 != node2 && PRMCalculateDistance(node1->config, node2->config, numofDOFs) < radius) 
			{
                // Convert configurations to 2D coordinates (end-effector positions)
                std::pair<double, double> pos1 = getEndEffectorPosition(node1->config, numofDOFs);
                std::pair<double, double> pos2 = getEndEffectorPosition(node2->config, numofDOFs);

                // Check if the path between the two configurations is valid
                if (IsValidLineSegment(pos1.first, pos1.second, pos2.first, pos2.second, map, x_size, y_size)) 
				{
                    node1->neighbors.push_back(node2);
                    node2->neighbors.push_back(node1);
                }
            }
        }
    }
}


// A* search 
std::vector<PRMNode*> PRMAStarSearch(PRMNode* startNode, PRMNode* goalNode, int numofDOFs) 
{
    // Priority queue - A* 
    std::priority_queue<std::pair<double, PRMNode*>, std::vector<std::pair<double, PRMNode*>>, std::greater<std::pair<double, PRMNode*>>> openSet;

    // Cost to reach 
    std::unordered_map<PRMNode*, double> gScore;
    gScore[startNode] = 0.0;

    // Parent node 
    std::unordered_map<PRMNode*, PRMNode*> cameFrom;

    // Heuristic - distance to the goal
    auto heuristic = [goalNode, numofDOFs](PRMNode* node) 
	{
        return PRMCalculateDistance(node->config, goalNode->config, numofDOFs);
    };

    openSet.push({heuristic(startNode), startNode});

    while (!openSet.empty()) 
	{
        PRMNode* current = openSet.top().second;
        openSet.pop();

        // If reached goal
        if (PRMConfigurationsEqual(current->config, goalNode->config, numofDOFs)) 
		{
            // Reconstruct path
            std::vector<PRMNode*> path;
            while (current != nullptr) 
			{
                path.push_back(current);
                current = cameFrom[current];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (PRMNode* neighbor : current->neighbors) 
		{
            double tentative_gScore = gScore[current] + PRMCalculateDistance(current->config, neighbor->config, numofDOFs);

            if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) 
			{
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                openSet.push({gScore[neighbor] + heuristic(neighbor), neighbor});
            }
        }
    }

    // If no path
    return {};
}

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
// {
//     /* TODO: Replace with your implementation */
//     planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
// }

{
    int numSamples = 10000000;    
    double connectionRadius = PI / 5.0; 

    std::srand(std::time(0));

    // Find configuration
    std::vector<PRMNode*> nodes = PRMSampleConfigurations(numSamples, numofDOFs, map, x_size, y_size);

    // Add start and goal
    PRMNode* startNode = new PRMNode(armstart_anglesV_rad, numofDOFs);
    nodes.push_back(startNode);

    PRMNode* goalNode = new PRMNode(armgoal_anglesV_rad, numofDOFs);
    nodes.push_back(goalNode);

    // Connect the nodes
    PRMConnectNodes(nodes, connectionRadius, numofDOFs, map, x_size, y_size);

    // A* search
    std::vector<PRMNode*> path = PRMAStarSearch(startNode, goalNode, numofDOFs);

    if (!path.empty()) {
        std::cout << "Path found using PRM!" << std::endl;

        // Store the plan
        *planlength = path.size();
        *plan = (double**)malloc(*planlength * sizeof(double*));
        for (int i = 0; i < *planlength; ++i) {
            (*plan)[i] = (double*)malloc(numofDOFs * sizeof(double));
            for (int j = 0; j < numofDOFs; ++j) {
                (*plan)[i][j] = path[i]->config[j];
            }
        }
    } else {
        std::cout << "PRM failed to find a path." << std::endl;
        *planlength = 0;
    }

    for (PRMNode* node : nodes) {
        delete node;
    }
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
	tie(map, x_size, y_size) = loadMap(mapFilePath);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
	string outputFile = outputDir + "/" + argv[6];
	std::cout << "Writing solution to: " << outputFile << std::endl;

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
