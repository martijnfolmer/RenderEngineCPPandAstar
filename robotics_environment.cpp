// robotics_environment.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

/*

    This is a project to create a render engine inside of the command line using ascii symbols, then adding an Astar pathfinding functionality.
    The purpose of this project is educational (how do render engines work) and my own training of C++ skills. I highly recommend doing something
    like this if you are a programmer with some experience looking to increase your own c++ capabilities.

    If you set the variable PlayerControlled to true, you will be in control of the "robot"/"player" and be able to move around the environment
    using WASD. If you set it to false, the robot will attempt to find a path from its starting location to an ending location using a version of
    the AStar pathfinding algorithm

    Some improvements that I would make if I had more time:
    - multiple robots that have to avoid eachother whilst going about their business, which requires a more dynamic pathfinding solution
    - multiple tasks and multiple robots, so the challenge becomes assigning the task in such a way that it is done in the least amount of time
    - visualisation of the path inside of the render, so you can see where the robot intends to go.
    - Create a SLAM algorithm to have the robot try to find its own way
    - Variable speeds, so we have to take into account the speed up and slow down of the robot
    - robots which are not omni-directional and must take turning circles into account

    Feel free to use any part of this code for any and all purposes. The Astar class and autonomous movement is fully made by me and 
    the render engine part of this code is heavily inspired by the algorithms shown in a video from javidx9, which can be found at the 
    following address : https://www.youtube.com/watch?v=xW8skO7MFYw&t=82s
    
    Author : Martijn Folmer
    Date : 14-05-2023

*/


#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <queue>
#include <stack>
#include <Windows.h>
#include <math.h>
using namespace std;

//IMPORTANT : this is the variable which controls whether you are driving around yourself, or whether you will get an autonomous robot. Set to either true or false
const bool PlayerControlled = false;     // if set to True, we do a player controlled (WASD) environment.
                                        // if set to False, the environment will center around a robot moving from one place to another

// Math constants
const float pi = 3.14159265358979323846;    

// Field variables and environment rendering variables
//Constants
const int nScreenWidth = 120;         // expected columns of the screen
const int nScreenHeight = 40;         // expected height of the screen

const int nMapHeight = 16;            // height of the map 
const int nMapWidth = 16;             // width of the map 

const float fFOV = 3.14159 / 4.0;     // field of view (total view is from -FOV/2 to FOV/2)
const float fDepth = 16.0f;           // how far we can look maximum

//Player values
float fPlayerX = 1.0f;                        // X location of the player
float fPlayerY = 1.0f;                        // Y location of the player
float fPlayerA = 0.0 * 3.14159 / 180;         // Angle that the player is facing (in rad)
float fPlayerV = 5.0*0.8f;                    // the speed of the player
float fPlayerAV = 1.2*0.8f;                   // how fast we turn

//define a structure to store coordinates
struct Vector2 {
    int x;      // x coordinate of the grid
    int y;      // y coordinate of the grid
    Vector2() : x(0), y(0) {};                      // default constructor
    Vector2(int x_, int y_) : x(x_), y(y_) {};      // initialization constructor
};

//player pathfinding
queue<Vector2> pathCur;     // the queue that gets updated by AstarPathfinding with the coordinates of our calculated path



/// <summary>
/// The is the Astar class, which we use to perform the a star pathfinding algorithm to find the shortest path between a start and end point
/// </summary>
class AstarPathfinding {

    public:
        
        // constructor
        AstarPathfinding() {
            std::cout << "We initialized the A star class\n";
        }
     
        ////////////////////////
        // Setters and Getters
        ////////////////////////
        void SetMap(wstring _mapString) {
            // Set the field we need for pathfinding, and then reset it.
            field = _mapString;
            resetAstarField();      // reset the Astar field so it is primed to be used for forward propagation
        }

        void SetStartCoor(int _xco, int _yco) {
            StartCoor.x = _xco;
            StartCoor.y = _yco; 
        }

        void SetEndCoor(int _xco, int _yco) {
            EndCoor.x = _xco;
            EndCoor.y = _yco;
        }

        queue<Vector2> GetPath() {
            return path;
        }


        /// <summary>
        /// Tun the Astar algorithm on the field so we find the shortest path between two points. Updates the Path queue, which we can retrieve using the
        /// getPath function
        /// </summary>
        void RunAstar() {
            // This version of Astar does not have early stopping, so it will explore the whole field. As long as start and end coordinates are
            // As long as start and end coordinates are within the grid, a path will be found. However, the cost might be so high 
            ForwardPropagation(astar_field, StartCoor);    // perform forward propagation
            BackwardPropagation(astar_field);              // perform backward propagation
            ReversePath();                                 // reverse the path, so we start at the starting coordinates and end at the ending coordinates
        }


        /// <summary>
        /// Testing function : visualisation. Show the Astar field, which we use to store the costs of getting places, calculated in forward propagation
        /// </summary>
        void PrintAstarField() {
            std::cout << "\n";
            for (int i = 0; i < nMapWidth; i++) {
                for (int j = 0; j < nMapHeight; j++) {
                    std::cout << astar_field[i][j] << "  ";                 // each value is -1, which means it hasn't yet been discovered
                }
                std::cout << "\n";
            }
        }



    private:
        wstring field;                          // The field representation (# is a wall, other characters are free spaces)
        int astar_field[nMapWidth][nMapHeight]; // The array which we use to store our costs (calculated in forwardPropagation)
        Vector2 StartCoor;                      // The start coordinates (where we come from)
        Vector2 EndCoor;                        // The end coordinates (where we want to go)

        
        // Borders if we allow both orthogonal and diagonal movement
        //const Vector2 borderCoordinates[8] = { Vector2(1,0), Vector2(1, -1), Vector2(0,-1), Vector2(-1, -1), Vector2(-1, 0), Vector2(-1,1), Vector2(0,1), Vector2(1,1)};
 
        // Borders if we allow only orthogonal movment
        const Vector2 borderCoordinates[4] = { Vector2(1,0), Vector2(0,-1), Vector2(-1, 0), Vector2(0,1) };


        queue<Vector2> gridsToCheck;    // the queue with the coordinates of cells that we want to check for our pathfinding in forwardPropagation
        queue<Vector2> path;            // the path we return (in the form of a queue)


        /// <summary>
        /// Ckear a queue data structure of all its contents
        /// </summary>
        /// <param name="q">the queue data structure we want to clear</param>
        void clear_queue(std::queue<Vector2>& q)
        {
            std::queue<Vector2> empty;
            std::swap(q, empty);
        }

        /// <summary>
        /// Reset the astar_field array, which turns all values into -1, meaning they are unexplored cells
        /// </summary>
        void resetAstarField() {
            // update our Astarfield so it has a -1 for each of the elements
            for (int i = 0; i < nMapWidth; i++) {
                for (int j = 0; j < nMapHeight; j++) {
                    astar_field[i][j] = -1;                 // each value is -1, which means it hasn't yet been discovered
                }
            }
        }


        /// <summary>
        /// Turn the xco and yco coordinates of a field into the appropriate location inside of the wstring (and read it)
        /// </summary>
        /// <param name="xco">The x coordinate of a cell</param>
        /// <param name="yco">The y coordinate of a cell</param>
        /// <returns></returns>
        wchar_t getFieldCharacter(int xco, int yco) {
            
            return field[yco * nMapWidth + xco];
        }


        /// <summary>
        /// Forward propagation of the Astar algorithm. Start from the starting cell, move outwards, assign costs to each cell based on how difficult/long it takes to get there
        
        /// </summary>
        /// <param name="astar_field_cur">The array that contains the costs we calculate.</param>
        /// <param name="startCoordinates">The coordinates of the cell where our path starts</param>
        void ForwardPropagation(int(&astar_field_cur)[nMapWidth][nMapHeight], Vector2 &startCoordinates) {
            
            astar_field_cur[startCoordinates.x][startCoordinates.y] = 0;        // set the starting elements to 0 (because that is where we start)
            Vector2 Start(startCoordinates.x, startCoordinates.y);              
            gridsToCheck.push(Start);                                           

            // As long as there are unexplored cells, we will continue to check for borders
            while (gridsToCheck.size() > 0) {

                Vector2 gridCur = gridsToCheck.front();
                gridsToCheck.pop();     // remove from the queue
                int CostCheck = astar_field_cur[gridCur.x][gridCur.y];      // the cost to get to this part

                // check all of the borders
                for (Vector2 coor : borderCoordinates) {
                    int grid_x = coor.x + gridCur.x;        // get the coordinates of the bordering grid
                    int grid_y = coor.y + gridCur.y;

                    // check if within range
                    if (grid_x >= 0 and grid_y >= 0 and grid_x < nMapWidth and grid_y < nMapHeight) {
                        // get the string associated with this
                        wchar_t borderCharacter = getFieldCharacter(grid_x, grid_y);
                        int borderCost = astar_field_cur[grid_x][grid_y];
                        
                        int bordercost_new = 0;
                        if (borderCharacter == '#') {
                            bordercost_new = CostCheck + 999999;    // Add a cost of 999999 to it, which will inform us that moving through here is not an option
                        }
                        else {
                            bordercost_new = CostCheck + 1;         // just a neighbouring border, so cost is a normal +1
                        }

                        // if we found a better cost, or it is an unexplored space
                        if (borderCost == -1 || bordercost_new < borderCost) {
                            astar_field_cur[grid_x][grid_y] = bordercost_new;
                            // append to the queue as a grid we need to check
                            Vector2 newGridToCheck(grid_x, grid_y);
                            gridsToCheck.push(newGridToCheck);
                        }
                    }
                }
            }
        }



        /// <summary>
        /// The backward propagation of the a star algorithm, using the costs array to find the cheapest route from the end coordinate to the start
        /// </summary>
        /// <param name="astar_field_cur"></param>
        void BackwardPropagation(int(&astar_field_cur)[nMapWidth][nMapHeight]) {

            // From the end coordinate, create a path, and append to the queue
            clear_queue(path);      // clear the path


            // add the end to the path
            int costCur = astar_field_cur[EndCoor.x][EndCoor.y];
            Vector2 PathCur(EndCoor.x, EndCoor.y);                  // This is where the path currently is
            path.push(PathCur);

            // Keep finding the next neighbour until we reach the cost of 0
            while (costCur > 0) {
                // find the lowest number amongs the borders
                int minCost = 9999999999999;
                Vector2 minCoor(0, 0);
                for (Vector2 coor : borderCoordinates) {
                    int grid_x = coor.x + PathCur.x;        // get the coordinates of the bordering grid
                    int grid_y = coor.y + PathCur.y;

                    // check if the cost of the bordering cell is the lowest it can be
                    int costCheck = astar_field_cur[grid_x][grid_y];
                    if (costCheck < minCost) {
                        minCost = costCheck;
                        minCoor.x = grid_x;
                        minCoor.y = grid_y;
                    }
                }
            
                // Add the new cells to the paths
                costCur = minCost;
                PathCur.x = minCoor.x;
                PathCur.y = minCoor.y;
                path.push(PathCur);
            }
        }

        /// <summary>
        /// Reverse the queue
        /// </summary>
        void ReversePath() {
            // Reverse our path queue, so our path starts at StartCoor and ends at EndCoor

            std::stack<Vector2> s;

            // pop elements from the queue and push them onto the stack
            while (!path.empty()) {
                s.push(path.front());
                path.pop();
            }

            // pop elements from the stack and push them back onto the queue
            while (!s.empty()) {
                path.push(s.top());
                s.pop();
            }
        }

};


// Find out which way the robot needs to point to move down the found path
double angleBetweenPoints(double x1, double y1, double x2, double y2) {
    double angle = atan2(y2-y1, x2-x1);
    if (angle < 0) {
        angle += 2 * pi;
    }

    angle = 5 * pi / 2 - angle;
    if (angle > 2 * pi) { angle -= 2 * pi; }
    if (angle < 0) { angle += 2 * pi; }

    return angle;

}




int main()
{
    // Create the screen buffer
    wchar_t* screen = new wchar_t[nScreenWidth * nScreenHeight];   //Setting the screen array. wchar_t = 16-bit wide character used to store Unicode encoded as UTF-16LE. * declares a pointer and allows you to refer directly to values in memory
    HANDLE hConsole = CreateConsoleScreenBuffer(GENERIC_READ | GENERIC_WRITE, 0, NULL, CONSOLE_TEXTMODE_BUFFER, NULL); // a HANDLE is a pointer to an object located on the GC (garbage collection) heap, 
    SetConsoleActiveScreenBuffer(hConsole); // Sets the hConsole as the target of our console
    DWORD dwBytesWritten = 0;   // DWORD = double word, 32 bit numbers (int)


    //create the map ("." means it is an empty space, "#"  means it is a wall)
    wstring map;
    map += L"################";
    map += L"#..............#";
    map += L"######.........#";
    map += L"#..............#";
    map += L"#....##........#";
    map += L"#....##........#";
    map += L"#.....#####....#";
    map += L"#.........###..#";
    map += L"#.........###..#";
    map += L"#..##.....###..#";
    map += L"#..##..........#";
    map += L"#..............#";
    map += L"#........#######";
    map += L"#..............#";
    map += L"#..............#";
    map += L"################";

    // Timers to check how much time has passed between frames
    auto tp1 = chrono::system_clock::now();
    auto tp2 = chrono::system_clock::now();


    // If playerControlled = false, means we make a robot that calculates and follows an autonomous path
    if (PlayerControlled == false) {
        // create an A pathfinding class and calculate the path we should take
        AstarPathfinding AStar;
        AStar.SetMap(map);                                      // Pass the map to the Astar class, so a path can be found
        AStar.SetStartCoor((int)fPlayerX, (int)fPlayerY);       // where the robot starts
        AStar.SetEndCoor(14, 14);                               // Where we want the robot to go
        AStar.RunAstar();                                       // run our A-star pathfinding, so we can 
        pathCur = AStar.GetPath();
        
        // TESTING : print the length of the path
        //std::cout << "The found path has a length of :" << pathCur.size();

        // TESTING: print the astar field
        // AStar.PrintAstarField();
        
    }

    // Gameloop
    while (1) {


        // We need a consistent frame rate, so we use the chrono clock to determine how much time has passed between frames
        tp2 = chrono::system_clock::now();
        chrono::duration<float> elapsedTime = tp2 - tp1;
        tp1 = tp2;                                                              
        float fElapsedTime = elapsedTime.count();           // A measure of how much time has passed


        // Make sure that fPlayerA is always between 0 and pi
        if (fPlayerA < 0) { fPlayerA += pi*2; }
        if (fPlayerA > pi*2) { fPlayerA -= pi*2; }

        // Playercontrolled, means we can move around freely in the environment
        if (PlayerControlled == true) {
            // Using User Input : AD for rotation, WS for walkding forwards and backwards
            
            // rotation
            if (GetAsyncKeyState((unsigned short)'A') & 0x8000) {
                fPlayerA -= (fPlayerAV) * fElapsedTime;
            }
            if (GetAsyncKeyState((unsigned short)'D') & 0x8000) {
                fPlayerA += (fPlayerAV) * fElapsedTime;
            }

            // Moving forward + collision detection
            if (GetAsyncKeyState((unsigned short)'W') & 0x8000) {
                fPlayerX += sinf(fPlayerA) * fPlayerV * fElapsedTime;   // move based on a direction by the player
                fPlayerY += cosf(fPlayerA) * fPlayerV * fElapsedTime;

                // collisions
                if (map[(int)fPlayerY * nMapWidth + (int)fPlayerX] == '#') {
                    fPlayerX -= sinf(fPlayerA) * fPlayerV * fElapsedTime;   // move based on a direction by the player
                    fPlayerY -= cosf(fPlayerA) * fPlayerV * fElapsedTime;
                }
            }
            // Moving backward + collision detection
            if (GetAsyncKeyState((unsigned short)'S') & 0x8000) {
                fPlayerX -= sinf(fPlayerA) * fPlayerV * fElapsedTime;   // move based on a direction by the player
                fPlayerY -= cosf(fPlayerA) * fPlayerV * fElapsedTime;

                // collisions
                if (map[(int)fPlayerY * nMapWidth + (int)fPlayerX] == '#') {
                    fPlayerX += sinf(fPlayerA) * fPlayerV * fElapsedTime;   // move based on a direction by the player
                    fPlayerY += cosf(fPlayerA) * fPlayerV * fElapsedTime;
                }
            }
        }
        else {
            // robot controlled, so we move from one place to another
            if (pathCur.size() > 0) {
                Vector2 CellGoal = pathCur.front();     // the next cell we want to go to
                float distance = std::sqrt(std::pow(fPlayerX - (float)CellGoal.x, 2) + std::pow(fPlayerY - (float)CellGoal.y, 2));
                
                // check if distance is smaller than what we want, so we can move on to the next thing
                if (distance < fPlayerV * fElapsedTime) {
                    fPlayerX = (float)CellGoal.x;   // move robot to the last part
                    fPlayerY = (float)CellGoal.y;
                    pathCur.pop();                  //remove from the queue
                }
                else {
                    // move robot angle towards where we want to go
                    float angleToGoal = angleBetweenPoints((double)fPlayerX, (double)fPlayerY, (double)CellGoal.x, (double)CellGoal.y);

                    if (angleToGoal != fPlayerA) {

                        // find the difference
                        float angDiff = angleToGoal - fPlayerA;
                        if (angDiff > pi) { angDiff -= pi*2; }
                        if (angDiff < -pi) { angDiff += pi*2; }


                        if (abs(angDiff) < fPlayerAV * fElapsedTime) {
                            fPlayerA = angleToGoal;
                        }
                        else {
                            if (angDiff < 0) {fPlayerA -= (fPlayerAV)*fElapsedTime;}
                            else { fPlayerA += (fPlayerAV)*fElapsedTime; }
                        }

                    }
                    // If we are facing the correct direction, move the robot
                    else {
                        fPlayerX += sinf(fPlayerA) * fPlayerV * fElapsedTime;   // move based on a direction by the player
                        fPlayerY += cosf(fPlayerA) * fPlayerV * fElapsedTime;

                        // collisions (should not happen if we did our pathplanning correctly)
                        if (map[(int)fPlayerY * nMapWidth + (int)fPlayerX] == '#') {
                            fPlayerX -= sinf(fPlayerA) * fPlayerV * fElapsedTime;   // move based on a direction by the player
                            fPlayerY -= cosf(fPlayerA) * fPlayerV * fElapsedTime;
                        }
                    }
                }
            }
        }

        // Do a computation for each column on the screen (from left to right)
        // Each column of the console will be related to a ray (shot out from the player/robots perspective)
        for (int x = 0; x < nScreenWidth; x++) {

            // find the angle that our raycast is shot out at. 
            float fRayAngle = (fPlayerA - fFOV / 2.0f) + ((float)x / (float)nScreenWidth) * fFOV;
            float fDistanceToWall = 0;
            bool bHitWall = false;  // whether we hit a wall
            bool bBoundary = false; //whether we hit the boundary of a wall (the edge of the cell)

            float fEyeX = sinf(fRayAngle);  // unit vector for ray in player spce
            float fEyeY = cosf(fRayAngle);

            while (!bHitWall && fDistanceToWall < fDepth) {

                fDistanceToWall += 0.1f;

                // where our test coordinates  are
                int nTestX = (int)(fPlayerX + fEyeX * fDistanceToWall);
                int nTestY = (int)(fPlayerY + fEyeY * fDistanceToWall);

                // If ray is out of bounds, it means we have found an edge
                if (nTestX < 0 || nTestX >= nMapWidth || nTestY < 0 || nTestY >= nMapHeight) {
                    bHitWall = true;
                    fDistanceToWall = fDepth;
                }
                else {
                    // Convert our nTextX and nTestY to the 1D array location of our map, and check if that location is an #, which would make it a wall
                    if (map[nTestY * nMapWidth + nTestX] == '#') {
                        bHitWall = true;
                    
                        // check if we hit the edge of a block
                        vector<pair<float, float>> p;       //distance, dot

                        // Get the perfect coordinates of the wall section we are looking at (offset from our player)
                        for (int tx = 0; tx < 2; tx++) {
                            for (int ty = 0; ty < 2; ty++) {
                                float vy = (float)nTestY + ty - fPlayerY;
                                float vx = (float)nTestX + tx - fPlayerX;
                                float d = sqrt(vx * vx + vy * vy);
                                float dot = (fEyeX * vx / d) + (fEyeY * vy / d);
                                p.push_back(make_pair(d, dot));
                            }
                        }
                        // Sort Pairs from closest to farthest
                        sort(p.begin(), p.end(), [](const pair<float, float>& left, const pair<float, float>& right) {return left.first < right.first; });
                    
                        float fBound = 0.01;    //if the angle is less than this, means we are at a corner
                        if (acos(p.at(0).second) < fBound) bBoundary = true;
                        if (acos(p.at(1).second) < fBound) bBoundary = true;
                        if (acos(p.at(2).second) < fBound) bBoundary = true;
                    }
                }
            }
        
            // Calculate distance to ceiling and floor (the farther away, the more ceiling and floor is visible, the closer, the less is visible)
            int nCeiling = (float)(nScreenHeight / 2.0) - nScreenHeight / ((float)fDistanceToWall); // midpoint - proportion of distancetowall
            int nFloor = nScreenHeight - nCeiling; // floor is a mirror of the ceiling
           
            // Get the shade of the wall, based on how far away it is
            short nShade = ' ';
            for (int y = 0; y < nScreenHeight; y++) {
                if (y < nCeiling) {
                    screen[y * nScreenWidth + x] = ' ';     // shade in ceiling with blank space
                }
                else if (y > nCeiling && y <= nFloor) {
                    if (fDistanceToWall <= fDepth / 4.0f) nShade = 0x2588;      // very close
                    else if (fDistanceToWall < fDepth / 3.0) nShade = 0x2593;
                    else if (fDistanceToWall < fDepth / 2.0) nShade = 0x2592;
                    else if (fDistanceToWall < fDepth) nShade = 0x2591;
                    else  nShade = ' ';                                         // far away

                    if (bBoundary) nShade = ' ';               //black it out if it is the edge of a wall (makes the walls clearer)
                    screen[y * nScreenWidth + x] = nShade;     // shade in wall with #
                }
                else {
                    // Shade floor based on distance
                    float b = 1.0f - (((float)y - nScreenHeight / 2.0f) / ((float)nScreenHeight / 2.0f));
                    if (b < 0.25) nShade = '#';
                    else if (b < 0.5) nShade = 'x';
                    else if (b < 0.75) nShade = '.';
                    else if (b < 0.9) nShade = '-';
                    else nShade = ' ';
                    screen[y * nScreenWidth + x] = nShade;     // shade in with blank.
                }
            }
        }

        // Display stats, such as player/robot location, angle they are facing, and current FPS
        swprintf_s(screen, 40, L"X=%3.2f, Y=%3.2f, A=%3.2f FPS=%3.2f ", fPlayerX, fPlayerY, fPlayerA, 1.0f / fElapsedTime);

        // Display Map
        for (int nx = 0; nx < nMapWidth; nx++)
            for (int ny = 0; ny < nMapWidth; ny++)
            {
                screen[(ny + 1) * nScreenWidth + nx] = map[ny * nMapWidth + nx];
            }
        screen[((int)fPlayerY + 1) * nScreenWidth + (int)fPlayerX] = 'P';   // draw where the player/robot currently is
        screen[nScreenWidth * nScreenHeight - 1] = '\0';                    // Give a character to the entire screen array
        WriteConsoleOutputCharacter(hConsole, screen, nScreenWidth * nScreenHeight, { 0,0 }, &dwBytesWritten); // {0, 0} = coordinate of where we want to write
    }


    return 0;
}

