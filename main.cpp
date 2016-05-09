//
//  main.cpp
//  TrainRoute
//
//  Created by Adam Bouzourene on 12/6/15.
//  Copyright © 2015 Adam Bouzourene. All rights reserved.

#include <iostream>
#include <list>
#include <queue>
#include <stack>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <limits>

using namespace std;

const int DEFAULT_VAL =  -1; //global variable for default distance in BFT Search


class subwaySystem {
public:
    struct trainStop; //forward declaration for type definition
    
    typedef unordered_map<trainStop*, list<trainStop*>*> Graph; //first type: stops are vertices of the graph, second type: lists of edges for each vertices. adding a transfer through the trainStop class adds an edge here and vice versa
    
    struct coordinate { //simple coordinate struct for stop locations
        float x;
        float y;
    };
    
    float calcDist (const coordinate& c1, const coordinate& c2) { return sqrt(abs(c1.x - c2.x) + abs(c1.y - c2.y)); } //calculates distance between two coordinates using quadratic formula.
    
    struct trainStop { //represents mta stop and holds relevant data (name, location, etc)
        string id;
        string name;
        coordinate loc;
        list<trainStop*> transfers;
        
        
        trainStop(const string stop_id, const string stop_name, const float stop_lat, const float stop_lon) {
            id = stop_id;
            name = stop_name;
            loc.x = stop_lat;
            loc.y = stop_lat;
        }
    };
    
    struct vertexInf    // Stores information for a vertex
    {
        int dist;   // distance to vertex from the source
        trainStop* prev;    // previous node in BFS tree
    };
    
private:
    Graph stops; //vertex graph for transit system
    
public:
    subwaySystem(){}
    
    void addStop(trainStop* stop) { //can be used even if transfers have not been assigned yet
        stops[stop] = &stop->transfers;
    }
    
    bool hasStop(trainStop* stop) {
        return (stops.find(stop) != stops.end());
    }
    
    trainStop* getStop(const string& id) { //takes in stop id which is unique
        for (Graph::iterator itr = stops.begin(); itr != stops.end(); itr++) {
            if (itr->first->id == id)
                return itr->first;
        }
        return nullptr;
    }
    
    void addTransfer(trainStop* stop, trainStop* connection) { //adding a transfer here also adds an edge to the vertex in the graph
        stop->transfers.push_back(connection);
    }
    
    void printpath(trainStop* j, unordered_map<trainStop*, vertexInf>& vinfo)
    {
        stack<trainStop*> t; //holds path in FIFO order
        
        trainStop* current = j;
        while (current != nullptr) //fills stack with all stops in path
        {
            t.push(current);
            current = vinfo[current].prev;
        }
        while (!t.empty()) //prints out stack of stops until empty
        {
            cout << "[" <<t.top()->name << "(" << t.top()->id << ")" << "] -> ";
            t.pop();
        }
    }
    
    trainStop* findClosestStop(const coordinate& user) { //find nearest train station in graph to given geographic coordinates
        float min = numeric_limits<float>::max();
        float dist = 0;
        trainStop* closest = nullptr;
        for (Graph::iterator itr = stops.begin(); itr != stops.end(); itr++) { //loops through entire graph for linear runtime. should be quadtree since all coordinates stay the same for all queries know but didnt want to make whole implementation
            dist = calcDist(user, itr->first->loc) < min;
            if (dist <= min) {
                min = dist;
                closest = itr->first;
            }
        }
        return closest;
    }
    
    // Breadth First Search
    // The unweighted shortest path algorithm on the graph stops, with vertex src as the source
    // Prints the distance (number of edges) of the shortest path from the source stop to every possible stop in the system
    
    void shortestpaths(trainStop* src)
    {
        if (!hasStop(src)) {
            cout << "invalid source stop choice" << endl;
            return;
        }
        
        queue<trainStop*> vertQ;             // vertQ is the queue of stops to be analyzed
        
        unordered_map<trainStop*, vertexInf> verts(stops.size());               // stores transfer info for the stops
        
        for (Graph::const_iterator j = stops.begin(); j != stops.end(); j++)                 // Initialize distances and prev values for transit system
        {
            verts[j->first].dist = DEFAULT_VAL;
            verts[j->first].prev = nullptr;
        }
        
        
        verts[src].dist = 0; //set source stop to have zero distance
        
        vertQ.push(src); // start tree with source stop
        while  (!vertQ.empty() )
        {
            trainStop* cVert = vertQ.front();
            vertQ.pop();
            for (list<trainStop*>::const_iterator trf = stops[cVert]->begin(); trf != stops[cVert]->end(); trf++)
            {
                
                if (verts[*trf].dist == DEFAULT_VAL)          // test if distance of transfer from source not determined yet
                {
                    verts[*trf].dist = verts[cVert].dist + 1;
                    verts[*trf].prev = cVert;
                    vertQ.push(*trf);
                }
            }
        }
        
        cout << "----------- Shortest Path to " << src->name << "(" << src->id << ")" << " -----------" << endl;
        for (unordered_map<trainStop*, vertexInf>::iterator j = verts.begin(); j != verts.end(); j++)        // print distances from source and paths
        {
            if (j->second.dist != DEFAULT_VAL) {
                cout << "From: " << j->first->name << "(" << j->first->id << ")" << endl;
                cout << "Distance: " << j->second.dist << " Transfers" << endl;
                cout << "Shortest Path: ";
                printpath(j->first, verts);
                cout << endl << endl;
            }
        }
    }
};







vector<string> getLines(const string& filename) //returns vector of strings from passed filename
{
    string iline;
    vector<string> text;
    ifstream ifile(filename);
    if((ifile.is_open()))
    {
        while (getline(ifile,iline))
        {
            text.push_back(iline);
        }
        ifile.close();
    }
    else
    {
        cerr << "not valid";
    }
    
    return text;
}

subwaySystem::trainStop* addNewStop(const string& lineIn) //used to parse from 'stops' text file
{
    vector<string> strings;
    istringstream sline(lineIn);
    string s;
    while (getline(sline, s, ',')) {
        strings.push_back(s);
    }
    float lat = stof(strings[4]);
    float lon = stof(strings[5]);
    subwaySystem::trainStop* temp = new subwaySystem::trainStop(strings[0], strings[2], lat, lon);
    return temp;
}

void addToMap(const string& lineIn, subwaySystem& smap) //used to parse from 'transfers' text file
{
    vector<string> strings;
    vector<string> connections;
    istringstream sline(lineIn);
    string s;
    while (getline(sline, s, ',')) {
        strings.push_back(s);
    }
    smap.addTransfer(smap.getStop(strings[0]), smap.getStop(strings[1])); //all initialization of stops is done through main transit system class
}







void printClosestStop(const subwaySystem::coordinate& loc, subwaySystem& smap) { //tests get closest stop
    if (smap.findClosestStop(loc) != nullptr)
        cout << smap.findClosestStop(loc)->name << endl;
}

void testTrainClasses() { //driver function for class
    subwaySystem nyc;
    string stopsStr;
    string transStr;
    vector<string> fText;
    
    cout << "enter stops filename: ";
    stopsStr = "input.txt";//cin >> stopsStr;
    cout << endl << "enter transfers filename: ";
    transStr = "transfers.txt";//cin >> transStr;
    cout << endl << endl;
    
    fText = getLines(stopsStr);
    
    fText.erase(fText.begin());
    
    for (size_t x = 0; x < fText.size(); x++) {
        nyc.addStop(addNewStop(fText[x]));
    }
    fText.clear();
    fText = getLines(transStr);
    fText.erase(fText.begin());
    for (size_t x = 0; x < fText.size(); x++) {
        addToMap(fText[x], nyc);
    }
    subwaySystem::trainStop* kek = nyc.getStop("127");
    nyc.shortestpaths(kek);
    subwaySystem::coordinate currentLoc;
    currentLoc.x = 40.742587;
    currentLoc.y = -73.936997;
    printClosestStop(currentLoc, nyc);
}


int main()
{
    testTrainClasses();
}