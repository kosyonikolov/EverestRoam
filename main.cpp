#include <algorithm>
#include <cassert>
#include <format>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <string>
#include <vector>

struct Edge
{
    int src, dst;
    float distance;
    int elevationGain; // Non-negative
};

struct Graph
{
    int nVerts;
    std::map<std::string, int> name2Vertex;
    std::vector<std::string> vertex2Name;
    std::vector<Edge> edges;
    std::vector<std::vector<int>> localEdges; // per-vertex
};

struct FileEdge
{
    std::string src, dst;
    float distance; // km
    // As reported by Komoot - they switch places when (src, dst) is flipped
    int ascent;  // m
    int descent; // m
};

struct SearchConfig
{
    int minEdgeRepeatDistance = 6;
    float maxPrintKm = 600;
};

std::vector<FileEdge> readGraphFile(const std::string & fileName)
{
    std::ifstream file(fileName);
    if (!file.is_open())
    {
        throw std::runtime_error(std::format("Failed to open graph file [{}]", fileName));
    }

    std::vector<FileEdge> result;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }
        std::istringstream iss(line);
        FileEdge protoEdge;
        if (iss >> protoEdge.src >> protoEdge.dst >> protoEdge.distance >> protoEdge.ascent >> protoEdge.descent)
        {
            result.push_back(protoEdge);
        }
        else
        {
            std::cerr << std::format("Failed to parse line [{}], ignoring\n", line);
        }
    }

    return result;
}

void printGraphPath(const Graph & graph, const std::vector<int> & path, std::ostream & stream)
{
    // Count how many vertices get visited
    std::vector<bool> visited(graph.nVerts, false);
    std::stringstream ss;
    if (!path.empty())
    {
        const auto & first = graph.edges[path[0]];
        ss << graph.vertex2Name[first.src];
        visited[first.src] = true;
        for (int i = 0; i < path.size(); i++)
        {
            const int v = graph.edges[path[i]].dst;
            ss << " - " << graph.vertex2Name[v];
            visited[v] = true;
        }
    }

    const int nVisited = std::count(visited.begin(), visited.end(), true);
    stream << std::format("[{}] {}", nVisited, ss.str());
}

void findAllPaths(const Graph & graph, const int start, const float distanceTarget, const int elevTarget,
                  const SearchConfig & cfg, std::ostream & stream)
{
    std::vector<int> path;
    struct DfsElem
    {
        int v;
        float dist;
        int elevGain;
        int i = 0; // Last used edge
    };

    std::stack<DfsElem> stack;
    stack.push({start, 0, 0, 0});

    while (!stack.empty())
    {
        auto & curr = stack.top();

        // Check if we are done
        if (curr.dist >= distanceTarget && curr.elevGain >= elevTarget)
        {
            if (curr.dist <= cfg.maxPrintKm)
            {
                // Avoid routes that are too flat
                printGraphPath(graph, path, stream);
                stream << std::format("\n{} km / {} m", curr.dist, curr.elevGain);
                stream << std::endl;
            }
            
            stack.pop();
            path.pop_back();
            continue;
        }

        const int forbiddenIdx0 = std::max<int>(path.size() - cfg.minEdgeRepeatDistance, 0);

        // Check if we have edges we can visit
        const int v = curr.v;
        const auto & edgeIds = graph.localEdges[v];
        int i = curr.i;
        int nextEdgeIdx = -1;
        for (; i < edgeIds.size(); i++)
        {
            const int edgeIdx = edgeIds[i];
            auto it = std::find(path.begin() + forbiddenIdx0, path.end(), edgeIdx);
            if (it == path.end())
            {
                // Valid edge
                nextEdgeIdx = edgeIdx;
                break;
            }
        }

        if (nextEdgeIdx >= 0)
        {
            // There is a valid edge - travel along it
            // Increase the current element's counter so that we don't use it again
            curr.i = i + 1;
            const auto & e = graph.edges[nextEdgeIdx];
            DfsElem next;
            next.v = e.dst;
            next.dist = curr.dist + e.distance;
            next.elevGain = curr.elevGain + e.elevationGain;
            stack.push(next);
            path.push_back(nextEdgeIdx);
        }
        else
        {
            // We've ran out of edges for this vertex
            // Time to remove it from the stack - also remove the edge that lead us here
            stack.pop();
            if (!path.empty())
            {
                path.pop_back();
            }
        }
    }
}

Graph buildGraph(const std::vector<FileEdge> & fileEdges)
{
    Graph result;

    auto getId = [&](const std::string & name) -> int {
        auto it = result.name2Vertex.find(name);
        if (it != result.name2Vertex.end())
        {
            return it->second;
        }
        const int idx = result.vertex2Name.size();
        result.vertex2Name.push_back(name);
        result.name2Vertex[name] = idx;
        return idx;
    };

    // Add all edges first
    for (const auto & fe : fileEdges)
    {
        const int src = getId(fe.src);
        const int dst = getId(fe.dst);
        Edge fwdEdge{src, dst, fe.distance, fe.ascent};
        Edge revEdge{dst, src, fe.distance, fe.descent};
        result.edges.push_back(fwdEdge);
        result.edges.push_back(revEdge);
    }

    // Create vertices and assign their edges
    const int nV = result.vertex2Name.size();
    result.nVerts = nV;
    result.localEdges.resize(nV);

    const int nE = result.edges.size();
    for (int i = 0; i < nE; i++)
    {
        const auto & e = result.edges[i];
        result.localEdges[e.src].push_back(i);
    }

    return result;
}

int main(int argc, char ** argv)
{
    const std::string usageMsg = "./EverestRoam <graph file> <start>";
    if (argc != 3)
    {
        std::cerr << usageMsg << "\n";
        return 1;
    }

    const std::string graphFileName = argv[1];
    const std::string startName = argv[2];

    const auto fileEdges = readGraphFile(graphFileName);
    const auto graph = buildGraph(fileEdges);

    const auto itStart = graph.name2Vertex.find(startName);
    if (itStart == graph.name2Vertex.end())
    {
        std::cerr << std::format("Start name [{}] not found in graph!\n", startName);
        return 1;
    }
    const int idStart = itStart->second;

    SearchConfig searchCfg;
    findAllPaths(graph, idStart, 400, 10000, searchCfg, std::cout);

    return 0;
}
