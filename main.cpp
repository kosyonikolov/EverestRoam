#include <algorithm>
#include <cassert>
#include <format>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <random>
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

struct Path
{
    std::vector<int> edges;
    float distance;
    int elevationGain;
    float cost;
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
    float maxPrintKm = 500;
    bool printOnBest = true;

    struct
    {
        // Vertical meter's cost is 1
        float linearKmCost = 25;
        // Bonus points for visiting many places
        float uniqueVertexCost = -2000;
        std::vector<float> edgeRepeatPenalty = {500, 1000};
        std::vector<int> vertexRepeatPenalty = {0, 500, 1000};
    } cost;

    struct
    {
        int minLength = 2;
        int maxLength = 4;
    } tailCycle;
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

int countUniqueVertices(const Graph & graph, const std::vector<int> & path)
{
    std::vector<bool> visited(graph.nVerts, false);
    if (!path.empty())
    {
        const auto & first = graph.edges[path[0]];
        visited[first.src] = true;
        for (int i = 0; i < path.size(); i++)
        {
            const int v = graph.edges[path[i]].dst;
            visited[v] = true;
        }
    }
    const int nVisited = std::count(visited.begin(), visited.end(), true);
    return nVisited;
}

void printGraphPath(const Graph & graph, const std::vector<int> & path, std::ostream & stream)
{
    std::stringstream ss;
    if (!path.empty())
    {
        const auto & first = graph.edges[path[0]];
        ss << graph.vertex2Name[first.src];
        for (int i = 0; i < path.size(); i++)
        {
            const auto & e = graph.edges[path[i]];
            const int v = e.dst;
            const bool isTrain = e.distance < 1;
            ss << (isTrain ? " >>> " : " - ") << graph.vertex2Name[v];
        }
    }

    stream << ss.str();
}

void printGraphPath(const Graph & graph, const Path & path, std::ostream & stream)
{
    printGraphPath(graph, path.edges, stream);
    stream << "\n";
    stream << std::format("{} km / {} m\nCost = {}\n", path.distance, path.elevationGain, path.cost);
}

Graph buildGraph(const std::vector<FileEdge> & fileEdges)
{
    Graph result;

    auto getId = [&](const std::string & name) -> int
    {
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

bool checkForTailCycles(const std::vector<int> & vertexPath, const int minLength, const int maxLength)
{
    const int n = vertexPath.size();
    for (int k = minLength; k <= maxLength; k++)
    {
        if (2 * k > n)
        {
            break;
        }
        const int i1 = n - k;
        const int i2 = i1 - k;
        bool isLoop = true;
        for (int j = 0; j < k; j++)
        {
            if (vertexPath[i1 + j] != vertexPath[i2 + j])
            {
                isLoop = false;
                break;
            }
        }

        if (isLoop)
        {
            return true;
        }
    }

    return false;
}

std::optional<Path> findPath(const Graph & graph, const int start, const float distanceTarget, const int elevTarget,
                             const SearchConfig & cfg, const float maxCost, const int end = -1)
{
    std::vector<int> pathEdges;
    std::vector<int> pathVertices;
    std::vector<int> edgeCounts(graph.edges.size(), 0);
    std::vector<int> vertexCounts(graph.nVerts, 0);
    struct DfsElem
    {
        int v;
        float dist = 0;
        int elevGain = 0;
        float repeatPenalty = 0;
        int i = 0; // Last used edge

        float calcCost(const SearchConfig & cfg) const
        {
            return elevGain + dist * cfg.cost.linearKmCost + repeatPenalty;
        }
    };

    std::stack<DfsElem> stack;
    stack.push({start});
    pathVertices.push_back(start);
    vertexCounts[start]++;

    auto pop = [&]()
    {
        stack.pop();
        pathVertices.pop_back();
        if (!pathEdges.empty())
        {
            int lastEdge = pathEdges.back();
            edgeCounts[lastEdge]--;
            vertexCounts[graph.edges[lastEdge].dst]--;
            pathEdges.pop_back();
        }
    };

    const int edgeCountLimit = cfg.cost.edgeRepeatPenalty.size();
    const int vertexCountLimit = cfg.cost.vertexRepeatPenalty.size() - 1;
    while (!stack.empty())
    {
        auto & curr = stack.top();

        const float currCost = curr.calcCost(cfg);
        // Check if we have exceeded the max cost
        if (currCost > maxCost)
        {
            pop();
            continue;
        }

        // Check if we've made an illegal tail cycle
        const bool haveTailCycle = checkForTailCycles(pathVertices, cfg.tailCycle.minLength, cfg.tailCycle.maxLength);
        if (haveTailCycle)
        {
            pop();
            continue;
        }

        // Check if we are done
        if (curr.dist >= distanceTarget && curr.elevGain >= elevTarget && (end < 0 || curr.v == end))
        {
            Path result;
            result.edges = pathEdges;
            result.cost = currCost;
            result.distance = curr.dist;
            result.elevationGain = curr.elevGain;
            return result;
        }

        // Check if we have edges we can visit
        const int v = curr.v;
        const auto & edgeIds = graph.localEdges[v];
        int i = curr.i;
        int nextEdgeIdx = -1;
        for (; i < edgeIds.size(); i++)
        {
            const int edgeIdx = edgeIds[i];
            if (edgeCounts[edgeIdx] < edgeCountLimit && vertexCounts[graph.edges[edgeIdx].dst] < vertexCountLimit)
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

            // Calc edge penalty
            const int newEdgeCnt = ++edgeCounts[nextEdgeIdx];
            const auto & penaltyLut = cfg.cost.edgeRepeatPenalty;
            if (newEdgeCnt >= 2 && !penaltyLut.empty())
            {
                const int repIdx = newEdgeCnt - 2;
                const int penalty = penaltyLut[std::min<int>(repIdx, penaltyLut.size() - 1)];
                next.repeatPenalty += penalty;
            }

            // Calc vertex penalty
            const int newVertexCnt = ++vertexCounts[next.v];
            const auto & vertPenaltyLut = cfg.cost.vertexRepeatPenalty;
            if (newVertexCnt >= 2 && !vertPenaltyLut.empty())
            {
                const int repIdx = newVertexCnt - 2;
                const int penalty = vertPenaltyLut[std::min<int>(repIdx, vertPenaltyLut.size() - 1)];
                next.repeatPenalty += penalty;
            }

            stack.push(next);
            pathVertices.push_back(next.v);
            pathEdges.push_back(nextEdgeIdx);
        }
        else
        {
            // We've ran out of edges for this vertex
            // Time to remove it from the stack - also remove the edge that lead us here
            pop();
        }
    }

    return {};
}

int main(int argc, char ** argv)
{
    const std::string usageMsg = "./EverestRoam <graph file> <start> [end]";
    if (argc < 3)
    {
        std::cerr << usageMsg << "\n";
        return 1;
    }

    const std::string graphFileName = argv[1];
    const std::string startName = argv[2];
    const std::string endName = argc > 3 ? argv[3] : "";

    const auto fileEdges = readGraphFile(graphFileName);
    auto graph = buildGraph(fileEdges);
    const bool randomize = true;
    if (randomize)
    {
        std::cout << "Randomizing edge order...\n";
        std::default_random_engine rng(std::random_device{}());
        for (auto & localEdges : graph.localEdges)
        {
            std::shuffle(localEdges.begin(), localEdges.end(), rng);
        }
    }

    const auto itStart = graph.name2Vertex.find(startName);
    if (itStart == graph.name2Vertex.end())
    {
        std::cerr << std::format("Start name [{}] not found in graph!\n", startName);
        return 1;
    }
    const int idStart = itStart->second;

    int idEnd = -1;
    if (endName != "")
    {
        const auto itEnd = graph.name2Vertex.find(endName);
        if (itEnd == graph.name2Vertex.end())
        {
            std::cerr << std::format("End name [{}] not found in graph!\n", endName);
            return 1;
        }
        idEnd = itEnd->second;
    }

    const float distanceTarget = 400; // km
    const int elevTarget = 10000;     // m

    SearchConfig searchCfg;
    float lowCost = distanceTarget * searchCfg.cost.linearKmCost + elevTarget;

    std::optional<Path> bestPath;

    // Find first cost limit that works
    const float scale = 2;
    float highCost = lowCost * scale;
    const int maxIters = 20;
    for (int i = 0; i < maxIters; i++)
    {
        std::cout << std::format("Trying cost = {}\n", highCost);
        auto maybePath = findPath(graph, idStart, distanceTarget, elevTarget, searchCfg, highCost, idEnd);
        if (maybePath)
        {
            std::cout << std::format("Found possible path at {}\n", highCost);
            printGraphPath(graph, maybePath.value(), std::cout);
            bestPath = maybePath;
            break;
        }
        highCost *= scale;
    }

    if (!bestPath)
    {
        std::cout << "No possible path!\n";
        return 0;
    }

    std::cout << std::format("Search range: [{}, {}]\n", lowCost, highCost);
    const float eps = 50;
    lowCost -= 1;
    // Low cost is not possible, high cost is possible
    while (highCost - lowCost > eps)
    {
        const float midCost = 0.5 * (highCost + lowCost);
        std::cout << std::format("Trying cost = {}\n", midCost);
        auto maybePath = findPath(graph, idStart, distanceTarget, elevTarget, searchCfg, midCost, idEnd);
        if (maybePath)
        {
            std::cout << std::format("Found possible path at {}\n", midCost);
            printGraphPath(graph, maybePath.value(), std::cout);
            highCost = midCost;

            if (maybePath->cost < bestPath->cost)
            {
                bestPath = maybePath;
            }
        }
        else
        {
            lowCost = midCost;
        }
        std::cout << std::format("Search range: [{}, {}]\n", lowCost, highCost);
    }

    std::cout << "\n====== Best path ======\n";
    printGraphPath(graph, bestPath.value(), std::cout);

    return 0;
}
