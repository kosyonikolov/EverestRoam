#include <format>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
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
        Edge fwdEdge {src, dst, fe.distance, fe.ascent};
        Edge revEdge {dst, src, fe.distance, fe.descent};
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

    return 0;
}
